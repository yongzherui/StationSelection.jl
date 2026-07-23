"""
Solve fixed assignments captured by a BendersYZH diagnostic dump as standalone
`RouteCoveringProblem`s under several route stop limits.

This is deliberately not a Benders experiment: station choices and assignments
are held fixed, and only the route-column stop limit changes.

Usage:

    julia --project=. scripts/analyze_fixed_route_covering_stop_limits.jl \
        <dump.jls> <data_dir> <output_dir> [iterations]

`iterations` is a comma-separated list (default: `14,last`). The experiment
uses stop limits 4, 5, 6, and the model's uncapped sentinel.
"""

using CSV
using DataFrames
using Gurobi
using JuMP
using Serialization
using StationSelection

include(joinpath(@__DIR__, "aggregate_od_route_method_grid.jl"))

length(ARGS) >= 3 || error(
    "Usage: analyze_fixed_route_covering_stop_limits.jl " *
    "<dump.jls> <data_dir> <output_dir> [iterations]",
)

const DUMP_PATH = ARGS[1]
const DATA_DIR = ARGS[2]
const OUTPUT_DIR = ARGS[3]
const ITERATION_SPEC = length(ARGS) >= 4 ? ARGS[4] : "14,last"

function requested_iterations(spec::AbstractString, n_iterations::Int)
    values = Int[]
    for token in split(spec, ',')
        stripped = strip(token)
        push!(values, stripped == "last" ? n_iterations : parse(Int, stripped))
    end
    values = unique(values)
    all(i -> 1 <= i <= n_iterations, values) ||
        error("requested iterations $values outside dump range 1:$n_iterations")
    return values
end

function route_problem(assignments, open_stations, max_stops)
    return RouteCoveringProblem(
        length(open_stations),
        open_stations,
        assignments;
        assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
        route_regularization_weight=10.0,
        walk_cost_weight=0.1,
        repositioning_time=20.0,
        max_walking_distance=600.0,
        max_wait_time=900.0,
        detour_factor=2.0,
        max_stops=max_stops,
        max_new_columns=20,
        n_candidates=20,
        pricing_time_limit_sec=120.0,
    )
end

function selected_route_rows(case_iteration, stop_label, cg_result)
    rows = NamedTuple[]
    theta = cg_result.final_result.model[:theta_compat]
    columns = Dict(c.id => c for c in cg_result.final_result.mapping.columns)
    for ((column_id, scenario), variable) in sort!(collect(theta); by=first)
        activation = value(variable)
        activation > 0.5 || continue
        column = columns[column_id]
        route = get(column.metadata, "route", ())
        push!(rows, (
            case_iteration=case_iteration,
            stop_limit=stop_label,
            scenario=scenario,
            column_id=column_id,
            activation=activation,
            tau=column.tau,
            route_cost=10.0 * (column.tau + 20.0),
            route_length=length(route),
            n_certified_pairs=length(column.od_pairs),
            route=string(route),
            certified_pairs=string(Tuple(column.od_pairs)),
        ))
    end
    return rows
end

function main()
    dump = deserialize(DUMP_PATH)
    iterations = requested_iterations(ITERATION_SPEC, length(dump))
    data, _max_walk = build_instance("zhuzhou", 10, 32, 42, DATA_DIR)
    optimizer_env = Gurobi.Env()
    summary_rows = NamedTuple[]
    route_rows = NamedTuple[]

    for iteration in iterations
        record = dump[iteration]
        assignments = record.assignments
        open_stations = sort!(unique!(Int[v for pair in values(assignments) for v in pair]))
        length(open_stations) == 5 || error(
            "iteration $iteration assignment uses $(length(open_stations)) stations, expected 5",
        )

        for (stop_label, max_stops) in [
            ("4", 4),
            ("5", 5),
            ("6", 6),
            ("uncapped", nothing),
        ]
            problem = route_problem(assignments, open_stations, max_stops)
            cg_result = run_aggregate_od_route_column_generation(
                problem,
                data;
                optimizer_env=optimizer_env,
                verbose=false,
                max_cg_iters=10_000,
                max_new_columns=20,
                n_candidates=20,
                pricing_time_limit_sec=120.0,
                ip_time_limit_sec=300.0,
                mip_gap=1e-4,
            )
            ip_value = cg_result.final_result.objective_value
            lp_value = cg_result.lp_bound
            gap = (isnothing(ip_value) || !isfinite(lp_value)) ? NaN :
                (ip_value - lp_value) / max(abs(ip_value), 1e-9)
            push!(summary_rows, (
                case_iteration=iteration,
                stop_limit=stop_label,
                open_stations=string(Tuple(open_stations)),
                n_assignments=length(assignments),
                cg_status=string(cg_result.status),
                cg_stop_reason=string(cg_result.cg_stop_reason),
                cg_iterations=cg_result.n_cg_iters,
                generated_columns=length(cg_result.generated_columns),
                lp_objective=lp_value,
                ip_objective=ip_value,
                lp_ip_gap=gap,
            ))
            append!(route_rows, selected_route_rows(iteration, stop_label, cg_result))
        end
    end

    mkpath(OUTPUT_DIR)
    CSV.write(joinpath(OUTPUT_DIR, "fixed_assignment_stop_limit_summary.csv"), DataFrame(summary_rows))
    CSV.write(joinpath(OUTPUT_DIR, "fixed_assignment_selected_routes.csv"), DataFrame(route_rows))
end

main()
