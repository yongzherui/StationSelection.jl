"""
    analyze_rho_pair_station_selection.jl <output_dir> <run_dir> ...

Offline station-subset test on exhaustive pricing traces. Compares marginal rho,
theta load, pair-aware passenger coverage, travel-adjusted pair coverage, and
theta-core/pair hybrids. Writes snapshot- and method-level CSV summaries.
"""

using CSV, DataFrames, Statistics

length(ARGS) >= 2 || error(
    "usage: analyze_rho_pair_station_selection.jl <output_dir> <run_dir> ...",
)
const OUTDIR = ARGS[1]
const RUN_DIRS = ARGS[2:end]
const BETA = 10.0 # matches diag_y_support_churn.jl's route_regularization_weight

parse_set(value, separator=';') =
    Set(parse.(Int, filter(x -> !isempty(x), split(String(value), separator))))

function coverage_value(opportunities, selected::Set{Int}, travel_lambda::Float64)
    best = Dict{Tuple{Int, Int}, Float64}()
    for o in opportunities
        o.pickup in selected && o.dropoff in selected || continue
        reward = max(0.0, o.rho - travel_lambda * BETA * o.travel)
        key = (o.scenario, o.passenger)
        best[key] = max(get(best, key, 0.0), reward)
    end
    return sum(values(best); init=0.0)
end

function greedy_pair_subset(opportunities, n::Int, l::Int, travel_lambda::Float64;
                            fixed::Set{Int}=Set{Int}())
    selected = copy(fixed)
    if isempty(selected)
        best_pair = (1, min(2, n))
        best_value = -Inf
        for j in 1:n, k in (j + 1):n
            candidate = Set((j, k))
            value = coverage_value(opportunities, candidate, travel_lambda)
            if value > best_value + 1e-9
                best_pair, best_value = (j, k), value
            end
        end
        union!(selected, best_pair)
    end
    while length(selected) < l
        current = coverage_value(opportunities, selected, travel_lambda)
        candidates = [j for j in 1:n if !(j in selected)]
        isempty(candidates) && break
        gain, station = maximum(
            (coverage_value(opportunities, union(selected, Set((j,))), travel_lambda) - current, -j)
            for j in candidates
        )
        push!(selected, -station)
    end

    # One-for-one local improvement, preserving the theta-fixed core.
    improved = true
    while improved
        improved = false
        incumbent = coverage_value(opportunities, selected, travel_lambda)
        best_swap = nothing
        best_value = incumbent
        for out in setdiff(selected, fixed), incoming in setdiff(Set(1:n), selected)
            candidate = union(setdiff(selected, Set((out,))), Set((incoming,)))
            value = coverage_value(opportunities, candidate, travel_lambda)
            if value > best_value + 1e-9
                best_value = value
                best_swap = (out, incoming)
            end
        end
        if !isnothing(best_swap)
            delete!(selected, best_swap[1])
            push!(selected, best_swap[2])
            improved = true
        end
    end
    return selected
end

snapshot_rows = NamedTuple[]
for dir in RUN_DIRS
    for opportunity_path in sort!(filter(
        path -> endswith(path, "_opportunity_rhos.csv"), readdir(dir; join=true),
    ))
        case = replace(basename(opportunity_path), "_opportunity_rhos.csv" => "")
        prefix = joinpath(dir, case)
        opportunities_df = CSV.read(opportunity_path, DataFrame)
        priced = CSV.read(prefix * "_priced_routes.csv", DataFrame)
        station_scores = CSV.read(prefix * "_station_rho_scores.csv", DataFrame)
        theta = CSV.read(prefix * "_theta.csv", DataFrame)
        columns = CSV.read(prefix * "_columns.csv", DataFrame)
        y = CSV.read(prefix * "_ysupport.csv", DataFrame)
        l = Int(y.l[1])
        n = maximum(Int.(station_scores.station_index))

        endpoints = Dict{Int, Set{Int}}()
        for c in eachrow(columns)
            stations = Set{Int}()
            for assignment in filter(x -> !isempty(x), split(String(c.assignments), ';'))
                _, j, k = parse.(Int, split(assignment, ':'))
                push!(stations, j)
                push!(stations, k)
            end
            endpoints[Int(c.column_id)] = stations
        end

        solve_sequences = sort!(unique(Int.(priced.source_solve_sequence)))
        for q in solve_sequences
            opp = [(
                scenario=Int(o.scenario), passenger=Int(o.passenger),
                pickup=Int(o.pickup_index), dropoff=Int(o.dropoff_index),
                rho=Float64(o.rho), travel=Float64(o.direct_travel_time),
            ) for o in eachrow(opportunities_df) if Int(o.solve_sequence) == q]
            routes = [(
                stations=parse_set(r.route, '-'), rc=Float64(r.reported_reduced_cost),
            ) for r in eachrow(priced) if Int(r.source_solve_sequence) == q]
            isempty(routes) && continue
            global_best = minimum(r.rc for r in routes)

            score_rows = filter(r -> Int(r.solve_sequence) == q, eachrow(station_scores))
            marginal = Set(Int(r.station_index) for r in
                sort(score_rows; by=r -> (-Float64(r.positive_rho_sum), Int(r.station_index)))[1:l])

            load = zeros(Float64, n)
            for t in eachrow(theta)
                Int(t.solve_sequence) == q || continue
                value = Float64(t.theta_value)
                value > 1e-7 || continue
                for j in endpoints[Int(t.column_id)]
                    load[j] += value
                end
            end
            theta_ranked = sort!(collect(1:n); by=j -> (-load[j], j))
            theta_subset = Set(theta_ranked[1:l])
            theta_core = Set(theta_ranked[1:min(4, l)])

            methods = [
                ("marginal_rho", marginal),
                ("theta_load", theta_subset),
                ("pair", greedy_pair_subset(opp, n, l, 0.0)),
                ("pair_travel_025", greedy_pair_subset(opp, n, l, 0.25)),
                ("pair_travel_050", greedy_pair_subset(opp, n, l, 0.50)),
                ("pair_travel_100", greedy_pair_subset(opp, n, l, 1.00)),
                ("theta4_pair", greedy_pair_subset(opp, n, l, 0.0; fixed=theta_core)),
                ("theta4_pair_travel_050",
                    greedy_pair_subset(opp, n, l, 0.50; fixed=theta_core)),
            ]
            for (method, selected) in methods
                feasible = [r.rc for r in routes if issubset(r.stations, selected)]
                restricted_best = isempty(feasible) ? missing : minimum(feasible)
                push!(snapshot_rows, (
                    case=case, solve_sequence=q, method=method, l=l,
                    selected_indices=join(sort!(collect(selected)), ";"),
                    pair_coverage_value=coverage_value(opp, selected,
                        method in ("pair_travel_025",) ? 0.25 :
                        method in ("pair_travel_050", "theta4_pair_travel_050") ? 0.50 :
                        method in ("pair_travel_100",) ? 1.00 : 0.0),
                    n_captured_routes=length(feasible),
                    has_improving_route=!isempty(feasible),
                    unrestricted_best_rc=global_best,
                    restricted_best_rc=restricted_best,
                    normalized_rc_regret=ismissing(restricted_best) ? missing :
                        (restricted_best - global_best) / abs(global_best),
                    contains_unrestricted_best=!ismissing(restricted_best) &&
                        abs(restricted_best - global_best) <= 1e-8,
                ))
            end
        end
    end
end

snapshot_df = DataFrame(snapshot_rows)
summary_rows = NamedTuple[]
for method in sort!(unique(snapshot_df.method))
    rows = filter(r -> r.method == method, eachrow(snapshot_df))
    regrets = collect(skipmissing(r.normalized_rc_regret for r in rows))
    push!(summary_rows, (
        method=method, n_snapshots=length(rows),
        improving_route_share=mean(r.has_improving_route for r in rows),
        exact_best_route_share=mean(r.contains_unrestricted_best for r in rows),
        mean_captured_routes=mean(r.n_captured_routes for r in rows),
        median_normalized_rc_regret=isempty(regrets) ? missing : median(regrets),
        mean_normalized_rc_regret=isempty(regrets) ? missing : mean(regrets),
        within_10pct_share=isempty(regrets) ? missing : mean(regrets .<= 0.10),
    ))
end

mkpath(OUTDIR)
CSV.write(joinpath(OUTDIR, "snapshot_results.csv"), snapshot_df)
CSV.write(joinpath(OUTDIR, "method_summary.csv"), DataFrame(summary_rows))
println("wrote $(joinpath(OUTDIR, "snapshot_results.csv"))")
println("wrote $(joinpath(OUTDIR, "method_summary.csv"))")
show(stdout, MIME("text/plain"), DataFrame(summary_rows)); println()
