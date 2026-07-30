"""
    scripts/diag_lifted_routing_lb_bound_warning.jl

Diagnostic: re-runs the same 8 (cut_derivation, lifted_walking_objective, lifted_routing_lower_bound)
combinations from test_aggregate_od_route_lifted_routing_lower_bound.jl's "Test B" on the same
5-station fixture, but labels each run explicitly and reports its final lower_bound/incumbent/gap
metadata directly (rather than relying on @warn log scraping), to determine whether the
"Benders master lower bound exceeds the feasible incumbent" warning correlates with
lifted_routing_lower_bound=true specifically or is pre-existing on this fixture.

Usage:
    julia --project=. scripts/diag_lifted_routing_lb_bound_warning.jl
"""

using StationSelection, DataFrames, Dates, JuMP, Gurobi, Printf
const MOI = JuMP.MOI

function fixture()
    stations = DataFrame(id=collect(1:5), lon=Float64.(1:5), lat=zeros(5))
    requests = DataFrame(
        id=[1, 2],
        start_station_id=[1, 2],
        end_station_id=[5, 4],
        request_time=[DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 8, 1)],
    )
    walking_costs = Dict{Tuple{Int, Int}, Float64}()
    for i in 1:5, j in 1:5
        walking_costs[(i, j)] = 100.0
    end
    walking_costs[(1, 1)] = 0.0
    walking_costs[(1, 2)] = 3.0
    walking_costs[(4, 5)] = 3.0
    walking_costs[(5, 5)] = 0.0
    walking_costs[(2, 2)] = 0.0
    walking_costs[(4, 4)] = 0.0
    routing_costs = Dict{Tuple{Int, Int}, Float64}()
    for i in 1:5, j in 1:5
        routing_costs[(i, j)] = abs(i - j) + 1.0
    end
    return create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
end

function model()
    return AggregateODRouteModel(
        4;
        assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
        max_walking_distance=5.0,
        route_regularization_weight=3.0,
        walk_cost_weight=0.37,
        repositioning_time=1.5,
        max_stops=3,
        max_wait_time=1000.0,
        detour_factor=2.0,
    )
end

data = fixture()
m = model()

for cut_derivation in (:standard, :zero_completion)
    for lifted_walking in (false, true)
        for lifted_lb in (false, true)
            env = Gurobi.Env()
            result = run_opt(
                data, m,
                BendersSolver(
                    config=SolverConfig(optimizer_env=env, silent=true, mip_gap=0.0),
                    decomposition=BendersYZ(),
                    inner_solver=ColumnGenerationSolver(
                        config=SolverConfig(optimizer_env=env, silent=true, mip_gap=0.0),
                        max_iterations=200, max_columns_per_iteration=20, n_candidates=20,
                        final_ip_time_limit_sec=30.0,
                    ),
                    max_iterations=50,
                    reprice_subproblem=true,
                    cut_derivation=cut_derivation,
                    lifted_walking_objective=lifted_walking,
                    lifted_routing_lower_bound=lifted_lb,
                ),
            )
            lb = get(result.metadata, "benders_lower_bound", NaN)
            incumbent = get(result.metadata, "benders_incumbent_objective", NaN)
            gap = get(result.metadata, "benders_outer_gap_relative", NaN)
            @printf(
                "cut_derivation=%-16s lifted_walking=%-5s lifted_lb=%-5s status=%-8s obj=%-8.4f lower_bound=%-8s incumbent=%-8s gap=%s\n",
                cut_derivation, lifted_walking, lifted_lb, result.termination_status,
                result.objective_value, string(lb), string(incumbent), string(gap),
            )
        end
    end
end
