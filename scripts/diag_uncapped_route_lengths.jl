"""
    scripts/diag_uncapped_route_lengths.jl <n_stations>

Re-solve one PFA case (uncapped or capped max_stops) and report the LENGTH
distribution of the routes actually SELECTED in the final integer solution
(theta > 0.5). Answers "does the uncapped optimum really use long routes, or are
the selected routes short even when the search was allowed to go long?"

Env (mirrors passenger_free_assignment_cg_scaling.jl):
    PFA_N_PAIRS (16), PFA_N_SCENARIOS (1), PFA_SEEDS (single, 42),
    PFA_MAX_STOPS (0 => uncapped typemax), PFA_MAX_VISITS (3),
    PFA_CASE_TIME, PFA_CERT_TIME, PFA_PRICING_TIME, PFA_IP_TIME
"""

using Gurobi, JuMP, Printf, StationSelection
include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = parse(Int, get(ENV, "PFA_N_PAIRS", "16"))
const N_SCEN = parse(Int, get(ENV, "PFA_N_SCENARIOS", "1"))
const SEED = parse(Int, split(get(ENV, "PFA_SEEDS", "42"), ',')[1])
const _RMS = parse(Int, get(ENV, "PFA_MAX_STOPS", "0"))
const MAX_STOPS = _RMS <= 0 ? typemax(Int) : _RMS
const _RMV = parse(Int, get(ENV, "PFA_MAX_VISITS", "3"))
const MAX_VISITS = _RMV <= 0 ? typemax(Int) : _RMV
const CASE_TIME = parse(Float64, get(ENV, "PFA_CASE_TIME", "2400"))
const CERT_TIME = parse(Float64, get(ENV, "PFA_CERT_TIME", "1800"))
const PRICING_TIME = parse(Float64, get(ENV, "PFA_PRICING_TIME", "120"))
const IP_TIME = parse(Float64, get(ENV, "PFA_IP_TIME", "300"))
const GRB_ENV = Gurobi.Env()
_l_for(n) = max(2, ceil(Int, n / 2))

function main()
    n = parse(Int, ARGS[1])
    data, _ = generate_zhuzhou_data(DATA_DIR, n, N_PAIRS; n_scenarios=N_SCEN, seed=SEED)
    model = AggregateODRouteModel(
        _l_for(n);
        route_regularization_weight=10.0, walk_cost_weight=0.1, repositioning_time=20.0,
        max_walking_distance=600.0, max_wait_time=900.0, detour_factor=2.0,
        max_stops=MAX_STOPS, max_visits_per_node=MAX_VISITS,
    )
    tag = MAX_STOPS == typemax(Int) ? "uncapped" : "ms$(MAX_STOPS)"
    @printf("=== n=%d scen=%d seed=%d l=%d %s ===\n", n, N_SCEN, SEED, _l_for(n), tag)

    r = run_passenger_free_assignment_column_generation(
        model, data; optimizer_env=GRB_ENV,
        max_cg_iters=2000, n_candidates=20, max_new_columns=20,
        pricing_time_limit_sec=PRICING_TIME, certification_time_limit_sec=CERT_TIME,
        ip_time_limit_sec=IP_TIME, total_time_limit_sec=CASE_TIME,
        verbose=false,
    )

    @printf("stop=%s certified=%s mip=%s\n",
        r.cg_stop_reason, r.lp_bound_certified,
        isnothing(r.mip_objective) ? "n/a" : @sprintf("%.2f", r.mip_objective))

    sol = r.final_result.solution
    if isnothing(sol)
        println("no integer solution to inspect"); return
    end
    theta = sol[1].route_columns                       # Dict(id => value)
    rows = r.final_result.metadata["column_rows"]      # NamedTuples w/ .column_id, .route, .tau
    route_by_id = Dict(row.column_id => row.route for row in rows)
    tau_by_id = Dict(row.column_id => row.tau for row in rows)
    scen_by_id = Dict(row.column_id => row.scenario for row in rows)

    selected = sort([id for (id, v) in theta if v > 0.5])
    lengths = Int[]
    println("\nselected routes (theta>0.5):")
    println("  id      scen  stops  tau      route")
    for id in selected
        route = route_by_id[id]
        nstops = length(split(route, "-"))
        push!(lengths, nstops)
        @printf("  %-7d %-4d  %-5d  %-7.1f  %s\n", id, scen_by_id[id], nstops, tau_by_id[id], route)
    end

    isempty(lengths) && (println("\n(no vehicle routes selected -- all walk-only)"); return)
    maxlen = maximum(lengths)
    hist = Dict{Int, Int}()
    for L in lengths; hist[L] = get(hist, L, 0) + 1; end
    println("\nroute-length histogram (stops => count):")
    for L in sort(collect(keys(hist)))
        @printf("  %2d stops: %s (%d)\n", L, "#"^hist[L], hist[L])
    end
    @printf("\nselected: %d routes, max %d stops, %d routes with >4 stops (%.0f%%)\n",
        length(lengths), maxlen, count(>(4), lengths),
        100 * count(>(4), lengths) / length(lengths))
end

main()
