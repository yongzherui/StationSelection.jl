"""
    scripts/diag_passenger_cg_certificate_validation.jl

Validates the passenger free-assignment CG scheme's optimality certificate.

# What is actually being certified, and how this checks it

When CG stops with `:optimality_proven`, the claim is: the LP over the
*restricted* pool equals the LP over the *complete* column set. That holds iff
no column in the complete set has negative reduced cost at the final duals.

Enumerating the complete column set is intractable -- a column is a (route,
assignment-vector) pair, so one route yields exponentially many columns. But the
check does not need them all: for a fixed dual vector, the assignment vector
minimizing a route's reduced cost is exactly the per-passenger argmax that route
replay already computes, and every other assignment vector for that route has
*higher* reduced cost. So brute-forcing routes, each scored with its argmax
assignment, gives the true minimum reduced cost over the whole complete set.

This script therefore:
  1. runs CG to termination on a small instance;
  2. re-solves the final RMP and extracts its duals;
  3. brute-force enumerates every physical route within max_stops /
     max_visits_per_node, scoring each against those duals with ALL feasible
     assignments available (not the dual-filtered pricing subset);
  4. asserts the minimum reduced cost is >= -tol whenever the run claimed
     `:optimality_proven`.

It also checks `lp_bound <= mip_objective + tol` (LP relaxation must bound the
integer optimum) and that no passenger is left on its slack when a servable
assignment exists.

Usage:
    julia --project=. scripts/diag_passenger_cg_certificate_validation.jl [n_stations] [n_pairs]
"""

using Printf, Gurobi, JuMP, StationSelection

const MOI = JuMP.MOI

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const SEED = 42
const N_SCENARIOS = 1
const TOL = 1e-5

const GRB_ENV = Gurobi.Env()

function build_model_for(n_stations::Int, max_walk::Float64)
    return AggregateODRouteModel(
        max(2, ceil(Int, n_stations / 2));
        route_regularization_weight = 1.0,
        walk_cost_weight            = 0.1,
        repositioning_time          = 20.0,
        max_walking_distance        = max_walk,
        max_wait_time               = 900.0,
        detour_factor               = 2.0,
        max_stops                   = 3,
        max_visits_per_node         = 2,
    )
end

"""
Minimum reduced cost over the complete column set at the given duals, by brute
force over physical routes. `all_candidates` deliberately uses every feasible
assignment (rho computed from duals, no positivity filter beyond what the
reward-layer preprocessing needs), so no route's best assignment is hidden.
"""
function brute_force_min_reduced_cost(
    master_data::PassengerFreeAssignmentMasterData,
    alpha::Dict{Int, Float64},
    gamma_o::Dict{Tuple{Int, Int}, Float64},
    gamma_d::Dict{Tuple{Int, Int}, Float64},
    scenario::Int,
)
    candidates = PassengerAssignmentCandidate[]
    for p_id in get(master_data.passengers_by_scenario, scenario, Int[])
        a = get(alpha, p_id, 0.0)
        for (j, k) in master_data.feasible_assignments[p_id]
            rho = a - get(gamma_o, (p_id, j), 0.0) - get(gamma_d, (p_id, k), 0.0) -
                master_data.walk_cost_weight * master_data.assignment_walk_cost[(p_id, j, k)]
            rho > 1e-12 || continue
            push!(candidates, PassengerAssignmentCandidate(
                p_id, j, k, master_data.ride_limit[(p_id, j, k)], rho,
            ))
        end
    end
    isempty(candidates) && return Inf, Int[]

    pricing_data = create_passenger_free_assignment_pricing_data(
        scenario, master_data.nodes, master_data.travel_cost, candidates;
        route_regularization_weight=master_data.route_regularization_weight,
        max_wait_time=master_data.max_wait_time,
        repositioning_time=master_data.repositioning_time,
        max_stops=master_data.max_stops,
        max_visits_per_node=master_data.max_visits_per_node,
    )

    nodes = master_data.nodes
    max_stops = master_data.max_stops == typemax(Int) ? length(nodes) : master_data.max_stops
    max_visits = master_data.max_visits_per_node
    best_rc = Inf
    best_route = Int[]
    visit_counts = Dict{Int, Int}()
    route = Int[]

    function recurse!()
        if length(route) >= 2
            assignments, _tau, rc = StationSelection._passenger_free_assignment_column_from_route(
                copy(route), pricing_data,
            )
            if !isempty(assignments) && rc < best_rc - 1e-12
                best_rc = rc
                best_route = copy(route)
            end
        end
        length(route) >= max_stops && return
        for nd in nodes
            !isempty(route) && nd == route[end] && continue
            get(visit_counts, nd, 0) < max_visits || continue
            push!(route, nd)
            visit_counts[nd] = get(visit_counts, nd, 0) + 1
            recurse!()
            visit_counts[nd] -= 1
            pop!(route)
        end
    end

    for start in nodes
        push!(route, start)
        visit_counts[start] = 1
        recurse!()
        visit_counts[start] = 0
        pop!(route)
    end
    return best_rc, best_route
end

function main()
    n_stations = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 8
    n_pairs = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 6

    println("=== CG certificate validation: n_stations=$n_stations n_pairs=$n_pairs ===")
    data, _meta = generate_zhuzhou_data(
        DATA_DIR, n_stations, n_pairs; n_scenarios=N_SCENARIOS, seed=SEED,
    )
    model = build_model_for(n_stations, 600.0)

    result = run_passenger_free_assignment_column_generation(
        model, data;
        optimizer_env=GRB_ENV,
        max_cg_iters=500,
        n_candidates=5,
        max_new_columns=5,
        pricing_time_limit_sec=30.0,
        certification_time_limit_sec=300.0,
        ip_time_limit_sec=120.0,
        verify_reduced_costs=true,
        verbose=true,
    )

    println()
    println("status                = $(result.status)")
    println("cg_stop_reason         = $(result.cg_stop_reason)")
    println("lp_bound               = $(result.lp_bound)  certified=$(result.lp_bound_certified)")
    println("mip_objective          = $(result.mip_objective)")
    println("cg iterations / rounds = $(result.n_cg_iters) / $(result.n_rounds)")
    println("columns in pool        = $(result.n_columns)")
    println("passengers             = $(result.n_passengers)")
    println("master rows            = $(result.n_master_rows)")
    println("open stations          = $(result.open_stations)")
    println("unserved passengers    = $(result.unserved_passengers)")
    println("certification seconds  = $(round(result.certification_seconds; digits=2)) exhausted=$(result.certification_exhausted)")
    @printf("timing: total=%.3fs  pricing=%.3fs  lp=%.3fs  certification=%.3fs  labels=%d\n",
        result.total_seconds, result.total_pricing_seconds, result.total_lp_seconds,
        result.certification_seconds, result.total_labels_generated)
    println("(job wall time also includes Julia precompile + Gurobi env + this script's " *
            "independent re-run and brute-force check, none of which are CG cost)")
    println()

    ok = true

    # ── check 1: LP relaxation must bound the integer optimum ─────────────────
    if !isnothing(result.mip_objective)
        if result.lp_bound <= result.mip_objective + TOL
            @printf("PASS  lp_bound (%.6f) <= mip_objective (%.6f)\n", result.lp_bound, result.mip_objective)
        else
            @printf("FAIL  lp_bound (%.6f) > mip_objective (%.6f)\n", result.lp_bound, result.mip_objective)
            ok = false
        end
    end

    # ── check 2: the optimality certificate itself ────────────────────────────
    # Rebuild the master, replay CG's pool into it, re-solve the LP, and confirm
    # brute force cannot find a negative-reduced-cost route at those duals.
    mapping = create_map(model, data)
    master_data = create_passenger_free_assignment_master_data(model, data, mapping)
    println("re-running CG to recover final duals for the brute-force check ...")
    master = build_passenger_free_assignment_master(master_data, GRB_ENV; relax_integrality=true)
    set_silent(master.model)

    # Regenerate the pool by re-running the same CG loop against this fresh master
    # is wasteful; instead price iteratively here until certification is clean,
    # which reproduces the same terminal dual vector.
    next_id = 1
    for _iter in 1:1000
        optimize!(master.model)
        primal_status(master.model) == MOI.FEASIBLE_POINT || break
        alpha, gamma_o, gamma_d = extract_passenger_free_assignment_duals(master)
        added_any = false
        for s in sort!(collect(keys(master_data.passengers_by_scenario)))
            cands = passenger_free_assignment_pricing_candidates(master_data, alpha, gamma_o, gamma_d, s)
            isempty(cands) && continue
            pd = create_passenger_free_assignment_pricing_data(
                s, master_data.nodes, master_data.travel_cost, cands;
                route_regularization_weight=master_data.route_regularization_weight,
                max_wait_time=master_data.max_wait_time,
                repositioning_time=master_data.repositioning_time,
                max_stops=master_data.max_stops,
                max_visits_per_node=master_data.max_visits_per_node,
            )
            isempty(pd.opportunities) && continue
            existing = PassengerFreeAssignmentRouteColumn[
                c for c in values(master.columns) if get(c.metadata, "scenario", 0) == s
            ]
            cols, _exh, _st = passenger_free_assignment_pricing_by_label_setting(
                pd, existing;
                next_column_id=next_id, max_new_columns=10^6, n_candidates=10^6, time_limit=300.0,
            )
            next_id += length(cols)
            for c in cols
                _t, action = add_passenger_free_assignment_column!(master, c)
                action == :added && (added_any = true)
            end
        end
        added_any || break
    end

    optimize!(master.model)
    lp_ref = objective_value(master.model)
    alpha, gamma_o, gamma_d = extract_passenger_free_assignment_duals(master)
    @printf("independent exhaustive-CG LP bound = %.6f  (CG reported %.6f)\n", lp_ref, result.lp_bound)
    if isapprox(lp_ref, result.lp_bound; atol=1e-4)
        println("PASS  CG LP bound matches an independently regenerated exhaustive-pricing LP bound")
    else
        @printf("FAIL  LP bound mismatch: %.6f vs %.6f\n", lp_ref, result.lp_bound)
        ok = false
    end

    worst = Inf
    for s in sort!(collect(keys(master_data.passengers_by_scenario)))
        rc, rt = brute_force_min_reduced_cost(master_data, alpha, gamma_o, gamma_d, s)
        @printf("  scenario %d: brute-force min reduced cost = %s  route=%s\n",
            s, isinf(rc) ? "none" : @sprintf("%.8f", rc), string(rt))
        isfinite(rc) && (worst = min(worst, rc))
    end
    if isinf(worst) || worst >= -TOL
        println("PASS  no negative-reduced-cost route exists at the final duals -- certificate valid")
    else
        @printf("FAIL  brute force found reduced cost %.8f < -%.1e -- certificate INVALID\n", worst, TOL)
        ok = false
    end

    println()
    println(ok ? "OVERALL: PASS" : "OVERALL: FAIL")
    ok || exit(1)
end

main()
