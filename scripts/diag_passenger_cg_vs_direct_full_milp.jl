"""
    scripts/diag_passenger_cg_vs_direct_full_milp.jl

Verify the passenger free-assignment CG scheme against a DIRECT enumerate-then-MIP
solve of the **full problem** (not just the LP relaxation).

# What was already verified elsewhere, and what this adds

Prior checks all concerned the LP:
  * `diag_passenger_free_assignment_vs_direct.jl` -- the pricing oracle finds the
    true minimum-reduced-cost route (vs brute force over routes);
  * `diag_passenger_cg_certificate_validation.jl` -- at the certified duals no
    route has negative reduced cost;
  * `test_passenger_dual_selection.jl` -- LP over the fully enumerated column set
    equals ordinary CG's and the selector's LP bound.

None of those tests the **integer** optimum. CG certifies the LP bound; the final
MIP is solved over the *pool CG happened to generate*, which can in principle be
strictly worse than the MIP over the complete column set. This script closes that
gap by building the complete column set and solving the full MILP directly.

# Why the "complete column set" is finite and small enough to enumerate

A column is a (route, assignment-vector) pair, so naively the set is exponential:
for each route, each passenger may take any of its certified `(j,k)` or none, and
sub-assignment columns are genuinely useful in the MILP (fewer assignments means
lower walking cost AND fewer stations forced open).

But the master only ever sees a column through its **assignment set** and its
`tau`: the objective is
`beta*(tau + repo) + walk_weight * sum_assignments w`, and the coverage/linking
rows depend only on which `(p,j,k)` it carries. So for a given assignment set, the
only route worth keeping is the cheapest one achieving it -- exactly the dedup rule
`_passenger_free_assignment_column_signature` already implements. The complete
column set, up to what the master can distinguish, is therefore

    { (assignment set A, min tau over routes certifying all of A) }

which is bounded by `prod_p (#certified options for p + 1)` per route rather than
anything exponential in the route count. That is enumerable for small instances.

# Checks performed

  1. `z_direct_full <= z_cg_mip + tol`   -- direct enumeration can only match or beat
     the CG pool's MIP. A violation would mean the direct model is wrong.
  2. `z_direct_full == z_cg_mip`          -- the CG pool contained an integer optimum.
     A strict gap here is the real failure mode this script exists to detect.
  3. `lp_bound <= z_direct_full + tol` when certified -- the certified LP bound must
     bound the true integer optimum.
  4. `lp_bound == z_cg_mip  ==>  z_cg_mip == z_direct_full` -- the proof sketch above,
     checked empirically.

Usage:
    julia --project=. scripts/diag_passenger_cg_vs_direct_full_milp.jl [n_stations] [n_pairs] [max_stops]
"""

using Printf, Gurobi, JuMP, DataFrames, Dates, StationSelection

const MOI = JuMP.MOI

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const SEED = 42
const TOL = 1e-5
const MAX_ASSIGNMENT_SETS_PER_ROUTE = 20_000
const MAX_TOTAL_COLUMNS = 400_000

const GRB_ENV = Gurobi.Env()

_l_for(n::Int) = max(2, ceil(Int, n / 2))

function build_model_for(n_stations::Int, max_stops::Int)
    return AggregateODRouteModel(
        _l_for(n_stations);
        route_regularization_weight = 1.0,
        walk_cost_weight            = 0.1,
        repositioning_time          = 20.0,
        max_walking_distance        = 600.0,
        max_wait_time               = 900.0,
        detour_factor               = 2.0,
        max_stops                   = max_stops,
        max_visits_per_node         = 2,
    )
end

"""
All `(p, j, k)` triples a finished physical route certifies -- every one, not just
each passenger's argmax (which is what `_replay_passenger_free_assignment_route`
returns). Mirrors the pricer's own clock rules: age resets only within
`max_wait_time`, and `(p,j,k)` fires when `age(j) <= R_pjk` on arrival at `k`.
"""
function certified_triples(route::Vector{Int}, pd::PassengerFreeAssignmentPricingData)
    out = Dict{Int, Vector{Tuple{Int, Int}}}()
    isempty(route) && return out
    station_age = Dict{Int, Float64}(route[1] => 0.0)
    elapsed = 0.0
    current = route[1]
    for idx in 2:length(route)
        nxt = route[idx]
        dt = StationSelection._passenger_free_assignment_travel(pd, current, nxt)
        elapsed += dt
        station_age = Dict(st => age + dt for (st, age) in station_age)
        for opp in get(pd.assignments_by_destination, nxt, PassengerAssignmentOpportunity[])
            age = get(station_age, opp.origin, Inf)
            age <= opp.ride_limit + 1e-9 || continue
            lst = get!(() -> Tuple{Int, Int}[], out, opp.passenger)
            (opp.origin, opp.destination) in lst || push!(lst, (opp.origin, opp.destination))
        end
        elapsed <= pd.max_wait_time + 1e-9 && (station_age[nxt] = 0.0)
        current = nxt
    end
    return out
end

route_tau(route, pd) = length(route) < 2 ? 0.0 :
    sum(StationSelection._passenger_free_assignment_travel(pd, route[i], route[i + 1])
        for i in 1:(length(route) - 1))

"""
Complete column set as `signature => (assignments, min tau, scenario)`.
Returns `(columns, capped)`; `capped == true` means an enumeration limit was hit
and the "complete" set is NOT complete, so downstream conclusions are void.
"""
function enumerate_complete_column_set(md::PassengerFreeAssignmentMasterData)
    best = Dict{Any, Tuple{Vector{Tuple{Int, Int, Int}}, Float64, Int}}()
    capped = false

    for s in sort!(collect(keys(md.passengers_by_scenario)))
        cands = PassengerAssignmentCandidate[]
        for p_id in md.passengers_by_scenario[s]
            for (j, k) in md.feasible_assignments[p_id]
                push!(cands, PassengerAssignmentCandidate(
                    p_id, j, k, md.ride_limit[(p_id, j, k)], 1.0,
                ))
            end
        end
        isempty(cands) && continue
        pd = create_passenger_free_assignment_pricing_data(
            s, md.nodes, md.travel_cost, cands;
            route_regularization_weight=md.route_regularization_weight,
            max_wait_time=md.max_wait_time,
            repositioning_time=md.repositioning_time,
            max_stops=md.max_stops,
            max_visits_per_node=md.max_visits_per_node,
        )

        nodes = md.nodes
        max_stops = md.max_stops == typemax(Int) ? length(nodes) : md.max_stops
        max_visits = md.max_visits_per_node == typemax(Int) ? max_stops : md.max_visits_per_node
        route = Int[]
        counts = Dict{Int, Int}()

        function record_route!()
            length(route) >= 2 || return
            cert = certified_triples(copy(route), pd)
            isempty(cert) && return
            tau = route_tau(route, pd)
            ps = sort!(collect(keys(cert)))
            # options per passenger: each certified (j,k), or "not assigned"
            n_sets = 1
            for p in ps
                n_sets *= (length(cert[p]) + 1)
                if n_sets > MAX_ASSIGNMENT_SETS_PER_ROUTE
                    capped = true
                    return
                end
            end
            # iterate the cartesian product by mixed-radix counter
            radix = [length(cert[p]) + 1 for p in ps]
            for code in 0:(n_sets - 1)
                rem = code
                assigns = Tuple{Int, Int, Int}[]
                for (i, p) in enumerate(ps)
                    choice = rem % radix[i]
                    rem ÷= radix[i]
                    choice == 0 && continue        # passenger not served by this column
                    (j, k) = cert[p][choice]
                    push!(assigns, (p, j, k))
                end
                isempty(assigns) && continue
                sig = Tuple(sort(assigns))
                cur = get(best, sig, nothing)
                if isnothing(cur) || tau < cur[2] - 1e-9
                    best[sig] = (assigns, tau, s)
                end
                length(best) > MAX_TOTAL_COLUMNS && (capped = true; return)
            end
        end

        function dfs!()
            record_route!()
            capped && return
            length(route) >= max_stops && return
            for nd in nodes
                !isempty(route) && nd == route[end] && continue
                get(counts, nd, 0) < max_visits || continue
                push!(route, nd); counts[nd] = get(counts, nd, 0) + 1
                dfs!()
                counts[nd] -= 1; pop!(route)
                capped && return
            end
        end
        for st in nodes
            push!(route, st); counts[st] = 1
            dfs!()
            counts[st] = 0; pop!(route)
            capped && break
        end
    end
    return best, capped
end

function main()
    n_stations = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 6
    n_pairs = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 4
    max_stops = length(ARGS) >= 3 ? parse(Int, ARGS[3]) : 3

    println("=== CG vs DIRECT full-MILP: n=$n_stations p=$n_pairs max_stops=$max_stops ===")
    data, _meta = generate_zhuzhou_data(DATA_DIR, n_stations, n_pairs; n_scenarios=1, seed=SEED)
    model = build_model_for(n_stations, max_stops)
    mapping = create_map(model, data)
    md = create_passenger_free_assignment_master_data(model, data, mapping)
    @printf("passengers=%d  l=%d\n", length(md.passengers), md.l)

    # ── 1. CG ─────────────────────────────────────────────────────────────────
    cg = run_passenger_free_assignment_column_generation(
        model, data; optimizer_env=GRB_ENV,
        max_cg_iters=2000, n_candidates=10, max_new_columns=10,
        pricing_time_limit_sec=60.0, certification_time_limit_sec=300.0,
        ip_time_limit_sec=300.0, total_time_limit_sec=900.0, verbose=false,
    )
    @printf("CG : stop=%s certified=%s lp=%.6f mip=%s pool=%d\n",
        cg.cg_stop_reason, cg.lp_bound_certified, cg.lp_bound,
        isnothing(cg.mip_objective) ? "-" : @sprintf("%.6f", cg.mip_objective), cg.n_columns)

    # ── 2. complete column set + direct full MILP ─────────────────────────────
    t0 = time()
    complete, capped = enumerate_complete_column_set(md)
    @printf("enumerated %d distinct assignment-set columns in %.1fs (capped=%s)\n",
        length(complete), time() - t0, capped)
    if capped
        println("!! enumeration hit a cap -- the column set is NOT complete; " *
                "conclusions below are void. Rerun with a smaller instance.")
        exit(2)
    end

    direct = build_passenger_free_assignment_master(md, GRB_ENV; relax_integrality=false)
    set_silent(direct.model)
    next_id = 1
    for (_sig, (assigns, tau, s)) in complete
        col = PassengerFreeAssignmentRouteColumn(
            next_id, Int[], assigns, tau;
            metadata=Dict{String, Any}("scenario" => s),
        )
        add_passenger_free_assignment_column!(direct, col)
        next_id += 1
    end
    for th in values(direct.theta); set_binary(th); end
    for xv in values(direct.x_same); set_binary(xv); end
    set_optimizer_attribute(direct.model, "TimeLimit", 900.0)
    optimize!(direct.model)
    d_term = termination_status(direct.model)
    z_direct = d_term == MOI.OPTIMAL ? objective_value(direct.model) : nothing
    @printf("DIRECT full MILP: term=%s obj=%s columns=%d\n",
        d_term, isnothing(z_direct) ? "-" : @sprintf("%.6f", z_direct), length(direct.theta))

    # also the LP over the complete set, for reference
    direct_lp = build_passenger_free_assignment_master(md, GRB_ENV; relax_integrality=true)
    set_silent(direct_lp.model)
    nid = 1
    for (_sig, (assigns, tau, s)) in complete
        add_passenger_free_assignment_column!(direct_lp, PassengerFreeAssignmentRouteColumn(
            nid, Int[], assigns, tau; metadata=Dict{String, Any}("scenario" => s)))
        nid += 1
    end
    optimize!(direct_lp.model)
    z_direct_lp = termination_status(direct_lp.model) == MOI.OPTIMAL ?
        objective_value(direct_lp.model) : NaN
    @printf("DIRECT full LP  : obj=%.6f\n", z_direct_lp)
    println()

    # ── 3. checks ─────────────────────────────────────────────────────────────
    ok = true
    if isnothing(z_direct) || isnothing(cg.mip_objective)
        println("SKIP comparisons (a MILP did not solve to optimality)")
        ok = false
    else
        if z_direct <= cg.mip_objective + TOL
            @printf("PASS  z_direct (%.6f) <= z_cg_mip (%.6f)\n", z_direct, cg.mip_objective)
        else
            @printf("FAIL  z_direct (%.6f) > z_cg_mip (%.6f) -- direct model is WRONG\n",
                z_direct, cg.mip_objective); ok = false
        end
        if isapprox(z_direct, cg.mip_objective; atol=TOL)
            println("PASS  CG pool contained an integer optimum (no pool-completeness gap)")
        else
            @printf("FAIL  pool-completeness gap: CG pool MIP %.6f vs complete-set MIP %.6f (%.4f%%)\n",
                cg.mip_objective, z_direct,
                100 * (cg.mip_objective - z_direct) / abs(z_direct)); ok = false
        end
        if cg.lp_bound_certified
            if cg.lp_bound <= z_direct + TOL
                @printf("PASS  certified lp_bound (%.6f) <= z_direct (%.6f)\n", cg.lp_bound, z_direct)
            else
                @printf("FAIL  certified lp_bound (%.6f) > z_direct (%.6f) -- INVALID BOUND\n",
                    cg.lp_bound, z_direct); ok = false
            end
            if isapprox(cg.lp_bound, z_direct_lp; atol=1e-4)
                println("PASS  certified lp_bound equals the LP over the complete column set")
            else
                @printf("FAIL  certified lp_bound %.6f != complete-set LP %.6f\n",
                    cg.lp_bound, z_direct_lp); ok = false
            end
            if isapprox(cg.lp_bound, cg.mip_objective; atol=TOL)
                println("NOTE  lp_bound == cg_mip, so full-problem optimality was already " *
                        "provable without enumeration; direct solve confirms it.")
            end
        else
            println("NOTE  CG did not certify, so no bound claim is checked.")
        end
    end

    println()
    println(ok ? "OVERALL: PASS" : "OVERALL: FAIL")
    ok || exit(1)
end

main()
