"""
    scripts/diag_certified_qbar_duality.jl

Tests a specific hypothesis about the invalid-cut bug found in
diag_direct_enumeration_guide_cut_validity.jl: `_certified_qbar` (y_mw_cut.jl) computes
`Q_bar = sum(coverage-constraint duals) + walking_cost_share`, but the LP it certifies
against (`_certified_route_covering_pi`'s re-solved route-covering LP) ALSO has
station-limit (`sum(y)==l`), fixed-open-station, and assignment constraints with nonzero
RHS. By full LP duality, `sum(dual_i * RHS_i)` over ALL constraints (not just coverage)
must equal the LP's primal objective -- if those other constraints' duals are nonzero,
summing ONLY the coverage duals silently omits their contribution, making `Q_bar` wrong
even though the completion LP is later "tight" at Q_bar by construction (self-consistent
but built on a wrong target). This script reproduces the exact LP `_certified_route_covering_pi`
builds for the known-bad y_hat=[11,22,40,106,108] and inspects EVERY constraint's dual,
comparing sum(coverage duals) against the LP's own objective value.
"""

using DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "sample09_mw_vs_direct.jl"))

n_stations = 10
l = L_FOR[n_stations]
max_stops = resolve_max_stops(:ms3, n_stations)
data = load_sample09(n_stations)
model = build_model(l, max_stops, MAX_WALK, CFG)
unit_model = StationSelection._unit_weighted_routing_model(model)

mapping = StationSelection.create_map(model, data)
requests, demand, feasible_pairs = StationSelection._aggregate_od_route_benders_requests(mapping)

y_star_ids = [11, 22, 40, 106, 108]  # phase1's own iteration-1 y_hat (bad cut's origin)
y_hat = zeros(Float64, data.n_stations)
for id in y_star_ids
    y_hat[mapping.station_id_to_array_idx[id]] = 1.0
end
open_stations = StationSelection._open_station_values(y_hat)

assignments, infeasible = StationSelection._fixed_assignments_from_y(
    data, requests, feasible_pairs, y_hat;
    style=model.assignment_policy.feasibility_cut_style,
    max_walking_distance=model.max_walking_distance,
    allow_walk_only=model.allow_walk_only,
    allow_same_station=true,
)
println("infeasible: ", infeasible)

optimizer_env = Gurobi.Env()
config = SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=CFG.mip_gap)
inner_cg = ColumnGenerationSolver(
    config=config, max_iterations=CFG.inner_cg_max_iters, max_columns_per_iteration=20, n_candidates=20,
    pricing_time_limit_sec=CFG.inner_pricing_time, final_ip_time_limit_sec=CFG.inner_ip_time_limit,
)
solver = BendersSolver(
    config=config, decomposition=BendersYZ(), inner_solver=inner_cg,
    max_iterations=CFG.benders_max_iters, reprice_subproblem=true, max_reprice_rounds=CFG.max_reprice_rounds,
    cut_derivation=:zero_completion, lifted_walking_objective=true,
)

route_problem = StationSelection._route_covering_problem_from_assignments(unit_model, assignments, open_stations)
cg_result = StationSelection.run_aggregate_od_route_column_generation(
    route_problem, data;
    optimizer_env=optimizer_env, verbose=false,
    max_cg_iters=inner_cg.max_iterations, max_new_columns=inner_cg.max_columns_per_iteration,
    n_candidates=inner_cg.n_candidates, reduced_cost_tol=inner_cg.reduced_cost_tol,
    pricing_time_limit_sec=inner_cg.pricing_time_limit_sec, ip_time_limit_sec=inner_cg.final_ip_time_limit_sec,
    mip_gap=CFG.mip_gap, silent=true,
)
println("cg_stop_reason=", cg_result.cg_stop_reason, "  lp_bound=", cg_result.lp_bound)

pool = cg_result.generated_columns
lp_problem = StationSelection._copy_with_initial_columns(route_problem, pool; relax_integrality=true)
build = StationSelection.build_model(lp_problem, data; optimizer_env=optimizer_env, relax_integrality=true)
m = build.model
set_silent(m)
set_optimizer_attribute(m, "Method", 1)
set_optimizer_attribute(m, "Presolve", 0)
optimize!(m)
r_value = objective_value(m)
@printf("LP objective (r_value) = %.6f  (should match true Q(y*) = 913.072000)\n", r_value)

# --- Sum coverage-constraint duals only, exactly as _certified_route_covering_pi does ---
coverage = m[:aggregate_od_route_coverage_constraints]
mapping2 = build.mapping
pi_by_request = Dict{NTuple{3, Int}, Float64}()
for (key, con) in coverage
    _j, _k, s, od_idx, _pair_idx = key
    o, d = mapping2.Omega_s[s][od_idx]
    request = (s, o, d)
    request in requests || continue
    pi_by_request[request] = dual(con)
end
coverage_dual_sum = sum(values(pi_by_request); init=0.0)
@printf("sum(coverage-constraint duals only) = %.6f\n", coverage_dual_sum)

# --- Now sum EVERY constraint's dual * RHS, to see what's actually being left out ---
println("\n--- All constraint types and their dual*RHS contribution (only nonzero shown) ---")
total_dual_times_rhs = 0.0
for (F, S) in list_of_constraint_types(m)
    for con in all_constraints(m, F, S)
        local d
        try
            d = dual(con)
        catch
            continue
        end
        abs(d) > 1e-9 || continue
        rhs = try
            normalized_rhs(con)
        catch
            0.0
        end
        global total_dual_times_rhs += d * rhs
        abs(d * rhs) > 1e-6 && println("  ", name(con), "  dual=", d, "  rhs=", rhs, "  dual*rhs=", d * rhs)
    end
end
@printf("sum(dual*rhs) over ALL constraints with nonzero dual = %.6f\n", total_dual_times_rhs)
@printf("difference (coverage-only sum vs LP objective) = %.6f\n", r_value - coverage_dual_sum)
