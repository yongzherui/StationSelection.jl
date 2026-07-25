# n=15 method-compare sweep: exact jobs/methods hitting the LP/IP outer-gap bug, plus a recipe for observing it directly in `RouteCoveringProblem`

*2026-07-24*

## Relationship to prior work

This is **Bug 3** from [[2026-07-23_benders_reports_optimal_with_unclosed_outer_gap]] /
[[2026-07-24_benders_optimal_with_unclosed_gap_grid_reproducer]] (cross-referenced in memory as
[[project_benders_false_optimal_lp_ip_gap]]) surfacing again in the live n=15
`aggregate_od_route_method_compare` sweep, now that the unrelated tie-break-cost crash
(`assert_endpoint_chain_near_binary` firing on genuinely fractional `z`, fixed this session by
resizing `tie_break_scale` in `_big_m_tie_break_costs`, `src/opt/constraints/aggregate_od_route.jl`)
no longer masks it by crashing first. This note adds nothing to the root-cause analysis in those two
notes -- it exists purely to (a) pin down the *exact* jobs/methods in this sweep currently exhibiting
it, so they're easy to re-check once bug 3 itself is fixed, and (b) give a copy-pasteable recipe for
pulling the LP-vs-IP gap directly out of `RouteCoveringProblem` for any of them, without having to
re-derive the internal call sequence from scratch each time.

## Exact failing cases (n=15 sweep, as of this session)

All four below are cases where `analyze_method_compare.jl`'s "provably exact" tier (Direct +
repriced Benders, which must always agree) disagrees. `jobs.txt` here is
`experiments/aggregate_od_route_method_compare/jobs.txt`.

| instance | max_stops_mode | method | jobs.txt row | objective | final_lower_bound | final_outer_gap | n_iterations | selected_stations |
|---|---|---|---:|---:|---:|---:|---:|---|
| grid_n15_p8_s42 | ms4 | `direct_ms4` (truth) | 451 | **491.7** | -- | -- | -- | `[2,4,5,6,7,8,10,11]` |
| grid_n15_p8_s42 | ms4 | `bendersYZ_std_reprice_ms4` | 464 | 501.3 | 368.867 | 26.42% | 259 | `[2,4,5,6,7,8,11,14]` |
| grid_n15_p8_s42 | ms4 | `bendersYZH_std_reprice_ms4` | 472 | 501.3 | 368.867 | 26.42% | 188 | `[1,5,7,8,9,10,11,12]` |
| zhuzhou_n15_p32_s123 | ms4 | `direct_ms4` (truth) | 851 | **93954.14** | -- | -- | -- | `[11,22,92,100,117,133,158,202]` |
| zhuzhou_n15_p32_s123 | ms4 | `bendersYZ_std_reprice_ms4` | 864 | 94142.78 | 90867.59 | 3.48% | 300 | `[11,22,92,100,117,133,158,202]` (**same set as Direct**) |
| zhuzhou_n15_p32_s123 | ms4 | `bendersYZH_std_reprice_ms4` | 872 | 94142.78 | 90867.59 | 3.48% | 270 | `[11,22,92,100,117,133,158,202]` (**same set as Direct**) |
| zhuzhou_n15_p32_s42 | ms4 | `direct_ms4` (truth) | 826 | **101883.09** | -- | -- | -- | `[11,92,100,117,133,138,158,202]` |
| zhuzhou_n15_p32_s42 | ms4 | `bendersYZ_std_reprice_ms4` | 839 | 102299.35 | 97695.17 | 4.50% | 351 | `[11,22,92,100,133,138,158,202]` (differs: 22 vs 117) |
| zhuzhou_n15_p32_s42 | ms4 | `bendersYZH_std_reprice_ms4` | 847 | 102299.35 | 97695.17 | 4.50% | 307 | `[11,22,92,100,133,138,158,202]` (differs: 22 vs 117) |
| zhuzhou_n15_p32_s999 | ms4 | `direct_ms4` (truth) | 876 | **91549.28** | -- | -- | -- | `[11,40,54,100,117,133,158,202]` |
| zhuzhou_n15_p32_s999 | ms4 | `bendersYZ_std_reprice_ms4` | 889 | 91749.28 | 90826.31 | 1.01% | 362 | `[11,40,54,100,117,133,158,202]` (**same set as Direct**) |
| zhuzhou_n15_p32_s999 | ms4 | `bendersYZH_std_reprice_ms4` | 897 | 91749.28 | 90826.31 | 1.01% | 319 | `[11,40,54,100,117,133,158,202]` (**same set as Direct**) |

All grid `_std_reprice_ms4` runs at `n_stations=15` in this sweep so far show the bug on `p8_s42`
only (`p8_s123`, `p8_s999`, `p16_*`, most of `p32_*` converge cleanly); on the zhuzhou family it's
consistently every `p32` seed at `ms4`, never `uncapped` (uncapped isn't in this table because no
uncapped/`p32` pair has reached the "provably exact" tier yet in this sweep to compare against).

## A sharper new data point: `zhuzhou_n15_p32_s123` and `zhuzhou_n15_p32_s999` select the *identical* station set as Direct, yet still report a worse objective

This is stronger than the `grid_n10_p16_s123` reproducer in
[[2026-07-24_benders_optimal_with_unclosed_gap_grid_reproducer]], where the terminating `y_hat` was
a *different* station set from the incumbent. Here, for two of the four cases,
**`selected_stations` is byte-for-byte identical to Direct's true optimum** -- `[11,22,92,100,117,133,158,202]`
and `[11,40,54,100,117,133,158,202]` respectively -- and yet Benders' reported objective is still
~189-200 units *higher* than Direct's for the exact same open-station set. Since the route-covering
cost for a *fixed* `y` is a well-defined optimization problem independent of which decomposition
found `y`, this means **Benders' own final re-solve of the route-covering subproblem for its
incumbent `y` is leaving cost on the table** -- a real LP/IP-adjacent gap, but this time located in
route *generation* (the inner CG's pricing pool), not (or not only) in the outer stopping rule. Worth
checking directly (see recipe below) whether the inner CG's column pool at that iteration is simply
missing a cheaper route combination that `DirectSolver`'s full enumeration finds.

## Recipe: observe the LP-vs-IP gap directly in `RouteCoveringProblem`

Everything below reuses production code paths verbatim (nothing new implemented for this note) --
same functions [[2026-07-24_benders_optimal_with_unclosed_gap_grid_reproducer]] used for its
`grid_n10_p16_s123` case, generalized into a standalone recipe.

```julia
using StationSelection
include("scripts/aggregate_od_route_method_grid.jl")   # build_instance, method_by_label, etc.
using StationSelection: _fixed_assignments_from_y, _route_covering_problem_from_assignments,
    run_aggregate_od_route_column_generation, create_map

data, max_walk = build_instance("zhuzhou", 15, 32, 123, joinpath(@__DIR__, "..", "..", "Data", "base_data"))
model = AggregateODRouteModel(
    8; assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
    route_regularization_weight=10.0, walk_cost_weight=0.1, repositioning_time=20.0,
    max_walking_distance=max_walk, max_wait_time=900.0, detour_factor=2.0, max_stops=4,
)

# The station set to interrogate -- either Benders' reported incumbent or Direct's true optimum;
# for the two "same set" cases above they're identical, so a single y_hat suffices.
y_hat_stations = [11, 22, 92, 100, 117, 133, 158, 202]   # zhuzhou_n15_p32_s123, jobs.txt row 851/864/872

mapping = create_map(model, data)
requests, demand, feasible_pairs = StationSelection._aggregate_od_route_benders_requests(mapping)
assignments, infeasible = _fixed_assignments_from_y(
    data, requests, feasible_pairs, y_hat_stations;
    style=:big_m_nearest, max_walking_distance=model.max_walking_distance,
    allow_walk_only=model.allow_walk_only, allow_same_station=true,
)
@assert isempty(infeasible)

route_problem = _route_covering_problem_from_assignments(model, assignments, y_hat_stations)

# (A) LP/CG bound -- what Benders' own inner solver sees at this y_hat.
cg_result = run_aggregate_od_route_column_generation(
    route_problem, data; optimizer_env=Gurobi.Env(), silent=true,
    max_iterations=200, pricing_time_limit_sec=120, final_ip_time_limit_sec=300,
)
println("CG stop_reason = ", cg_result.cg_stop_reason)
println("CG final IP objective (this is what Benders reports as objective_value) = ",
    cg_result.final_result.objective_value)
println("CG LP/dual bound (Q_bar-equivalent) = ", cg_result.lp_bound)
println("Route pool size at convergence = ", length(cg_result.generated_columns))

# (B) True IP cost via exhaustive enumeration (what Direct actually solves) -- run at higher
# max_stops/route-count ceilings than the sweep's CS_DIRECT_* defaults if this doesn't converge fast.
direct_result = run_aggregate_od_route_column_generation === nothing ? nothing : begin
    direct_solver = DirectSolver(config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true))
    run_opt(data, route_problem, direct_solver)
end
println("Direct enumeration objective (ground truth for this fixed y) = ", direct_result.objective_value)

# (C) The gap itself, and whether it's a pricing-pool-completeness gap (CG's own pool missing the
# route(s) Direct's enumeration uses) or a genuine LP-relaxation weakness (CG's LP bound is looser
# than the true IP optimum even with a complete pool):
println("CG final IP vs Direct IP (pool-completeness gap): ",
    cg_result.final_result.objective_value - direct_result.objective_value)
println("CG LP bound vs Direct IP (LP-relaxation-weakness gap): ",
    cg_result.lp_bound - direct_result.objective_value)
```

Reading the three numbers this prints:
- If **(C)'s first line is ~0** (CG's own final IP matches Direct exactly), the bug is entirely in
  the *outer* Benders loop's stopping rule (as in the `grid_n10_p16_s123` reproducer) -- the inner
  route-covering solve for this `y_hat` was fine, Benders just never re-ran/re-checked it at the
  point it stopped.
- If **(C)'s first line is nonzero** (as the identical-`y_hat`-but-different-objective cases above
  suggest it will be here), the inner CG's pricing pool is *itself* incomplete for this `y_hat` --
  i.e. the column-generation pricing pass (`run_aggregate_od_route_column_generation`,
  `pricing/column_generation.jl`) is terminating (hitting `pricing_time_limit_sec`, or a
  `max_visits_per_node`/label-count cap) before it has generated the specific route(s) that
  `DirectSolver`'s full enumeration includes. Check `cg_result.cg_stop_reason` -- anything other than
  `:optimality_proven` means the pool was never certified complete, which is exactly the caveat
  `_certified_qbar` (`benders/y_mw_cut.jl:685`) already throws on (`"requires cg_stop_reason=
  :optimality_proven ... that solve's own pricing pass already IS the certification"`) -- so this
  gap is, in a sense, already flagged by the codebase's own guard, just not surfaced up to
  `analyze_method_compare.jl`'s objective-agreement check.
- **(C)'s second line** isolates genuine LP weakness (present even with a fully-exhausted pool) from
  pool incompleteness -- compare against (C)'s first line to see how much of the total gap is which.

`inner_cg_iterations` / `generated_column_pool_size` are already logged per-iteration in
`experiments/aggregate_od_route_method_compare/iters/<instance>__<method>/
aggregate_od_route_benders_iterations.csv` for every one of the rows in the table above -- worth
checking whether the pool size plateaus (suggesting pricing genuinely exhausted, pointing at (C)'s
second line / true LP weakness) or is still growing when the outer loop stops (pointing at (C)'s
first line / a pricing time/iteration cap cutting off a still-improving pool).

## Not yet done

- Haven't actually run the recipe above end-to-end on these four cases yet -- this note records
  where to look and how, not the (C)-line numbers themselves. Next step before attempting any fix:
  run it on `zhuzhou_n15_p32_s123` (jobs.txt row 851/864/872) first, since it's the cleanest
  identical-station-set case.
- Haven't checked whether raising `pricing_time_limit_sec`/`CS_INNER_PRICING_TIME` (currently 120s
  per the sweep's defaults, see `run_method_compare_task.jl`'s env var table) alone closes the gap on
  these specific cases -- if it does, this is a tuning issue for the sweep's time budget, not a
  correctness bug in the pricing algorithm itself.
- The `grid_n15_p8_s42` case (byte-different station sets, `[2,4,5,6,7,8,11,14]` /
  `[1,5,7,8,9,10,11,12]` vs Direct's `[2,4,5,6,7,8,10,11]`) is still the *other* shape of this bug
  (outer loop settling on a different, worse `y`, per the original `grid_n10_p16_s123` reproducer's
  mechanism) -- not yet re-examined with the recipe above; only the two zhuzhou p32 same-station-set
  cases were singled out for that.
