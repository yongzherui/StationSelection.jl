"""Pool composition along an actual Benders trajectory: per iteration, per scenario, by how
many unbuilt stations each column touches.

`benders_pool_and_cut_anatomy.jl` samples four hand-picked incumbents at one scenario. This
runs the real loop instead -- master solve, incumbent, subproblem solve, cut, repeat -- and
dumps the pool after every iteration for every scenario. Two questions it can answer that the
sampled version cannot:

  how the pool's composition EVOLVES  -- the pool accumulates across Benders iterations by
    design (validity comes from exhaustion at extraction, not from a frozen pool), so the
    interesting quantity is whether later iterations keep adding unbuilt-touching columns or
    settle down.
  whether scenarios differ -- each scenario has its own demand groups, its own subproblem and
    its own pool, so "per scenario" is a real axis and not a repetition.

Classification per column, at the iteration's own incumbent:

  need_j (per unbuilt station, per iteration)  -- the question pool composition CANNOT answer.
    Pool holdings say whether a column touching `j` is present; they do not say whether any
    column through `j` MATTERS. What matters is whether some column through `j` has a binding
    dual constraint at THAT ITERATION's raw (pre-completion) duals, since that is the only
    thing that forces `gamma_j` above zero:

        need_j = max(0, max over columns whose unbuilt assign set is exactly {j} of
                        [ sum_p alpha_p - sum(gamma at built stations) - f_c ])

    computed over the ENUMERATED universe, so it is what the duals require and not what the
    arm's pool happens to hold. `need_j = 0` means no column through `j` is worth anything at
    these duals, so the truthful charge is zero and every unit the completion puts there is
    invented. Reported against `charged_j`, what the arm's cut actually carries.

  n_unbuilt_assign     how many DISTINCT unbuilt stations the column assigns at. Its theta is
                       pinned to 0 whenever this is > 0, so the column cannot improve the
                       objective -- it exists only for its dual constraint.
  all_unbuilt_assign   every assignment station is unbuilt.
  route_all_unbuilt    every VISITED station is unbuilt -- a route living entirely outside the
                       built set. These are the ones the restricted search can never reach.

The loop is mirrored from `solvers/benders/loop.jl` rather than driven through `run_opt`,
because there is no per-iteration hook. It reproduces the same four steps and the same
`objective_bound` lower bound; it does NOT reproduce the stopping rules beyond the bound gap
and the iteration cap, so treat its iteration counts as indicative and take converged counts
from the arms table in the notes.

Usage: sbatch benchmarks/diagnostics/run_benders_pool_trajectory.sh
Env: PT_N PT_P PT_S PT_SEED PT_MAX_STOPS PT_MAX_ITER PT_OUT
"""

using StationSelection
using JuMP
using Printf
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N = parse(Int, get(ENV, "PT_N", "10"))
const P = parse(Int, get(ENV, "PT_P", "8"))
const S = parse(Int, get(ENV, "PT_S", "3"))
const SEED = parse(Int, get(ENV, "PT_SEED", "42"))
const MAX_STOPS = parse(Int, get(ENV, "PT_MAX_STOPS", "4"))
const MAX_ITER = parse(Int, get(ENV, "PT_MAX_ITER", "60"))
const TAG = "n$(N)_p$(P)_s$(S)_seed$(SEED)_ms$(MAX_STOPS)"
const OUT = get(ENV, "PT_OUT", joinpath(@__DIR__, "results", "pool_trajectory", TAG))
mkpath(OUT)

problem, k, _meta = benchmark_problem(@__DIR__, "PT", N, P, S, SEED)
data = problem.data
monolith = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops = MAX_STOPS)

make_solver(oracle) = BendersSolver(
    config = SolverOptions(silent = true),
    max_iterations = MAX_ITER,
    subproblem = BendersSubproblemConfig(
        oracle = oracle, max_stops = MAX_STOPS, max_routes = 500_000,
        enumeration_time_limit_sec = 600.0, max_cg_iterations = 500,
        cg_pricing_time_limit_sec = 300.0,
    ),
)

@printf("instance: zhuzhou n=%d p=%d s=%d seed=%d k=%d max_stops=%d | max_iter %d\nout: %s\n\n",
        N, P, S, SEED, k, MAX_STOPS, MAX_ITER, OUT)
flush(stdout)

assign_stations(c) = Set(Iterators.flatten(((j, kk) for (_p, j, kk) in c.assignments)))
join_ids(xs) = join(sort(collect(xs)), "|")

# The universe, for `need_j`. The arms are CG-based so neither pool is the universe; what the
# duals REQUIRE has to be read off every column that exists, not off what got priced.
enum_solver = BendersSolver(config = SolverOptions(silent = true),
    subproblem = BendersSubproblemConfig(oracle = :direct_enumeration, max_stops = MAX_STOPS,
        max_routes = 500_000, enumeration_time_limit_sec = 600.0))
enum_build = build_model(problem, monolith, enum_solver)
enum_subs = enum_build.model[:benders_subproblem_builds]
universe = [collect(values(b.model[:joint_routing_assignment_columns])) for b in enum_subs]
walk_w = Float64(enum_subs[1].model[:joint_routing_assignment_walk_cost_weight])
ucost(s, c) = joint_routing_assignment_column_cost(enum_subs[s].model, data, enum_build.mapping, c)
@printf("enumerated universe for need_j: %s columns\n\n", join(length.(universe), "+"))

"""What station `j` must be charged at these RAW duals, from every column whose only unbuilt
assignment station is `j`. Returns `(need_j, n_candidates, best_column)`."""
function station_need(s, j, built, a, go, gd)
    best, bestc, n = -Inf, nothing, 0
    for c in universe[s]
        ast = assign_stations(c)
        setdiff(ast, built) == Set([j]) || continue
        n += 1
        v = -ucost(s, c)
        for (p, jj, kk) in c.assignments
            v += get(a, (s, p), 0.0)
            jj in built && (v -= get(go, ((s, p), jj), 0.0))
            kk in built && (v -= get(gd, ((s, p), kk), 0.0))
        end
        v > best && (best = v; bestc = c)
    end
    return (isfinite(best) ? max(0.0, best) : 0.0), n, bestc
end

col_io = open(joinpath(OUT, "trajectory_columns.csv"), "w")
println(col_io, "arm,iteration,scenario,incumbent,route,n_stops,n_pax,assignments,tau," *
    "assign_stations,unbuilt_assign_stations,n_unbuilt_assign,n_unbuilt_visits," *
    "all_unbuilt_assign,route_all_unbuilt,is_new_this_iteration")

need_io = open(joinpath(OUT, "trajectory_station_need.csv"), "w")
println(need_io, "arm,iteration,scenario,incumbent,station,need_j,charged_j," *
    "n_candidate_cols,best_route,best_assignments,best_tau,best_f_c")

sum_io = open(joinpath(OUT, "trajectory_summary.csv"), "w")
println(sum_io, "arm,iteration,scenario,incumbent,lower_bound,subproblem_cost,pool_size," *
    "added_this_iteration,n_unbuilt_0,n_unbuilt_1,n_unbuilt_2,n_unbuilt_3plus," *
    "all_unbuilt_assign,route_all_unbuilt")

for arm in (:column_generation, :column_generation_activated)
    solver = make_solver(arm)
    build = build_model(problem, monolith, solver)
    m = build.model
    mapping = build.mapping
    subs = m[:benders_subproblem_builds]
    set_silent(m)
    seen = [Set{Any}() for _ in 1:n_scenarios(data)]

    println("="^130)
    @printf("arm %s\n", arm)
    println("="^130)
    @printf("%5s %9s %12s %8s  %s\n", "iter", "LB", "sub cost", "pool", "per-scenario: pool (n=0/1/2/3+) all-unbuilt route-all-unbuilt")

    lower = -Inf
    for iteration in 1:MAX_ITER
        optimize!(m)
        JuMP.termination_status(m) == MOI.OPTIMAL || (println("  master $(JuMP.termination_status(m)) -- stopping"); break)
        lower = max(lower, JuMP.objective_bound(m))
        incumbent = StationSelection.extract_incumbent(build, mapping, m)
        built = Set(j for j in 1:N if incumbent[j] > 0.5)
        sub = StationSelection.solve_subproblem(build, mapping, m, incumbent, solver)

        parts = String[]
        matter_parts = String[]
        for s in 1:n_scenarios(data)
            pool = collect(values(subs[s].model[:joint_routing_assignment_columns]))
            d = zeros(Int, 4)
            n_all, n_route_all, n_new = 0, 0, 0
            for c in pool
                ast = assign_stations(c)
                un = setdiff(ast, built)
                unv = setdiff(Set(c.route), built)
                nu = length(un)
                d[min(nu, 3) + 1] += 1
                all_un = !isempty(un) && un == ast
                route_all = length(unv) == length(unique(c.route))
                all_un && (n_all += 1)
                route_all && (n_route_all += 1)
                key = (Tuple(c.route), Tuple(sort(c.assignments)))
                is_new = !(key in seen[s])
                is_new && (push!(seen[s], key); n_new += 1)
                println(col_io, join((arm, iteration, s, join_ids(built), join(c.route, ">"),
                    length(c.route), length(c.assignments),
                    join(("$(p):$(j):$(kk)" for (p, j, kk) in sort(c.assignments)), "|"),
                    c.tau, join_ids(ast), join_ids(un), nu, length(unv),
                    all_un, route_all, is_new), ","))
            end
            # RAW duals -- `solve_subproblem` completes its own copy, the model's are untouched.
            a, go, gd = extract_joint_routing_assignment_duals(subs[s].model)
            ca, cgo, cgd = deepcopy(a), deepcopy(go), deepcopy(gd)
            arm === :column_generation_activated && StationSelection._benders_activated_complete_duals!(
                ca, cgo, cgd, incumbent, data, mapping, s, walk_w)
            n_matter = 0
            for j in sort(collect(setdiff(1:N, built)))
                need, ncand, bc = station_need(s, j, built, a, go, gd)
                charged = sum(v for gg in (cgo, cgd) for ((k2, jj), v) in gg
                              if jj == j && k2[1] == s; init = 0.0)
                need > 1e-6 && (n_matter += 1)
                println(need_io, join((arm, iteration, s, join_ids(built), j, need, charged,
                    ncand,
                    bc === nothing ? "" : join(bc.route, ">"),
                    bc === nothing ? "" : join(("$(p):$(jj):$(kk)" for (p, jj, kk) in sort(bc.assignments)), "|"),
                    bc === nothing ? "" : string(bc.tau),
                    bc === nothing ? "" : string(ucost(s, bc))), ","))
            end
            push!(matter_parts, "s$(s):$(n_matter)/$(length(setdiff(1:N, built)))")

            cost_s = sub.scenarios[findfirst(r -> r.scenario == s, sub.scenarios)].objective
            println(sum_io, join((arm, iteration, s, join_ids(built), lower, cost_s,
                length(pool), n_new, d[1], d[2], d[3], d[4], n_all, n_route_all), ","))
            push!(parts, @sprintf("s%d: %d (%d/%d/%d/%d) %d %d", s, length(pool),
                                  d[1], d[2], d[3], d[4], n_all, n_route_all))
        end
        @printf("      unbuilt stations with need_j > 0 (columns that MATTER): %s\n",
                join(matter_parts, " | "))
        @printf("%5d %9.2f %12.2f %8d  %s\n", iteration, lower, sub.total_objective,
                sum(length(values(subs[s].model[:joint_routing_assignment_columns]))
                    for s in 1:n_scenarios(data)), join(parts, " | "))
        flush(stdout)

        StationSelection.add_benders_cut!(build, mapping, m, sub, solver)
        if sub.total_objective - lower <= 1e-6 * max(1.0, abs(sub.total_objective))
            println("  bounds met -- converged")
            break
        end
    end
end

close(col_io); close(sum_io); close(need_io)

open(joinpath(OUT, "README.md"), "w") do io
    println(io, """
# Pool trajectory — $TAG

Zhuzhou n=$N, p=$P, **s=$S**, seed $SEED, k=$k, max_stops=$MAX_STOPS. Two arms
(`:column_generation`, `:column_generation_activated`) driven through a mirror of the real
Benders loop, with the pool dumped after every iteration for every scenario.

`incumbent` is the master's station set at that iteration — the point the cut is anchored at.
(Earlier notes called this an "anchor"; it is just an incumbent.)

## trajectory_summary.csv — one row per (arm, iteration, scenario)

| field | meaning |
| --- | --- |
| `lower_bound` | the master's `objective_bound`, monotone |
| `subproblem_cost` | that scenario's exact second-stage cost at this incumbent |
| `pool_size`, `added_this_iteration` | columns held, and how many were new this iteration |
| `n_unbuilt_0/1/2/3plus` | pool columns by how many DISTINCT unbuilt stations they assign at |
| `all_unbuilt_assign` | every assignment station unbuilt |
| `route_all_unbuilt` | every VISITED station unbuilt — routes living wholly outside the built set |

A column with `n_unbuilt_assign > 0` has its `theta` pinned to 0 at that incumbent, so it
cannot improve the objective — it is in the pool purely for its dual constraint.

## trajectory_columns.csv — one row per (arm, iteration, scenario, column)

The same classification per column, plus `route`, `assignments` (`p:j:k`), `tau`, and
`is_new_this_iteration`. Note the pool accumulates across iterations by design, so a column
appears once per iteration from the point it enters; filter on `is_new_this_iteration` to get
arrival times instead of holdings.
""")
end

@printf("\nwrote %s\n", OUT)
foreach(f -> @printf("  %-28s %9.1f KB\n", f, filesize(joinpath(OUT, f)) / 1024),
        sort(readdir(OUT)))
