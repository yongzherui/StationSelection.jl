"""Does the Benders decomposition of the joint routing+assignment model actually solve it?

End-to-end exercise of `AggregateODRouteJointRoutingAssignmentFormulation` +
`BendersSolver` (`:direct_enumeration` oracle) on the Study 1 reference cell: Zhuzhou
n=10, p=8, seed 42, `max_stops=4`.

# What is verified, and why these numbers

The Benders subproblem must be an LP -- the cut IS its dual -- so what the decomposition
converges to is the optimum of the MIXED model: `y` binary, `theta`/`x_walk` continuous.
That is a different number from the all-binary direct MIP, by this formulation's LP-IP
gap. So each arm reports:

  benders      decomposed mixed optimum        (the thing under test)
  mixed_mono   same mixed model, monolithic    MUST equal `benders` -- the exactness test
  direct_mip   all-binary, same column pool    MUST be >= `benders` -- the LP-IP gap

`mixed_mono` is built by taking the package's own shared master body at
`relax_integrality=true` over the same enumerated pool and re-declaring `y` binary, so it
is the identical model the master/subproblem pair decomposes -- not a re-derivation that
could disagree for its own reasons.

# Arms (`BJ_ARMS`, comma-separated)

- `multicut_s1`      -- the baseline: default `MultiCut(:scenario)`, 1 scenario.
- `singlecut_s1`     -- explicit `MasterFormulation(f; cut_mode=SingleCut())`. `SingleCut`
                        aggregates the scenarios' cut data into one row, which is the SUM
                        of the per-scenario cuts and therefore valid for the same reason,
                        just weaker. With one scenario the two modes are the same cut, so
                        this arm is a wiring test (does the `Theta` keyed at group 0 get
                        found and cut?), not a strength test -- `multicut_s3` vs
                        `singlecut_s3` is where the modes can actually differ.
- `restricted_scope` -- formulation `max_stops=6`, subproblem capped at 4. The master
                        carries no `max_stops` dependence at all (its rows are the station
                        budget and endpoint feasibility, and the map keys off
                        `max_walking_distance`), so this MUST return exactly
                        `multicut_s1`'s objective while reporting
                        `benders_optimality_scope == "max_stops_restricted"`. That identity
                        is what makes the narrowing path testable without enumerating
                        `max_stops=6`, which is not tractable.
- `multicut_s3`      -- 3 scenarios. Exercises the per-scenario subproblem fan-out and
                        `MultiCut` with more than one cut group; `y` is shared across
                        scenarios, so this is a genuinely different (harder) master.
- `singlecut_s3`     -- the same at `SingleCut`: same optimum, but expected to need MORE
                        iterations than `multicut_s3`, since one aggregated row carries
                        what three separate rows carried.
- `infeasible_gate`  -- `k=1`, through `run_opt`. Most Zhuzhou OD pairs are further apart
                        than `2 * max_walking_distance`, so they carry no direct-walk
                        fallback and DO get endpoint-feasibility rows; one station cannot
                        be near every required location. Expects `SOLVE_INFEASIBLE` with
                        `metadata["infeasibility_reason"]` set -- i.e. the refutation comes
                        back as an ANSWER, not as a thrown error.
- `infeasible_master` -- the same `k=1`, but `build_model` + `optimize_model` DIRECTLY,
                        bypassing `run_opt`'s gate. This is the only way to reach the
                        infeasible-master branch, since the gate solves exactly the
                        master's own row set and so always refutes first. Expects
                        `SOLVE_INFEASIBLE` with a `master_*` stop reason. Worth testing
                        despite being unreachable via `run_opt`: benchmark scripts call
                        `build_model`/`optimize_model` directly all the time.

Also checked across arms: the final gap is 0, the reported objective is the UB (never the
master's lower bound), and the `reduced_cost(y)` cross-check agrees with the aggregated
linking duals (a per-scenario diagnostic; see `benders/subproblem.jl`).

Usage: sbatch benchmarks/diagnostics/run_benders_n10.sh
Env: BJ_N BJ_P BJ_S BJ_SEED BJ_MAX_STOPS BJ_MAX_ITERS BJ_ARMS
"""

using StationSelection
using JuMP
using Printf
include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N = parse(Int, get(ENV, "BJ_N", "10"))
const P = parse(Int, get(ENV, "BJ_P", "8"))
const SEED = parse(Int, get(ENV, "BJ_SEED", "42"))
const MAX_STOPS = parse(Int, get(ENV, "BJ_MAX_STOPS", "4"))
# Benders over binary `y` terminates finitely -- each cut is tight at the `yhat` it came
# from, so a repeated incumbent closes the gap -- but the bound is the number of feasible
# station sets (C(10,5) = 252 here), so the cap has to be comfortably above that to
# distinguish "converged" from "ran out of iterations".
const MAX_ITERS = parse(Int, get(ENV, "BJ_MAX_ITERS", "500"))
const ARMS = split(get(ENV, "BJ_ARMS",
    "multicut_s1,singlecut_s1,restricted_scope,multicut_s3,singlecut_s3," *
    "infeasible_gate,infeasible_master"), ',')
# Deliberately below any feasible station count: one station cannot sit near both endpoints
# of every demand group that has no direct-walk fallback.
const INFEASIBLE_K = 1
const MAX_ROUTES = 200_000
const ENUM_LIMIT = 300.0

_formulation(max_stops::Int) = AggregateODRouteJointRoutingAssignmentFormulation(
    ; BENCHMARK_BASELINE..., max_stops=max_stops)

"""One `(scenarios, effective max_stops)` monolithic reference pair, computed once.

`effective` is the SUBPROBLEM's `max_stops`, not the formulation's: that is the route
universe Benders actually searched, so it is the universe the reference has to be built
over for the comparison to mean anything (this is the whole point of the
`restricted_scope` arm).
"""
const REFERENCE_CACHE = Dict{Tuple{Int, Int}, Any}()

function reference(n_scen::Int, effective_max_stops::Int)
    return get!(REFERENCE_CACHE, (n_scen, effective_max_stops)) do
        problem, _k, _meta = benchmark_problem(@__DIR__, "BJ", N, P, n_scen, SEED)
        formulation = _formulation(effective_max_stops)
        data = problem.data
        mapping = create_aggregate_od_route_map(problem, formulation, data)
        columns = StationSelection.enumerate_joint_routing_assignment_columns(
            problem, formulation, data; max_routes=MAX_ROUTES, time_limit_sec=ENUM_LIMIT)

        function solve_ref(; mixed::Bool)
            build = StationSelection._build_joint_routing_assignment_model(
                data, mapping, problem.k, formulation;
                relax_integrality=mixed, initial_columns=columns)
            m = build.model
            set_silent(m)
            set_time_limit_sec(m, 900.0)
            # `relax_integrality=true` made EVERY family continuous; re-declaring only `y`
            # binary is what turns it into the mixed model Benders actually solves.
            mixed && JuMP.set_binary.(m[:y])
            optimize!(m)
            return (status=termination_status(m), objective=objective_value(m),
                    y=findall(v -> v > 0.5, value.(m[:y])))
        end

        mixed = solve_ref(mixed=true)
        direct = solve_ref(mixed=false)
        @printf("  reference(s=%d, ms=%d): %d cols | mixed_mono %s %.6f y=%s | direct_mip %s %.6f y=%s\n",
                n_scen, effective_max_stops, length(columns),
                mixed.status, mixed.objective, string(mixed.y),
                direct.status, direct.objective, string(direct.y))
        flush(stdout)
        (n_columns=length(columns), mixed=mixed, direct=direct)
    end
end

"""Run one arm and return its row, printing as it goes."""
function run_arm(arm::AbstractString)
    n_scen = endswith(arm, "_s3") ? 3 : 1
    formulation_max_stops = arm == "restricted_scope" ? 6 : MAX_STOPS
    subproblem_max_stops = MAX_STOPS
    cut_mode = startswith(arm, "singlecut") ? SingleCut() : MultiCut()

    problem, k, _meta = benchmark_problem(@__DIR__, "BJ", N, P, n_scen, SEED)
    formulation = _formulation(formulation_max_stops)
    # The monolith goes straight to `run_opt` (the ordinary call, which derives the master
    # at the default MultiCut); a non-default cut_mode needs the master constructed
    # explicitly, which is the documented reason that entry point exists.
    solve_formulation = cut_mode isa MultiCut ? formulation :
        AggregateODRouteJointRoutingAssignmentMasterFormulation(formulation; cut_mode=cut_mode)

    solver = BendersSolver(
        config=SolverOptions(silent=true, time_limit_sec=300.0),
        max_iterations=MAX_ITERS,
        optimality_tol=1e-6,
        subproblem=BendersSubproblemConfig(
            max_stops=subproblem_max_stops, max_routes=MAX_ROUTES,
            enumeration_time_limit_sec=ENUM_LIMIT),
        total_time_limit_sec=1800.0,
    )

    @printf("\n=== arm %s (s=%d, formulation max_stops=%d, subproblem cap=%d, %s) ===\n",
            arm, n_scen, formulation_max_stops, subproblem_max_stops,
            typeof(cut_mode).name.name)
    flush(stdout)
    result = run_opt(problem, solve_formulation, solver)
    md = result.metadata
    obj = something(result.objective_value, NaN)
    @printf("status %s | obj(UB) %.6f | LB %.6f | gap %.3e | iters %d (best %d) | cuts %d\n",
            result.termination_status, obj, md["benders_lower_bound"], md["benders_gap"],
            md["benders_iterations"], md["benders_best_iteration"], md["benders_cuts_added"])
    @printf("stop %s | scope %s | cols %d (enum %.2fs) | wall %.2fs = master %.2fs + sub %.2fs\n",
            md["benders_stop_reason"], md["benders_optimality_scope"],
            md["benders_enumerated_columns"], md["benders_enumeration_sec"],
            result.runtime_sec, md["benders_master_sec"], md["benders_subproblem_sec"])
    stations = result.solution.selected_station_indices
    @printf("stations idx %s -> ids %s | scenario costs %s\n",
            string(stations),
            string([get_station_id(result.mapping, j) for j in stations]),
            string(round.(result.solution.scenario_objectives; digits=3)))
    flush(stdout)

    ref = reference(n_scen, subproblem_max_stops)
    return (arm=arm, n_scen=n_scen, obj=obj, ref=ref, metadata=md,
            status=string(result.termination_status),
            iterations=md["benders_iterations"], cuts=md["benders_cuts_added"],
            scope=md["benders_optimality_scope"])
end

@printf("instance: zhuzhou n=%d p=%d seed=%d max_stops=%d | arms: %s\n",
        N, P, SEED, MAX_STOPS, join(ARMS, ", "))
flush(stdout)

"""The two infeasibility arms. Returns `nothing` for the row list -- these assert their
own outcome rather than being compared against a monolithic reference (there is no optimum
to compare)."""
function run_infeasible_arm(arm::AbstractString)
    problem, _k, _meta = benchmark_problem(@__DIR__, "BJ", N, P, 1, SEED)
    small = StationSelectionProblem(problem.data, INFEASIBLE_K;
                                    max_walking_distance=problem.max_walking_distance)
    formulation = _formulation(MAX_STOPS)
    solver = BendersSolver(
        config=SolverOptions(silent=true, time_limit_sec=300.0),
        max_iterations=MAX_ITERS,
        subproblem=BendersSubproblemConfig(
            max_stops=MAX_STOPS, max_routes=MAX_ROUTES,
            enumeration_time_limit_sec=ENUM_LIMIT))

    @printf("\n=== arm %s (k=%d, expected infeasible) ===\n", arm, INFEASIBLE_K)
    flush(stdout)
    if arm == "infeasible_gate"
        result = run_opt(small, formulation, solver)
        reason = get(result.metadata, "infeasibility_reason", nothing)
        @printf("status %s | reason %s\n", result.termination_status,
                isnothing(reason) ? "<none>" : reason)
        flush(stdout)
        return (arm=arm, status=string(result.termination_status),
                detail=isnothing(reason) ? "<no infeasibility_reason>" : "reason set",
                ok_extra=!isnothing(reason))
    else
        # Bypass run_opt (and therefore its gate) to reach the infeasible-master branch.
        build = build_model(small, formulation, solver)
        result = StationSelection.optimize_model(build, solver)
        stop = get(result.metadata, "benders_stop_reason", "<none>")
        @printf("status %s | stop %s | LB %.6f\n", result.termination_status, stop,
                result.metadata["benders_lower_bound"])
        flush(stdout)
        return (arm=arm, status=string(result.termination_status), detail=stop,
                ok_extra=startswith(String(stop), "master_"))
    end
end


rows = Any[]
infeasible_rows = Any[]
for a in ARMS
    arm = strip(a)
    if startswith(arm, "infeasible")
        push!(infeasible_rows, run_infeasible_arm(arm))
    else
        push!(rows, run_arm(arm))
    end
end

# ------------------------------------------------------------------ checks
println("\n=== checks ===")
checks = Tuple{String, Bool, String}[]
push_check!(name, ok, detail) = push!(checks, (name, ok, detail))

for r in rows
    # Guard FIRST: a reference that hit its own time limit reports an incumbent, not an
    # optimum, and would make the equality check below fail as though Benders were wrong.
    # Attributing a reference timeout to the thing under test is the worst way to read this
    # script, so it gets its own check rather than being folded into the comparison.
    push_check!("$(r.arm): references are optima",
        string(r.ref.mixed.status) == "OPTIMAL" && string(r.ref.direct.status) == "OPTIMAL",
        "mixed $(r.ref.mixed.status) / direct $(r.ref.direct.status)")
    push_check!("$(r.arm): == mixed_mono",
        isapprox(r.obj, r.ref.mixed.objective; rtol=1e-6, atol=1e-6),
        @sprintf("%.6f vs %.6f (diff %.3e)", r.obj, r.ref.mixed.objective,
                 r.obj - r.ref.mixed.objective))
    push_check!("$(r.arm): <= direct_mip",
        r.obj <= r.ref.direct.objective + 1e-6,
        @sprintf("%.6f vs %.6f (LP-IP gap %.3f%%)", r.obj, r.ref.direct.objective,
                 100 * (r.ref.direct.objective - r.obj) / max(1e-9, abs(r.ref.direct.objective))))
    push_check!("$(r.arm): converged OPTIMAL",
        r.metadata["benders_converged"] === true && r.status == "OPTIMAL",
        "$(r.status) / $(r.metadata["benders_stop_reason"])")
    push_check!("$(r.arm): objective is the UB",
        isapprox(r.obj, r.metadata["benders_upper_bound"]; rtol=1e-9, atol=1e-9),
        @sprintf("%.6f vs UB %.6f", r.obj, r.metadata["benders_upper_bound"]))
end

by_arm = Dict(r.arm => r for r in rows)

# Cut mode changes which rows the master carries, never the optimum it converges to.
for (multi, single) in (("multicut_s1", "singlecut_s1"), ("multicut_s3", "singlecut_s3"))
    (haskey(by_arm, multi) && haskey(by_arm, single)) || continue
    a, b = by_arm[multi], by_arm[single]
    push_check!("$multi == $single",
        isapprox(a.obj, b.obj; rtol=1e-6, atol=1e-6),
        @sprintf("%.6f vs %.6f | iters %d vs %d, cuts %d vs %d",
                 a.obj, b.obj, a.iterations, b.iterations, a.cuts, b.cuts))
end

# The narrowing path: capping the subproblem below the formulation reproduces the capped
# model exactly (the master has no max_stops dependence), and says so in the scope.
if haskey(by_arm, "restricted_scope") && haskey(by_arm, "multicut_s1")
    a, b = by_arm["multicut_s1"], by_arm["restricted_scope"]
    push_check!("restricted_scope == multicut_s1",
        isapprox(a.obj, b.obj; rtol=1e-6, atol=1e-6),
        @sprintf("%.6f vs %.6f", a.obj, b.obj))
    push_check!("restricted_scope labels the narrowing",
        b.scope == "max_stops_restricted", b.scope)
    push_check!("multicut_s1 claims full universe",
        a.scope == "full_route_universe", a.scope)
end

for r in infeasible_rows
    # A proven-infeasible instance is an ANSWER, not a usage error -- the whole point of
    # both fixes being tested here.
    push_check!("$(r.arm): INFEASIBLE not NOT_SOLVED",
        r.status == "INFEASIBLE", r.status)
    push_check!("$(r.arm): reports how it knew", r.ok_extra, string(r.detail))
end

for (name, ok, detail) in checks
    @printf("%-44s %s   %s\n", name, ok ? "PASS" : "FAIL", detail)
end
n_fail = count(c -> !c[2], checks)
@printf("\n%d checks, %d failed\n", length(checks), n_fail)
n_fail == 0 || error("verification failed")
println("ALL CHECKS PASSED")
