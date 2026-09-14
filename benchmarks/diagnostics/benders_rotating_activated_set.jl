"""Rotate `r` "most hopeful" unbuilt stations into the priced set each Benders iteration, and
see whether the accumulated cut family becomes strong without ever pricing all `n` stations.

STRICT COMPLETION ONLY. No row generation, no LPO, no Pareto step. The only change from
`:column_generation_activated` is WHICH stations pricing is allowed to search:

    r = 0   price over S            -- today's activated oracle, the baseline
    r > 0   price over S union T_i  -- T_i chosen fresh each iteration

Everything else is untouched: the closed-form completion still fills in every station outside
the priced set, and the cut is read off those duals.

# Why any `T_i` is sound

The completion's validity argument never mentions WHICH set was searched -- it needs only
that the search exhausted over the priced set and that the closed form covers everything
outside it. Substituting `S union T` for `S` leaves every line intact (see
`_benders_activated_complete_duals!`). So no new soundness claim, no conjecture, and the
per-iteration cut is valid globally exactly as it is today.

Implementation note: the subproblem's `y` is fixed at the TRUE incumbent (so `Q_s` is the
real second-stage cost), while the completion is handed a PSEUDO-incumbent with `T`'s entries
set to 1. That is the whole mechanism -- the completion skips `T`, so `T`'s candidates stay
above the pricer's `rho > 0` filter and the search covers `S union T`. `T`'s own `gamma` then
comes from the LP's own dual on its linking rows, i.e. the honest value, exactly as it does
for a built station.

# How `T_i` is chosen -- from the previous cuts, not from a fresh formula

At iteration `i` the master holds a family of cuts. For a station `j` unbuilt at THIS
iteration, the family's best (least over-promising) statement about building `j` is

    score_j = min over existing cuts of Gamma^cut_j

because the master takes the MAX over cuts, so the cut with the smallest coefficient on `y_j`
is the one that governs there. A small score means the family already says "building `j`
barely helps", which is the honest answer 93% of the time (`notes/2026-09-10_*_why_weak.md`)
and needs no pricing. A large score means every cut still over-promises `j`, so that is where
the master will go next -- and that is what to price.

`T_i` = the `r` unbuilt stations with the LARGEST score. At iteration 1 no cuts exist, so the
fallback is pure geometry: the stations appearing in the most valid `(group, station)` pairs,
which is what `Gamma^cf_j = sum_p (alpha_p - credit)` is proportional to.

`sum` and `max` over cuts are logged alongside `min` so the choice of aggregate can be
revisited from the data without re-running.

# max_stops

Runs at the FORMULATION's own `max_stops` (10 by default), NOT the 4 that earlier diagnostics
forced in order to have an enumerated reference. This test needs no such reference, and 4 was
never the operating point.

# One arm per job

The arms are independent, so this runs as an ARRAY: `SLURM_ARRAY_TASK_ID` selects one
`(n, mode, rule, r)` cell and the task runs only that. Sequential arms in one job would be
both slower and less comparable -- `benders_lpo_arm.jl` already records why (several arms
sharing a machine with a multi-threaded master produced a wrong wall-time reading, the
0.01x-vs-0.99x correction), so the master runs single-threaded here too and each task gets
its own node and its own output files.

`RA_LIST=1` prints the numbered cell list without solving, which is how you get the
`--array` range.

Usage: sbatch --array=1-12 benchmarks/diagnostics/run_benders_rotating_activated_set.sh
Env: RA_N RA_P RA_S RA_SEED RA_MAX_ITER RA_ARM_SEC RA_R RA_MODES RA_RULES RA_PLAIN RA_R0
     RA_THREADS RA_LIST RA_OUT
"""

using StationSelection
using JuMP
using Printf
using Random

include(joinpath(@__DIR__, "..", "lib", "cg_benchmark.jl"))

const N_VALUES = parse.(Int, split(get(ENV, "RA_N", "10,15"), ','))
const P = parse(Int, get(ENV, "RA_P", "8"))
const S = parse(Int, get(ENV, "RA_S", "3"))
const SEED = parse(Int, get(ENV, "RA_SEED", "42"))
const MAX_ITER = parse(Int, get(ENV, "RA_MAX_ITER", "300"))
# The iteration cap is deliberately loose -- cover r=3 needed 46 iterations at n=15, so a
# tight cap turns "does it converge" into "everything hit the cap". The real bound is this
# per-arm wall budget, which lets a stalling arm yield to the next one with its rows already
# on disk.
const ARM_SEC = parse(Float64, get(ENV, "RA_ARM_SEC", "2400"))
const PRICE_SEC = parse(Float64, get(ENV, "RA_PRICE_SEC", "300"))
const R_VALUES = parse.(Int, split(get(ENV, "RA_R", "1,2,3"), ','))
# Selection rules for T_i, all scored off the ACCUMULATED cuts except the two controls.
#   min_all   min over every cut of Gamma_j   -- the family's least over-promising statement
#   last      sum over the LAST iteration's cuts only
#   sum_all   sum over every cut
#   max_all   max over every cut
#   incidence static geometry, no adaptivity  -- control for "does adaptivity matter"
#   random    control for "does the ranking matter at all"
const RULES = Symbol.(split(get(ENV, "RA_RULES", "min_all,last,sum_all,max_all,incidence,random"), ','))
const WITH_PLAIN = get(ENV, "RA_PLAIN", "1") == "1"
# The r=0 baseline is today's oracle and is already known not to converge at any size tested
# (n=15: 60 iterations, gap 9826). With a loose iteration cap it burns its whole per-arm wall
# budget proving that again, so it is opt-in.
const WITH_R0 = get(ENV, "RA_R0", "0") == "1"
# rotate -- ONE block per scenario per iteration, the top-r stations. Covers U over
#           ceil(|U|/r) iterations.
# cover   -- U partitioned into DISJOINT blocks of size r, ALL priced this iteration, one
#           cut each. Every unbuilt station gets an honest coefficient in some cut after a
#           single iteration, at the cost of ceil(|U|/r) searches instead of 1 -- but the
#           pool is per-scenario and SHARED, so block 2 onward starts warm with block 1's
#           columns and only prices the delta.
const MODES = Symbol.(split(get(ENV, "RA_MODES", "rotate,cover"), ','))
# Thread budget. The three scenarios are INDEPENDENT (separate subproblem models, separate
# pools), so they are the natural unit of parallelism; the blocks within a scenario are NOT,
# because they share that scenario's pool by design and each starts warm from the last.
#
# Configuring this correctly matters more than it looks. `build_model` hands the master's
# `SolverOptions` to every subproblem build, so a single `threads = 3` would give the master
# 3 AND each concurrently-solved subproblem 3, i.e. 9 Gurobi threads on 3 CPUs. Instead:
# master gets the whole allocation (it is solved alone), each subproblem gets 1, and the
# scenarios run concurrently -- 3 x 1 = 3. The label search adds nothing on top: with one
# scenario per subproblem model `_run_pricing_round`'s `length(scenarios) > 1` conjunct is
# false, so pricing is single-threaded per call (`label_setting/round.jl`).
const THREADS = parse(Int, get(ENV, "RA_THREADS", string(Threads.nthreads())))
# The pricer. Left unset this resolves to the formulation's default, `:exact` -- whose
# measured exhaustion frontier is n<=20 all scenarios, n=25 to <=5, n=30 only s=1. Past
# that it returns `pricing_inconclusive` and the arm correctly refuses to emit a cut, which
# is what happened to every n=30 cell in array 22510828. `:relaxed_cluster` certifies by
# bounding every real route's reduced cost instead of exhausting the search, and is
# licensed for Benders cuts (see `BendersSubproblemConfig`); it REQUIRES a cluster count.
const PRICER = Symbol(get(ENV, "RA_PRICER", "exact"))
const RC_K = parse(Int, get(ENV, "RA_RC_K", "0"))
const PRICING = PRICER === :exact ? CGPricingConfig() :
    CGPricingConfig(mode = PRICER,
                    relaxed_cluster_count = RC_K > 0 ? RC_K :
                        error("RA_PRICER=$PRICER needs RA_RC_K > 0"))
const TASK = haskey(ENV, "SLURM_ARRAY_TASK_ID") ?
    parse(Int, ENV["SLURM_ARRAY_TASK_ID"]) : nothing
const TAG = "p$(P)_s$(S)_seed$(SEED)"
const OUT = get(ENV, "RA_OUT", joinpath(@__DIR__, "results", "rotating_activated", TAG))
mkpath(OUT)

# BENCHMARK_BASELINE's own max_stops (10) -- deliberately NOT overridden.
const FORMULATION = AggregateODRouteJointRoutingAssignmentFormulation(; BENCHMARK_BASELINE...)
@printf("p=%d s=%d seed=%d | max_stops %s | n in %s | r in %s | rules %s\nout: %s\n\n",
        P, S, SEED, string(FORMULATION.max_stops), string(N_VALUES), string(R_VALUES),
        string(RULES), OUT)
flush(stdout)

"""Per-station `(group, side)` incidence -- pure geometry, the static ranking and the
iteration-1 fallback. `Gamma^cf_j = sum_p (alpha_p - credit)` is proportional to it."""
function station_incidence(problem, mapping, data, n)
    inc = zeros(Int, n)
    for s in 1:n_scenarios(data), (p, (o, d)) in enumerate(mapping.Omega_s[s])
        mapping.Q_s[s][p] > 0 || continue
        for pair in get_valid_jk_pairs(mapping, o, d)
            is_walk_only_pair(pair) && continue
            inc[pair[1]] += 1; inc[pair[2]] += 1
        end
    end
    return inc
end

# ---- the cell list; one array task per entry ----
cells = Tuple{Int, Symbol, Symbol, Int, Bool}[]   # (n, mode, rule, r, plain)
for N in N_VALUES
    WITH_PLAIN && push!(cells, (N, :none, :none, 0, true))
    WITH_R0 && push!(cells, (N, :rotate, :none, 0, false))
    for mode in MODES, rule in RULES, r in R_VALUES
        push!(cells, (N, mode, rule, r, false))
    end
end
if get(ENV, "RA_LIST", "0") == "1"
    println("$(length(cells)) cells -- submit with --array=1-$(length(cells))")
    for (i, c) in enumerate(cells)
        @printf("  %3d  n=%-3d %-7s %-8s r=%d%s\n", i, c[1], c[2], c[3], c[4],
                c[5] ? "  (plain reference)" : "")
    end
    exit(0)
end
isnothing(TASK) || (1 <= TASK <= length(cells)) ||
    error("SLURM_ARRAY_TASK_ID $TASK outside 1:$(length(cells))")
const SUFFIX = isnothing(TASK) ? "" : "_task$(TASK)"

it_io = open(joinpath(OUT, "iterations$(SUFFIX).csv"), "w")
println(it_io, "n,arm,rule,r,iteration,incumbent,unbuilt,priced_extra,lower_bound," *
    "upper_bound,gap,cuts_added,cg_iterations_total,columns_added_total,pricing_sec,wall_sec")

# Every cut, in full: its constant and its coefficient on EVERY station. This is what lets
# the cut family be examined directly -- sparsity, where the mass sits, and how much of the
# lower-bound movement each cut is responsible for.
cut_io = open(joinpath(OUT, "cuts$(SUFFIX).csv"), "w")
println(cut_io, "n,arm,rule,r,iteration,scenario,block,cut_constant,station,built,gamma," *
    "lb_before,ub_after")

sc_io = open(joinpath(OUT, "station_scores$(SUFFIX).csv"), "w")
println(sc_io, "n,arm,rule,r,iteration,station,unbuilt,score_min_over_cuts," *
    "score_sum_over_cuts,score_max_over_cuts,score_last_iteration,incidence,selected,gamma_this_cut")

summary = Tuple{Int, String, Int, Int, Int, Float64, Float64, Float64, Float64}[]

"""One arm. `r` extra stations rotated in per iteration; `r = 0` is today's oracle.
`plain` runs `:column_generation` instead, as the reference."""
function run_arm(N::Int, rule::Symbol, r::Int; plain::Bool = false, mode::Symbol = :rotate)
    problem, k, _meta = benchmark_problem(@__DIR__, "RA", N, P, S, SEED)
    data = problem.data
    formulation = FORMULATION
    arm = plain ? "plain" : "$(mode)_$(rule)_r$(r)"
    oracle = plain ? :column_generation : :column_generation_activated
    solver = BendersSolver(
        config = SolverOptions(silent = true),
        max_iterations = MAX_ITER,
        subproblem = BendersSubproblemConfig(
            oracle = oracle, pricing = PRICING, max_cg_iterations = 500,
            cg_pricing_time_limit_sec = PRICE_SEC),
    )
    build = build_model(problem, formulation, solver)
    rng = 20260910 + N + r
    m = build.model
    mapping = build.mapping
    subs = m[:benders_subproblem_builds]
    set_silent(m)
    # master: the whole allocation. subproblems: one each, since they run concurrently.
    THREADS > 0 && set_optimizer_attribute(m, "Threads", THREADS)
    for b in subs
        set_silent(b.model)
        set_optimizer_attribute(b.model, "Threads", 1)
    end
    y = m[:y]
    theta = m[:benders_cut_variables]::Dict{Int, VariableRef}
    cut_mode = m[:aggregate_od_route_formulation].cut_mode
    walk_w = Float64(subs[1].model[:joint_routing_assignment_walk_cost_weight])
    incidence = station_incidence(problem, mapping, data, N)

    # every cut's Gamma vector, grouped by the iteration that produced it, so a rule can
    # look at the last round only or at the whole family.
    history = Vector{Vector{Dict{Int, Float64}}}()
    lower, best_ub = -Inf, Inf
    t_arm = time()
    n_cuts = 0

    println("="^132)
    @printf("arm %s   (oracle %s, r = %d)\n", arm, oracle, r)
    println("="^132)
    @printf("%5s %14s %16s %11s %11s %9s %6s %8s\n",
            "iter", "incumbent", "priced extra T", "LB", "UB", "gap", "cuts", "wall")

    for iteration in 1:MAX_ITER
        optimize!(m)
        JuMP.termination_status(m) == MOI.OPTIMAL || break
        lower = max(lower, JuMP.objective_bound(m))
        incumbent = StationSelection.extract_incumbent(build, mapping, m)
        built = Set(j for j in 1:N if incumbent[j] > 0.5)
        unbuilt = sort(collect(setdiff(1:N, built)))

        # ---- score the unbuilt stations ----
        flat = collect(Iterators.flatten(history))
        smin = Dict(j => (isempty(flat) ? NaN : minimum(get(h, j, 0.0) for h in flat)) for j in unbuilt)
        ssum = Dict(j => sum(get(h, j, 0.0) for h in flat; init = 0.0) for j in unbuilt)
        smax = Dict(j => maximum((get(h, j, 0.0) for h in flat); init = 0.0) for j in unbuilt)
        slast = Dict(j => (isempty(history) ? 0.0 :
                           sum(get(h, j, 0.0) for h in last(history); init = 0.0)) for j in unbuilt)
        # r <= 0 selects nothing, so the ranking is not consulted -- short-circuit before
        # the rule dispatch so the reference arms can pass `:none`.
        ranked = if r <= 0
            unbuilt
        elseif rule === :incidence || isempty(history)
            sort(unbuilt; by = j -> -incidence[j])
        elseif rule === :random
            shuffle!(MersenneTwister(rng + iteration), copy(unbuilt))
        elseif rule === :min_all
            sort(unbuilt; by = j -> (-smin[j], -incidence[j]))
        elseif rule === :last
            sort(unbuilt; by = j -> (-slast[j], -incidence[j]))
        elseif rule === :sum_all
            sort(unbuilt; by = j -> (-ssum[j], -incidence[j]))
        elseif rule === :max_all
            sort(unbuilt; by = j -> (-smax[j], -incidence[j]))
        else
            error("unknown rule $rule")
        end
        blocks = if r <= 0
            [Int[]]
        elseif mode === :rotate
            [ranked[1:min(r, length(ranked))]]
        elseif mode === :cover
            # Pack as many FULL-SIZE blocks as possible; the remainder is a runt.
            # MEASURED 2026-09-10, n=20 r=3 (|U|=10): this chunking gives [3,3,3,1] and a
            # gap of 531.5 after 300 iterations; "balancing" to [3,3,2,2] gives 1473.6 --
            # 2.8x WORSE, and slower, at the same 3600 cuts. Cut quality tracks how many
            # stations each cut is honest about, so three full-size blocks beat two, and
            # the runt costs only a row: the master takes the MAX over cuts, so a weak cut
            # dilutes nothing. The earlier reading -- that the singleton clogged the
            # master -- was wrong.
            [ranked[i:min(i + r - 1, length(ranked))] for i in 1:r:length(ranked)]
        else
            error("unknown mode $mode")
        end

        # Per-scenario slots, filled concurrently and assembled in scenario order afterwards,
        # so the cut sequence (and therefore the master) stays deterministic regardless of
        # which thread finishes first. No shared mutable state inside the threaded region and
        # no IO from it -- both would race.
        nS = n_scenarios(data)
        slot_cuts = [Tuple{Int, Float64, Dict{Int, Float64}}[] for _ in 1:nS]
        slot_cost = zeros(Float64, nS)
        slot_iters = zeros(Int, nS); slot_cols = zeros(Int, nS); slot_price = zeros(Float64, nS)
        slot_err = Vector{Any}(nothing, nS)
        Threads.@threads for s in 1:nS
          try
            sm = subs[s].model
            for j in eachindex(sm[:y]); JuMP.fix(sm[:y][j], incumbent[j]; force = true); end
            cost_s = NaN
            for T in blocks
                # `y` stays fixed at the TRUE incumbent, so `Q_s` is the real second-stage
                # cost; only the completion sees `T` as built, which is what lets pricing
                # search `S union T`. Blocks share this scenario's pool, so later blocks
                # start warm.
                pseudo = copy(incumbent)
                for j in T; pseudo[j] = 1.0; end
                res = StationSelection._solve_joint_routing_assignment_subproblem_by_cg!(
                    subs[s], solver.subproblem, plain ? incumbent : pseudo)
                res.converged || error("scenario $s block $T did not converge " *
                    "($(res.stop_reason)) -- a cut may only come from an exhausted round")
                slot_iters[s] += res.cg_iterations; slot_cols[s] += res.columns_added
                slot_price[s] += res.pricing_sec
                cost_s = JuMP.objective_value(sm)

                a, go, gd = extract_joint_routing_assignment_duals(sm)
                plain || StationSelection._benders_activated_complete_duals!(
                    a, go, gd, pseudo, data, mapping, s, walk_w)
                coef = Dict{Int, Float64}()
                for gg in (go, gd), (key, v) in gg
                    v == 0.0 && continue
                    key[1][1] == s || continue
                    coef[key[2]] = get(coef, key[2], 0.0) + v
                end
                const_s = sum(v for ((sc, _p), v) in a if sc == s; init = 0.0)
                implied = const_s - sum(get(coef, j, 0.0) * incumbent[j] for j in 1:N)
                isapprox(implied, cost_s; rtol = 1e-6, atol = 1e-6) || error(
                    "strong duality failed, scenario $s block $T: $implied vs $cost_s")
                push!(slot_cuts[s], (s, const_s, coef))
            end
            slot_cost[s] = cost_s
          catch e
            slot_err[s] = e
          end
        end
        for s in 1:nS
            isnothing(slot_err[s]) || throw(slot_err[s])
        end
        cuts = reduce(vcat, slot_cuts)
        total_cost = sum(slot_cost)
        cg_iters = sum(slot_iters); cols_added = sum(slot_cols); price_sec = sum(slot_price)
        gammas = Dict(s => c for (s, _k, c) in cuts)   # last block per scenario, for logging

        for j in unbuilt
            println(sc_io, join((N, arm, string(rule), r, iteration, j, true,
                isnan(get(smin, j, NaN)) ? "" : string(smin[j]),
                ssum[j], smax[j], slast[j], incidence[j],
                any(j in T for T in blocks),
                sum(get(gammas[s], j, 0.0) for s in 1:n_scenarios(data))), ","))
        end

        best_ub = min(best_ub, total_cost)
        gap = best_ub - lower
        for (bi, (s, const_s, coef)) in enumerate(cuts)
            for j in 1:N
                println(cut_io, join((N, arm, string(rule), r, iteration, s, bi, const_s, j,
                    j in built, get(coef, j, 0.0), lower, total_cost), ","))
            end
        end
        added = 0
        for (s, const_s, coef) in cuts
            grp = StationSelection.benders_cut_group(cut_mode, s)
            StationSelection.add_benders_optimality_cut!(m, theta[grp], const_s, y, coef)
            added += 1
        end
        push!(history, [c for (_s, _k, c) in cuts])
        n_cuts += added
        println(it_io, join((N, arm, string(rule), r, iteration, join(sort(collect(built)), "|"),
            join(unbuilt, "|"), join((join(T, "+") for T in blocks), "|"),
            lower, best_ub, gap, added,
            cg_iters, cols_added, round(price_sec; digits = 2),
            round(time() - t_arm; digits = 1)), ","))
        flush(it_io); flush(sc_io); flush(cut_io)
        @printf("%5d %14s %16s %11.2f %11.2f %9.2f %6d %7.1fs\n", iteration,
                join(sort(collect(built)), "|"),
                r <= 0 ? "-" : join((join(T, "+") for T in blocks), ","),
                lower, best_ub, gap, n_cuts, time() - t_arm)
        flush(stdout)

        if time() - t_arm > ARM_SEC
            @printf("  arm wall budget %.0fs exhausted at iteration %d\n", ARM_SEC, iteration)
            push!(summary, (N, arm, r, iteration, n_cuts, lower, best_ub,
                            best_ub - lower, time() - t_arm))
            return
        end
        if gap <= 1e-6 * max(1.0, abs(best_ub))
            println("  converged")
            push!(summary, (N, arm, r, iteration, n_cuts, lower, best_ub, gap, time() - t_arm))
            return
        end
    end
    println("  iteration cap reached WITHOUT converging")
    push!(summary, (N, arm, r, MAX_ITER, n_cuts, lower, best_ub, best_ub - lower, time() - t_arm))
end

"""One arm's failure must not take the grid with it. A cut may only come from an EXHAUSTED
pricing round, so an arm whose search cannot exhaust within budget raises -- which is the
correct behaviour and, past the measured pricing frontier (n<=20 all scenarios, n=25 to <=5,
n=30 only s=1), the expected one for the plain arm at larger `n`. Record and move on."""
function guarded(f, label)
    try
        f()
    catch e
        @printf("\n!! arm %s FAILED: %s\n", label, sprint(showerror, e)[1:min(end, 220)])
        push!(failures, label)
        flush(stdout)
    end
end

failures = String[]
selected = isnothing(TASK) ? collect(enumerate(cells)) : [(TASK, cells[TASK])]
@printf("running %d of %d cells | master threads %s\n\n", length(selected), length(cells),
        THREADS > 0 ? string(THREADS) : "auto")
for (i, (N, mode, rule, r, plain)) in selected
    label = "cell$(i) n$(N) $(plain ? "plain" : "$(mode) $(rule) r$(r)")"
    guarded(() -> run_arm(N, rule, r; plain = plain, mode = mode), label)
end

close(it_io); close(sc_io); close(cut_io)

println("\n", "="^132)
@printf("%4s %-22s %4s %7s %7s %13s %13s %11s %9s\n",
        "n", "arm", "r", "iters", "cuts", "lower bound", "upper bound", "gap", "wall")
for (n, arm, r, it, cuts, lb, ub, gap, wall) in summary
    @printf("%4d %-22s %4d %7d %7d %13.2f %13.2f %11.2f %8.1fs%s\n",
            n, arm, r, it, cuts, lb, ub, gap, wall, gap > 1e-6 ? "  NOT CONVERGED" : "")
end
isempty(failures) || @printf("\n%d arms failed: %s\n", length(failures), join(failures, ", "))
@printf("\nwrote %s\n", OUT)
foreach(f -> @printf("  %-22s %8.1f KB\n", f, filesize(joinpath(OUT, f)) / 1024),
        sort(readdir(OUT)))
