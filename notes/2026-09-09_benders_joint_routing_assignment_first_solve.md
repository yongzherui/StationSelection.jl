# Benders for the Joint routing+assignment formulation: first working solve (n=10)

2026-09-09. `AggregateODRouteJointRoutingAssignmentFormulation` + `BendersSolver` now
solves, verified exact against a monolithic reference. This note records the shape it took
and the two properties of its answer that have to travel with any number it produces.

## Shape: three formulation types, one block library

The decomposition is **not** a solver that builds two models by hand. It is a formulation
family (`src/opt/formulations/aggregate_od_route/joint_routing_assignment/`):

| Type | Carries |
| --- | --- |
| `…JointRoutingAssignmentFormulation` (monolith) | the whole model; `DirectMIPSolver`, `CGSolver` |
| `…JointRoutingAssignmentMasterFormulation` | `y` + cut placeholders `Θ`; `BendersSolver` |
| `…JointRoutingAssignmentBendersSubproblemFormulation` | one scenario's `x_walk`/`θ`, `y` fixed |

The split falls out of one observation: **`y` appears in neither the coverage rows nor the
objective.** It couples the model only through `pickup_link`/`dropoff_link`. So the first
stage is `y` plus the rows written only in `y` (station budget, endpoint feasibility) and
the second stage is literally everything else — and that second stage separates exactly by
scenario, since every column belongs to one scenario and every row is keyed `(s,p)`. The
two halves therefore cover the monolith's rows exactly once each, and each half's build is
a list of existing shared blocks with nothing re-derived.

Three enabling changes made that reuse real rather than aspirational:

1. **`scenarios` kwarg** on `add_joint_routing_assignment_{coverage,station_linking}_constraints!`,
   mirroring the one `add_walk_variables!` already carried (left over from the retired
   `BendersYX` subproblem, which was also per-scenario).
2. **`y` stays a variable in the subproblem**, pinned with `JuMP.fix(y[j], ŷ[j]; force=true)`.
   This is the choice that buys the most: the linking builder is reused *verbatim*, rows
   stay `θ - y[j] ≤ 0`, and there is no numeric-RHS second code path to drift from the
   first. Per-iteration update is `n` `fix` calls instead of a `set_normalized_rhs` sweep,
   and `reduced_cost(y[j])` becomes a free second route to the cut coefficients.
3. **`_stash_joint_routing_assignment_cost_parameters!`** — the cost weights and pool
   containers that `add_joint_routing_assignment_column!` reads off `m` rather than off a
   formulation argument. Fine with one build path; a live drift hazard with three.

Derived types are constructed **only** from a parent (`MasterFormulation(parent; cut_mode)`,
`BendersSubproblemFormulation(parent; max_stops)`), which copies the family's six shared
encoding fields. A master at one `detour_factor` against a subproblem at another produces
invalid cuts and a confidently wrong `OPTIMAL` with nothing raising anywhere, so that
combination is made unrepresentable rather than merely discouraged. `run_opt` still takes
one formulation — the monolith — because Benders is an algorithm for the same model.

## The cut, and why it is the simple one

With `y = ŷ`, `α ≥ 0` on the `≥` coverage rows and `γ ≥ 0` the negated duals of the `≤`
linking rows (`extract_joint_routing_assignment_duals`' own convention, reused):

    Θ_s ≥ Σ_p α[s,p] − Σ_j Γ[s,j] y_j,     Γ[s,j] = Σ_p (γ^O[(s,p),j] + γ^D[(s,p),j])

valid for every `y` because the dual feasible region does not mention `ŷ`. Two
simplifications are structural, not conveniences:

- **No feasibility cuts needed -- but the reason is the master, not `x_walk`.** I first
  wrote that `x_walk` covers every demand group so the subproblem is feasible at any `y`.
  That is FALSE: `x_walk` exists only for groups within `2 * max_walking_distance`, and a
  group beyond that needs a route column with both stations built. The real guarantee is the
  master's `add_aggregate_od_route_endpoint_feasibility_constraints!` rows -- the same rows
  the CG master carries -- so a `y` that fails them is never an incumbent. Those rows are a
  *necessary* condition only, so the guarantee is empirical: 0 of 86 endpoint-feasible sets
  have an infeasible subproblem (n=10 seed 42, s=1 and s=3), while 166 of the 252 sets
  outside the master's feasible set do. The brute-force audit found this by feeding
  unrestricted `y` and crashing, which is how the wrong claim surfaced.
- **No bound-dual term.** Neither `θ` nor `x_walk` has an upper bound in the relaxed build
  (`lower_bound = 0.0` only; the coverage rows are what hold them near 1). If anyone adds
  `θ ≤ 1`, this derivation needs a third term and silently under-cuts without it.

Every solve asserts `Σα − Σ Γ_j ŷ_j == objective` before its cut is built. One line, and it
is the thing that catches a flipped dual sign, a dropped linking family, or a subproblem
whose `mapping` disagrees with the master's — each of which otherwise converges to a wrong
number in silence.

Deliberately **not** used: the `:zero_completion` / `:restricted_mw_fixed_pi` cut
derivations on the retired pre-split markers. Those are what produced the invalid-cut bug
on route-covering LP degeneracy (`project_yz_completion_lp_invalid_cut`). Solving the true
subproblem avoids the class entirely.

## Two properties of the answer

**It is the MIXED optimum, not the direct-MIP one.** The cut IS the subproblem's LP dual,
so the subproblem must be an LP, so a converged run is optimal for `y` binary with
`θ`/`x_walk` continuous. Recorded as `metadata["benders_second_stage_relaxed"] = true`.
Getting the all-binary optimum needs integer L-shaped / combinatorial cuts — a different
project.

**Scope narrows with the enumeration cap.** The only oracle today is
`:direct_enumeration`; its pool is exponential in `max_stops` on two axes, so
`BendersSubproblemConfig.max_stops` defaults to 4 and narrows the formulation's own value
when larger. `metadata["benders_optimality_scope"]` then reads `"max_stops_restricted"`,
in the same spirit as `cg_optimality_scope`. Exercised by the `restricted_scope` arm below,
which relies on the master carrying no `max_stops` dependence to make the narrowing
testable without enumerating the wider universe.

## Measured

### Seed sweep (Zhuzhou n=10, p=8, 1 scenario, k=5, max_stops=4)

| seed | objective | iters | cuts | cols | loop wall | enum wall |
| --- | --- | --- | --- | --- | --- | --- |
| 42 | 12249.400939 | 4 | 3 | 16320 | 2.34 s | 3.54 s |
| 43 | 8758.928834 | 3 | 2 | 7500 | 2.19 s | 2.88 s |
| 45 | 8398.668920 | 2 | 1 | 1823 | 1.30 s | 2.02 s |

### Arm sweep (seed 42, `benders_joint_n10.jl`, 29/29 checks)

| arm | objective | iters | cuts | cols | scope |
| --- | --- | --- | --- | --- | --- |
| multicut_s1 | 12249.400939 | 4 | 3 | 16320 | full_route_universe |
| singlecut_s1 | 12249.400939 | 4 | 3 | 16320 | full_route_universe |
| restricted_scope | 12249.400939 | 4 | 3 | 16320 | **max_stops_restricted** |
| multicut_s3 | 25771.187908 | 4 | 5 | 21635 | full_route_universe |
| singlecut_s3 | 25771.187908 | 4 | 3 | 21635 | full_route_universe |

Two things the arm sweep established that the seed sweep could not:

**The cut dedup is load-bearing, not defensive.** `multicut_s3` added **5** cuts over 4
iterations with 3 scenarios; an un-deduplicated `MultiCut` would have added up to 12. So
the degenerate-dual repeat that `add_benders_cut!`'s return count exists to detect is a
real occurrence at the smallest interesting size, not a theoretical concern. Without the
count the loop still terminates here (the gap closes), but a case where it does not is
clearly reachable.

**A prediction that was wrong: `SingleCut` did NOT need more iterations.** The expectation
was that aggregating three scenarios' cut data into one row would cost iterations relative
to three separate rows. At s=3 both modes took 4 iterations to the same optimum
(`SingleCut` adding 3 cuts to `MultiCut`'s 5). n=10 with 1-5 total cuts puts neither mode
under any pressure, so this cell cannot separate them; the comparison needs a harder
instance before it means anything either way.

The `restricted_scope` row is the narrowing path: formulation `max_stops=6` with the
subproblem capped at 4 returns `multicut_s1`'s objective *exactly*, because the master
carries no `max_stops` dependence at all (its rows are the station budget and endpoint
feasibility, and the map keys off `max_walking_distance`). That identity is what makes the
path testable without enumerating `max_stops=6`, which is not tractable.

### Scaling at s=3: n=10 / 15 / 20, MultiCut, and the cut-count answer

`benders_scaling_s3.jl`, generous caps (`max_routes=20e6`, `max_iterations=2000`), 18/18
checks, 6m01s total including precompile:

| n | k | C(n,k) | pool | enum | iters | cuts | objective |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 10 | 5 | 252 | 21,635 | 3.6 s | 4 | 5 | 25771.187908 |
| 15 | 8 | 6,435 | 61,324 | 3.4 s | 4 | 8 | 28384.120680 |
| 20 | 10 | 184,756 | 237,353 | 12.2 s | 7 | 12 | 28735.190221 |

**CORRECTION: there was no oracle ceiling at n=15.** This note previously recorded n=15 x
s=3 as beyond `:direct_enumeration`'s reach because it threw
`exceeded max_routes=200000`. That reading was wrong on two counts. `max_routes` is checked
against the **pre-deduplication accumulator** (`enumeration.jl:267`; dedup runs only in the
final `return`), so it bounds *generated* columns, not the pool -- n=15 x s=3 generates over
200,000 raw entries but deduplicates to 61,324. And it is not a tractability wall at all: with
a generous cap, n=20 x s=3 enumerates 237,353 columns in **12.2 s**. Enumeration is simply
not the bottleneck at `max_stops=4` up to n=20; the earlier "ceiling" was an artifact of a
cap I had set too low and then over-interpreted. The real argument for a
`:column_generation` oracle has to be made at larger `max_stops` or larger `n`, not here.

**The cut count is near-flat in the size of the first-stage space.** 733x more station sets
from n=10 to n=20; 2.4x more cuts (5 -> 12). That is the answer to "why so few cuts": each
cut prices out EVERY station, not one vertex. `Gamma_j` is the shadow price of the linking
row `sum(theta) <= y_j`, and that row is binding precisely when `y_j = 0` -- so a single cut
says "and here is what each station you did NOT build would have been worth", which
constrains the whole space at once. A Benders cut that only pinned down its own incumbent
would have to scale with C(n,k).

**The trace shows the master exploring, not being steered** -- which is the direct
counter-evidence to the over-strong-cut worry, and needed the new
`BendersSolver.iteration_callback` to see at all. At n=20:

    iter |          LB |          UB | this-iter Q | cuts
       1 |        0.00 |    31820.92 |    31820.92 | 3
       2 |    27453.52 |    31820.92 |    32091.70 | 3   <-- proposed y is WORSE than incumbent
       3 |    27453.52 |    29098.17 |    29098.17 | 3
       4 |    28401.81 |    28735.19 |    28735.19 | 1
       5 |    28693.94 |    28735.19 |    29025.52 | 1   <-- again worse
       6 |    28693.94 |    28735.19 |    28735.19 | 1
       7 |    28735.19 |    28735.19 |    28735.19 | 0

At iterations 2 and 5 the master proposed station sets that evaluated *worse* than the best
already known. Over-strong cuts would drive the master monotonically at one point; instead
the lower bound rises monotonically, the upper bound descends, and the master rejects its own
proposals along the way. Textbook Benders behaviour. Note also that the "4 iterations
always" pattern from the earlier cells simply broke at n=20 (7 iterations) -- it was
smallness, not a suspicious constant.

**Still 0% LP-IP gap at every size.** `mixed_mono == direct_mip` to the last digit at n=10,
15 and 20. So `benders <= direct_mip` remains non-discriminating and the mixed-vs-integral
question is still open at n=20 -- it is not an artifact of tiny instances in the way I
assumed, which makes it more interesting rather than less.

**CG agrees at all three sizes** (independent column source, converged at
`full_route_universe`): `cg_lp == cg_ip == benders == mixed_mono == direct_mip` at each n,
with CG taking 3 / 9 / 6 iterations against pools of 21k / 61k / 237k enumerated columns.

### Independent verification: brute force + CGSolver cross-check

The monolithic `mixed_mono` comparison shares the model DEFINITION with Benders (same map,
same enumerated pool, same cost weights), so it validates the ALGORITHM and not the model.
Two further checks close that gap.

**Brute force over the master's feasible set** (`benders_brute_force_certificate.jl`).
At n=10/k=5 only 86 of the 252 station sets satisfy the master's endpoint-feasibility rows.
Evaluating the second stage exactly at each gives the true value function with no master,
no cuts and no monolithic MIP:

| s | brute-force min | Benders | ties at optimum | worst set | spread |
| --- | --- | --- | --- | --- | --- |
| 1 | 12249.400939 | 12249.400939 | 21 of 86 | 16538.302954 (+35.0%) | 4288.90 |
| 3 | 25771.187908 | 25771.187908 | 21 of 86 | 34862.882745 (+35.3%) | 9091.69 |

**Pointwise cut audit.** Every cut checked against the true `Q_s(y)` at every one of the 86
sets -- a valid cut must underestimate `Q_s` everywhere, not just at the `yhat` it came
from. Worst violation across all cuts x all sets: `+6.4e-07`, i.e. float noise. Every cut
is also *tight somewhere* (largest tightest-slack `6.4e-07`), so none is slack everywhere
and merely decorative. This is the check that would actually catch a too-strong cut, and it
matters precisely because converging in 3-5 cuts is the signature an invalid cut would
produce.

**CGSolver cross-check** (`benders_vs_cg_crosscheck.jl`) -- the only check with an
independent COLUMN SOURCE, since CG prices by label-setting and never enumerates. At n=10
s=1 seed 42, with CG converged and `cg_optimality_scope == "full_route_universe"`:

    cg_lp = benders = mixed_mono = cg_ip = direct_mip = 12249.400939

All five collapse to one number. `cg_lp` is a lower bound on the true mixed optimum
*whatever the enumerated pool contains* (pricing searched the full universe and `y` is
relaxed on top), so `cg_lp == direct_mip` says the enumerated pool is complete for the
optimum AND the `y`-relaxation is tight here. `cg_ip == direct_mip` is the pool-agreement
check specifically: two all-binary optima over two independently built pools. A route the
enumerator missed would show up there and nowhere else. CG reached it in 3 iterations
against a 16,320-column enumeration.

Note the precondition is not optional: on a budget-stopped CG run `cg_lp` bounds nothing and
a mismatch would prove nothing while looking exactly like a bug in one of the two
implementations. The script checks `cg_converged` and the scope first, before reporting any
comparison as meaningful.

### Infeasibility reporting (k=1)

A proven-infeasible instance must come back as an ANSWER, not a thrown error. Both routes
to that verified at `k=1`, which is infeasible here because most Zhuzhou OD pairs exceed
`2 * max_walking_distance` and so carry no direct-walk fallback, giving them
endpoint-feasibility rows that one station cannot satisfy:

| arm | status | how it knew |
| --- | --- | --- |
| infeasible_gate (via `run_opt`) | INFEASIBLE | `infeasibility_reason` -- "no size-1 station selection can reach every demand group that lacks a direct-walk fallback" |
| infeasible_master (bypassing the gate) | INFEASIBLE | `benders_stop_reason = "master_INFEASIBLE"`, `LB = -Inf` |

Worth being precise about the second one: **it is unreachable through `run_opt`**, because
the gate solves exactly the master's own row set and therefore always refutes first. So the
infeasible-master branch is defensive, not on the normal path -- it exists because
benchmark scripts call `build_model`/`optimize_model` directly, bypassing the gate, which
is how this arm reaches it. Keeping it is still right (an infeasible master IS a proof
about the problem, since a Benders cut can only raise `Theta` and never removes a feasible
`y`), but it should not be described as the mechanism by which infeasibility is normally
reported.

### Suite

93,086/93,086 tests pass with the Benders work in (run 22409466, 2m23s). Credit to a
parallel session for pointing out what that does and does not cover: the full suite on a
clean tree reaches the CG path, the relaxed-cluster pricers, the clustering formulations
and the include graph -- and NOT n=40/50 scale, long-running certification, or two-tier
escalation under real budgets. Those are only exercised by the Study 9/10 arrays.

LB == UB exactly on every cell, `gap = 0.000e+00`, and `benders == mixed_mono` to
`diff 0.000e+00`. Master time is negligible (0.02–0.04 s total); the loop is
subproblem-bound, and the whole run is dominated by up-front enumeration — which is the
expected and the damning fact about this oracle (see below).

**The LP-IP gap is 0% on every cell measured**, s=1 and s=3 alike, so `benders ≤ direct_mip` passes trivially and
cannot distinguish the mixed optimum from the integral one at this size. The monolithic
*mixed* comparison is the check that actually establishes exactness. A cell with a real
gap (the hub-route effect shows 21.6% at n=40) would be the stronger test and has not been
run.

Iteration counts of 2–4 against C(10,5) = 252 candidate station sets say the cuts are
strong here, but n=10 is not evidence about scaling -- note s=3 also took 4 iterations,
i.e. tripling the second stage did not move the iteration count at all, which is far more
likely to mean the instance is easy than that the method is insensitive to `s`.

A cross-implementation check worth doing when Study 11's data lands: a `y`-fixed-binary /
`theta`-and-`x_walk`-continuous snapshot over a CG pool is the same mixed model this
solves, so the two objectives must agree -- but only on seeds where CG certified
(`cg_stop_reason == "converged_by_certification"`, full-universe scope), since on a
budget-stopped seed the CG number is an upper bound and a mismatch would prove nothing.
Parameters have to match too: this oracle caps `max_stops` at 4 while the benchmarks run
at 10, so a like-for-like cell needs small `n` and `max_stops = 4`.

Repro: `sbatch benchmarks/diagnostics/run_benders_n10.sh` (env `BJ_SEED`, `BJ_N`, `BJ_P`,
`BJ_S`, `BJ_MAX_STOPS`); the script runs both monolithic references and fails loudly.

### The independent DirectMIPSolver arm, and what actually explains the cut count

`benders_direct_arm_and_flatness.jl`. Two gaps closed here.

**Gap 1: none of the earlier references were independent of each other.** `mixed_mono`,
`direct_mip` and `CGSolver`'s master are all built by the SAME function,
`_build_joint_routing_assignment_model`, so a bug in it would make all three agree and all
three be wrong. (Benders was the odd one out all along -- its master and subproblem are
separate constructions.) This runs the SHIPPED `DirectMIPSolver` path
(`optimize/aggregate_od_route/direct/build_joint_routing_assignment.jl`) instead:

| n | Benders | DirectMIPSolver | brute-force min over all `y` | pool | Direct wall |
| --- | --- | --- | --- | --- | --- |
| 10 | 25771.187908 | 25771.187908 | 25771.187908 (86 sets) | 21,635 | 3.4 s |
| 15 | 28384.120680 | 28384.120680 | 28384.120680 (4,437 sets) | 61,324 | 13.0 s |
| 20 | 28735.190221 | 28735.190221 | -- (C(20,10) too large) | 237,353 | 127.1 s |

8/8 checks. Brute force now also covers n=15 (4,437 master-feasible sets, exhaustively
evaluated), so the optimum is confirmed by exhaustion at two sizes, not one.

Still shared by EVERY arm and therefore tested by none: the `AggregateODRouteMap`,
`joint_routing_assignment_column_cost`, and the objective assembly. Those are the model
definition; falsifying them needs an oracle outside this formulation family.

**Gap 2: my explanation of the low cut count was wrong twice, and the corrected one is
narrower.**

First wrong claim: "each cut prices every station, since `Gamma_j` binds at `y_j = 0`, so one
cut constrains the whole space." The audit refutes it -- and so does the scaling:

| n | nonzero coefs per cut | round-1 LB as % of optimum | flatness `Q_max/Q_min` |
| --- | --- | --- | --- |
| 10 | 1-2 of 10 | 99.71% | 1.3528 |
| 15 | 2-7 of 15 | 66.48% | 1.4415 |
| 20 | 2-10 of 20 | 79.74% | -- |

Cuts are sparse at n=10 but reach 10 of 20 nonzeros at n=20, so density GROWS with `n`.

Second wrong claim, from the n=10 trace: "one round of cuts gets 99.7% of the way". That is
n=10-specific. At n=15 one round reaches only 66.5%.

**What survives is the flatness.** `Q_max/Q_min` is 1.35 at n=10 and 1.44 at n=15: even the
WORST master-feasible station set is only 35-44% more expensive than the best, and `Q` never
approaches zero. A Benders cut is exact at its anchor, so anchoring anywhere immediately
bounds the optimum to within a few tens of percent, and a handful of cuts closes the rest.
The low cut count is therefore a property of THIS INSTANCE FAMILY -- a high-floor,
narrow-range second-stage cost -- not of cut strength and not evidence the decomposition is
good. **These cells barely stress the method.**

That is consistent with the historical nearest-open runs needing 80-770 cuts at p=16/32: a
value function with a lower floor and wider spread gives cuts much less to work with. And it
predicts that a family where `Q` varies by orders of magnitude would need far more cuts here
too. Until such a cell is run, the cut counts above should be read as "this instance is easy
in the dimension Benders cares about", not as a property of the solver.

**Cut counts are not reproducible run to run; objectives are.** n=20/s=3 reported 12 cuts in
one run and 9 in another at identical configuration. The master is a MIP with many optimal
`y` (21 of 86 sets tie at the optimum at n=10), so which optimum Gurobi returns varies with
threading, and the cut sequence follows. Compare objectives across runs, never cut counts.

### Cut validity: what it means, how it is measured, and the results

A cut `Theta_g >= c - sum_j Gamma_j y_j` is VALID iff its right-hand side never exceeds the
true second-stage cost anywhere the master could go:

    for all master-feasible y:   c - sum_j Gamma_j y_j  <=  Q_g(y)

The failure it guards against is specific: if a cut's RHS exceeded `Q_g` at the true optimum,
the master would price that optimum above its real cost and could REJECT it -- converging
with `LB == UB` on a wrong answer, with nothing raising. A cut that only ever understates cost
can slow convergence but cannot delete the optimum.

The audit therefore reports, over every (cut x audited `y`) pair,

    violation = (cut RHS at y) - Q_g(y)          worst = max over all pairs

with `Q_g(y)` obtained by actually FIXING `y` and solving that scenario's LP to optimality,
not from any formula. **`worst ~ 0` from above is the expected result, not a suspicious one**:
each cut is derived tight at its own anchor by strong duality, and the anchors are in the
audit domain. A strongly NEGATIVE worst would be the concerning outcome -- it would mean no
cut is tight anywhere, i.e. the derivation is too weak.

| size | audit domain | worst violation |
| --- | --- | --- |
| n=10 s=1 | exhaustive, 86 of 252 sets | (rounded-signature audit; see below) |
| n=10 s=3 | exhaustive, 86 of 252 sets | (rounded-signature audit; see below) |
| n=15 s=3 | **exhaustive, 4,437 of 6,435 sets** | **+1.819e-12** |
| n=20 s=3 | 400 targeted+random per seed, 9 seeds | **+5.457e-12** worst of any seed |

n=20, ten seeds (42-51), one SLURM array task each -- 36 checks, 0 failed:

| seed | 42 | 43 | 44 | 45 | 46 | 47 | 48 | 49 | 50 | 51 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| iters | 7 | 3 | 4 | 4 | * | 4 | 5 | 4 | 4 | 3 |
| cuts | 12 | 6 | 8 | 8 | * | 9 | 11 | 9 | 8 | 6 |

`*` seed 46's first task died in Julia startup (see the depot note below) and was rerun.
Cuts 6-12 and iterations 3-7 across seeds, consistent with the flatness explanation.

#### A measurement bug that looked exactly like the thing being tested

The first n=15 exhaustive audit reported `+1.353e-06` against a `1e-6` tolerance -- a FAIL.
It was the audit, not the cuts. The script rebuilt each cut from
`m[:benders_cut_signatures]`, which is **rounded to 6 decimals** because it is a dedup key;
rounding the constant up and each coefficient down strengthens the reconstruction by up to
~5e-7 per term, which at 7 nonzero coefficients is more than enough to manufacture the
violation. Relative size was `4.8e-11` of an objective of 28,384.

The fix was to the measurement, not the tolerance -- loosening a tolerance to pass a failing
validity check would have been precisely the wrong move. `add_benders_optimality_cut!`'s
`ConstraintRef`s were being discarded, so the master now keeps `m[:benders_cuts]` as
`(group, ConstraintRef)` and the audit reads exact `normalized_rhs`/`normalized_coefficient`
values, asserting the `Theta` coefficient is 1 so a future change to the row shape cannot be
misread silently. Same audit, exact rows: `1.353e-06` -> `1.819e-12`.

The n=10 figures in the table above (`6.4e-07`) predate the fix and are inflated the same
way; they passed regardless, but they are not comparable to the n=15/n=20 numbers.

#### What the audit does NOT establish

- n=20 is sampled (400 of 184,756 sets), so evidence, not proof. n=10 and n=15 are complete.
- It audits only the cuts a given run actually generated. The STATIC argument in
  `benders/subproblem.jl` is what covers all possible cuts; the audit checks the
  implementation on the ones it produced.
- It cannot detect a wrong MODEL. `Q_g` is computed from the same subproblem the cut came
  from, so if that subproblem is wrong, cut and audit agree and are both wrong. The shared
  pieces across every arm remain `AggregateODRouteMap`,
  `joint_routing_assignment_column_cost` and the objective assembly.

#### Infrastructure note: do not share a Julia depot across a concurrent array

Task 5 of the first array died with `Bus error (signal 7)` in `gc_mark_outrefs` 47 s in
(MaxRSS 2 GB of 64 GB -- not OOM), before reaching any Benders code. Four of the ten tasks
entered precompilation simultaneously against the SHARED depot and three rewrote
`StationSelection.ji`; that invalidates the mmap of any sibling holding the old file, and the
next GC touch faults. Two corrections to what this note's author believed at the time:
Julia's pidfile lock serialises WRITERS and does nothing for a reader whose mapping is
replaced; and the caches were not "already warm" -- `src/` had been edited immediately before
submitting, making every task rebuild. The per-seed `try/catch` also cannot contain this:
SIGBUS kills the process rather than raising. The array script now defaults to a per-task
depot, and documents the safe fast path (warm the shared depot with ONE job first, then
submit).

### Comparison with the 2026-07/08 Benders work

`notes/2026-08-04_zhuzhou_benders_cut_ms5_scaling_results.md` measured the previous
(pre-split, now removed) Benders implementations on the same instance family. Its best
configuration was `BendersYZ` with restricted-MW cuts. Median iterations / optimality cuts /
solver wall, against this implementation at s=3:

| n, q | OLD BendersYZ-MW iters | OLD cuts | OLD time | NEW iters | NEW cuts | NEW wall |
| --- | --- | --- | --- | --- | --- | --- |
| 10, 3 | 28.0 | 81.0 | 29 s | **4** | **5** | 17.4 s |
| 15, 3 | 175.5 | 521.5 | 823 s | **4** | **8** | 9.1 s |
| 20, 3 | -- (3/6 succeeded) | 766.0 | 3,390 s | **7** | **12** | 69.9 s |
| 25, 3 | 0/6 succeeded | -- | -- | not run | | |

Old-side context: 45 of 348 tasks hit the `max_iterations=500` cap, 88 timed out, 44 OOMed
at 16 GB. No three-scenario case completed beyond n=25 by any method. `BendersY` (master
over `y` only, the same first-stage shape as this implementation) deteriorated sooner still:
at n=15 half its single-scenario runs hit 500 iterations.

So on nominally comparable `(n, q)` the new implementation uses **44x fewer iterations and
65x fewer cuts at n=15**, and 64x fewer cuts at n=20 where the old one only succeeded half
the time.

### Why -- and it is NOT that this implementation is better

Five differences, and the last one is the whole story:

1. **`p` differs**: p=8 here versus p=16/32 there, so 2-4x fewer OD pairs and coverage rows.
2. **`max_stops`**: 4 versus 5.
3. **Different MODEL, not just a different algorithm.** Those runs used
   `NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)` -- a procedural nearest-open
   assignment resolver. This formulation has free assignment baked into the columns. **The
   objectives are not comparable at all**; only rough algorithmic effort is, and even that
   is confounded by 1 and 2.
4. **Different first stage**: `BendersYZ`'s master carries `y` AND the nearest-open endpoint
   selectors `z`. This master carries `y` only.
5. **The subproblem there was INEXACT; here it is exact.** Those runs solved the
   route-covering subproblem by *inner column generation* with repricing, and derived cuts
   through completion tricks (`zero_completion`, `restricted_mw_fixed_pi`,
   `standard + reprice`). An inexact subproblem gives weaker cuts, and completion-derived
   cuts were separately found to be outright invalid in some cases
   (`2026-07-27` / `project_yz_completion_lp_invalid_cut`). This implementation solves the
   subproblem to optimality over a *complete* enumerated pool and takes the plain LP dual.

**The low iteration count is bought with up-front enumeration.** The old implementation paid
for an inexact subproblem in iterations; this one pays for an exact subproblem in
enumeration. That is a trade, not a win -- and it yields a falsifiable prediction: **when the
`:column_generation` oracle replaces `:direct_enumeration`, iteration and cut counts should
rise substantially toward the historical numbers**, because the subproblem becomes inexact in
exactly the way theirs was. If they *don't* rise, something else is going on and this
comparison is the reason to look.

### Three historical bugs this implementation avoids by construction

Not by cleverness -- the notes were the specification:

- `2026-07-21_benders_final_result_vs_best_result_bug.md`: the terminal return used the wrong
  incumbent. Here `BendersLoopState` tracks `best_incumbent`/`best_iteration` explicitly and
  `_benders_package_result` reports the best, never the last (the n=10 s=1 runs stop at
  iteration 4 and report iteration 2's solution, so this path is live, not theoretical).
- `2026-07-23_benders_reports_optimal_with_unclosed_outer_gap.md`: `OPTIMAL` reported with a
  large open outer gap. Here the status comes from `st.converged`, never from the master's
  `MOI` code, and the lower bound is read from `objective_bound` rather than
  `objective_value` so a non-zero `MIPGap` cannot fake convergence.
- `2026-07-27` invalid completion-LP cuts: avoided by not using completion cuts at all.

### One historical result that corrects this note's own feasibility claim

`2026-07-22_endpoint_coverage_feasibility_guarantee.md` made subproblem feasibility
*provable* for the old model via three changes together: endpoint coverage in the master,
`allow_same_station = true` so a `(j,j)` pair always resolves the "both endpoints' nearest
stations collide" case, and a walk-only companion rule. **The current package removed
same-station pairs entirely** (`compute_valid_jk_pairs` no longer emits `j == k`), so that
proof does not transfer verbatim -- which is why the guarantee here was only ever measured.

But the historical note also supplies the missing argument: by the triangle inequality, any
`j` within `max_walking_distance` of both `o` and `d` implies
`dist(o,d) <= 2 * max_walking_distance`, which is exactly the condition for `WALK_ONLY_PAIR`
to exist. So same-station pairs and walk-only cover the identical set of groups, and dropping
`(j,j)` loses nothing. That splits every group cleanly:

- `dist(o,d) <= 2 * max_walking_distance`: `x_walk` exists, always coverable.
- otherwise: no same-station `j` can exist either, so a genuine `(j,k)`, `j != k`, is
  required -- and endpoint coverage forces a station near `o` and one near `d` to be built,
  with the 2-stop route `j -> k` serving the pair whenever `routing_cost(j,k)` is finite
  (its ride limit is `detour_factor * routing_cost(j,k) >= routing_cost(j,k)` for
  `detour_factor >= 1`, so the direct hop always fits) and `max_wait_time` is met.

That reduces the residual risk to two concrete conditions -- an infinite routing cost between
the specific built pair, or a wait-time violation -- rather than leaving it as "necessary but
not sufficient, measured 0/86". Worth turning into a real build-time assertion.

Corroboration worth noting: that 2026-08-04 scaling run also reports "endpoint coverage is
imposed eagerly in the master; all recorded feasibility-cut counts are zero" -- the same
mechanism and the same observed outcome as here, three months earlier.

## What this is not

`:direct_enumeration` enumerating the whole route universe up front is exactly the cost
Benders exists to avoid, and it is the same objection that retired the pre-split
`AggregateODRouteBendersYXFormulation`. It is the right *first* oracle: with the pool
complete and fixed the subproblem is an honest LP with exact duals, so the loop, the cut
algebra and the hook wiring are verified before an inexact subproblem raises the question
of whether its duals still give valid cuts. `:column_generation` is the intended next
value and is currently rejected rather than ignored. `RouteCoveringProblem`
(`opt/problems/route_covering.jl`) remains the shape that oracle should reuse.
