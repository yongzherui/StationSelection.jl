# A Cummings-style locally Pareto-optimal Benders cut for a column-generated route subproblem

2026-09-10. Implements `BendersSubproblemConfig(oracle = :column_generation_activated_lpo)`.
Code: `src/opt/optimize/aggregate_od_route/benders/completion_lpo.jl`.
Experiment: `benchmarks/diagnostics/benders_activated_completion_audit.jl`.

## The hypothesis

At a fixed first-stage incumbent `ŷ` the recourse value `z* = Q(ŷ)` normally has MANY
dual-optimal representations. Column generation returns an arbitrary one, and an arbitrary
one can put large multipliers on stations `ŷ` does not build. Since the cut is

```
Θ ≥ Σ_p α_p − Σ_j g_j y_j,          g_j = Σ_p γᴼ_pj + Σ_p γᴰ_pj,
```

and a one-station swap `y' = ŷ − e_a + e_b` moves the prediction by exactly `g_a − g_b`,
large `g_b` on an unbuilt `b` is precisely what makes the cut say nothing about the master's
immediate neighbourhood. Re-optimising over the optimal dual FACE against a core point should
pick a representation that says more.

## Setting and notation

For one scenario, at incumbent `ŷ`: `J⁺ = {j : ŷ_j = 1}`, `J⁰ = {j : ŷ_j = 0}`. The second
stage is

```
min  Σ_p c_walk[p] x_walk[p] + Σ_c f_c θ_c
s.t. (α_p ≥ 0)     x_walk[p] + Σ_{c covers p} θ_c ≥ 1
     (γᴼ_pj ≥ 0)   Σ_{c: p picked up at j} θ_c ≤ ŷ_j
     (γᴰ_pk ≥ 0)   Σ_{c: p dropped at k} θ_c ≤ ŷ_k
```

with `f_c = c_route(V^c) + w Σ_{(p,j,k) ∈ A^c} demand_p · walk(o_p, d_p, (j,k))` and
`c_route(V) = β (τ_V + repositioning_time)`. Writing

```
ρ_pjk = α_p − γᴼ_pj − γᴰ_pk − w · demand_p · walk(o_p, d_p, (j,k))
```

the θ columns' dual rows are exactly `Σ_{(p,j,k) ∈ A^r} ρ_pjk ≤ c_route(V^r)`, and the x_walk
columns' are `α_p ≤ w · demand_p · walk(o_p, d_p, WALK_ONLY)` for the groups that have one.

`R` is the full route family; `R⁺` the family reachable with assignments (and therefore route
nodes) inside `J⁺`; `C⁺ ⊆ R⁺` the finite pool the activated CG solve leaves behind. **`C⁺` is
not `R⁺`** and confusing them is the central hazard of this whole construction.

## Phase 1 — the activated solve

Unchanged: `:column_generation_activated` prices built-only and installs the closed-form
completion `γᴼ_pj := max(γᴼ_pj, max(0, α_p − w · min_k walk))` BEFORE pricing, which is what
restricts the search (the pricer's own `ρ > 0` filter drops every candidate through an unbuilt
station). Its termination certificate is `min_{r ∈ R⁺} rc_r ≥ −ε_price`, and the cut is
licensed by that exhaustion, not by the pool being complete. We keep `z*`, the pool `C⁺`, the
dual, and the diagnostics.

## Phase 2 — the baseline completion (Method A)

`α̂` and the built-station `γ̂` are held fixed; only the `J⁰` multipliers are chosen, to do the
least damage at a core point `y^c`:

```
min_{g⁰ ≥ 0}  Σ_{j ∈ J⁰} y^c_j g_j       s.t.  ρ_pjk ≤ 0 for every valid triple touching J⁰.
```

This is a strict improvement on the closed form, which satisfies the same rows by loading
each one entirely onto one endpoint at its cheapest partner.

### The assignment-removal property — VERIFIED, with one precondition

The `ρ ≤ 0` rule is only globally valid if the route family has this property:

> For every feasible column `r = (V, A)` and any `A_in ⊆ A`, there is a feasible column
> `r'' = (V'', A_in)` in the searched family with `c_route(V'') ≤ c_route(V)`.

It holds here. Take `V''` to be `V` with every station carrying no retained assignment
deleted.

- **Cost.** `τ_{V''} ≤ τ_V` because the travel matrix is required to be metric package-wide —
  the pricer's age pruning already assumes it and asserts on violation. `c_route` is
  increasing in `τ`.
- **Feasibility.** Both route-feasibility conditions bound ELAPSED durations from above: the
  pickup window is `label.time ≤ max_wait_time`, the ride limit is
  `origin_age + travel ≤ detour_factor · routing_cost(j,k)`. Deleting a stop only decreases
  each left-hand side. Both right-hand sides are unchanged — `max_wait_time` is a constant,
  and `routing_cost(j,k)` is computed from the ASSIGNMENT's stations before any route exists
  (`pricing_round.jl`), not from route positions. Nothing measures wait against a fixed
  request clock, so no retained passenger waits longer. `A → U → A → B` with the pickup at the
  second `A` is fine too: age is measured from the last visit to that station.
- **Membership in `R⁺`.** When `A_in` is the built-only part of `A`, every node of `V''`
  carries a built assignment, and the activated pricer proposes nodes only from surviving
  candidates' origins and their opportunities' destinations (`exact/seed.jl`,
  `exact/extend.jl`) — which are exactly the built ones once `ρ ≤ 0` removes the rest.

**Precondition, stated as a live one:** this is exactly as sound as the metric-travel-matrix
requirement. On a non-metric matrix the shortcut can cost more and the argument fails — the
package asserts on violation elsewhere, but nothing in this file re-checks it.

### Why Method A needs no pricing

Split `A^r = A_in ∪ A_out` by whether both endpoints are built.

```
Σ_{A_out} ρ ≤ 0                  (the completion rows, one triple at a time)
Σ_{A_in}  ρ ≤ c_route(V'')       (the activated certificate — and Method A did not move
                                  a single multiplier appearing in it)
c_route(V'') ≤ c_route(V)        (assignment-removal)
⇒ Σ_{A^r} ρ ≤ c_route(V^r)       for every r ∈ R, not only R⁺.
```

With `α ≤ c_walk` and `α, γ ≥ 0`, the point is feasible for the full dual, so the cut is
globally valid; and since the completion touches only coordinates with `ŷ_j = 0`, it is still
tight at `ŷ`. **This is the argument that makes the `:baseline` arm pricing-free, and it is
the one the `:pareto` arm loses** — Phase 5 exists precisely because `:pareto` moves the
multipliers in `Σ_{A_in} ρ`.

### Which walking cost the completion rows use

The dual row's walk term carries `demand_p`; the pricer's candidate reward does not
(`joint_routing_assignment_pricing_candidates`). The rows are written with the DEMAND-FREE
term,

```
α_p − γᴼ_pj − γᴰ_pk ≤ w · walk(o, d, (j,k)),
```

which is the stronger requirement when `demand_p > 1`. Deliberate, and it serves both jobs at
once: it implies the dual condition validity needs, AND it matches the pricer's filter, which
is what keeps every separation round below ACTIVATED. The weaker demand-weighted row would
still be sound but would let a `demand_p > 1` triple through an unbuilt station survive
`ρ > 0` and re-open the full-universe search.

## Phase 3 — the core point

`y^c_j = K/|J|` satisfies the budget row `Σ_j y_j = K` strictly inside `0 ≤ y_j ≤ 1`, but the
master carries a second family — `add_aggregate_od_route_endpoint_feasibility_constraints!`'s
`Σ_{j near pt} y_j ≥ 1` for every location a demand group cannot walk from — and the uniform
point sits exactly ON one of those whenever a required location has fewer than `|J|/K`
candidate stations. So it is NOT in general a relative-interior point, and the default is an
auxiliary LP instead (`lpo_core_point = :max_min_slack`):

```
max t  s.t.  Σ_j y_j = K;  Σ_{j near pt} y_j ≥ 1 + t;  t ≤ y_j ≤ 1 − t;  t ≥ 0.
```

`t* = 0` means some face is structurally tight; the point is still relatively interior in the
directions that matter, and the value is reported so the weakened claim is visible. Both modes
report the min-slack they actually achieve. `lpo_core_point = :uniform` keeps the clean Phase-3
version available for comparison.

## Phase 4 — the Pareto auxiliary LP, as implemented

Nothing is fixed but the VALUE. Variables `α ≥ 0`, `γᴼ, γᴰ ≥ 0` (built AND unbuilt), all with
a safety box (see below).

```
max   Σ_p α_p − Σ_j y^c_j g_j
s.t.  Σ_p α_p − Σ_{j ∈ J⁺} g_j                   =  z*                          (optimal face)
      α_p                                        ≤  w·demand_p·walk(o,d,WALK)   (x_walk rows)
      Σ_{(p,j,k) ∈ A^r} ρ_pjk                    ≤  c_route(V^r)   ∀ r ∈ current row set
      α_p − γᴼ_pj − γᴰ_pk                        ≤  w·walk(o,d,(j,k))
                                                    ∀ valid (p,j,k) with j ∈ J⁰ or k ∈ J⁰
      α, γ ≥ 0
```

The face equality is what makes re-optimising the WHOLE dual safe: it pins `C(ŷ) = z*`, so the
Pareto step can redistribute mass away from the incumbent but can never lower the cut at it.
For binary `ŷ` it is the stated `Σ_{j ∈ J⁺}` form of `Σ_p α_p − Σ_j ŷ_j g_j`.

The `x_walk` rows are easy to forget and fatal if forgotten: without them `α_p` can exceed
what direct walking costs and the cut over-estimates `Q_s` wherever the routes it credits do
not exist.

**Feasibility is never in question**, which is worth stating because the brief asked what
happens if preserving `z*` under the completion rows is infeasible: the incoming closed-form
point satisfies every row above (`α̂ ≤ c_walk` by LP dual feasibility; the pool rows by the
activated certificate; `ρ ≤ 0` by construction of the closed form; the face equality because
the closed form touches only `J⁰`, which is exactly the strong-duality identity
`Σα̂ − Σ Γ̂ ŷ = z*` the solver already asserts). So `ParetoRMP` always has a feasible point, and
its optimum is `≥ C_closed-form(y^c)` — and, by the same containment, `≥ C_baseline(y^c)`,
which is checked in Phase 8.

**The safety box.** Each variable is bounded by `100 ×` the largest incoming `α̂` (overridable
via `lpo_variable_bound`). This exists only so that an unbounded ray reports as a
`bound_binding` status rather than aborting the completion; validity never depends on it (a
certified point is certified whatever bounded it), only strength does. Whether it bound is
recorded per round and in the status.

## Phase 5 — the Pareto pricing loop

The pool `C⁺` certified the ORIGINAL duals. It does not automatically certify the Pareto ones.
So:

```
row_set ← C⁺
repeat
    solve ParetoRMP over row_set
    read (α, γᴼ, γᴰ); form ρ
    price with the EXISTING exact pricer at these ρ, over R⁺
    if min reduced cost ≥ −ε_price:  CERTIFIED — stop
    else: each returned route is a VIOLATED DUAL ROW — add it, repeat
```

**The primal/dual correspondence, stated because the code reads like column generation and is
not:** in the primal restricted master a priced route is a new COLUMN; in this auxiliary dual
LP the same route is a new CONSTRAINT. The Pareto problem is solved by constraint generation,
and its separation oracle is the route pricer.

Three implementation commitments:

- **Optimality of `ParetoRMP` over the current rows is NOT a stopping condition.** Only
  `min_{r ∈ R⁺} rc_r ≥ −ε_price`, from an EXHAUSTED round, is. Empty-and-not-exhausted (a
  budget stop) proves nothing.
- **An uncertified candidate is never installed.** The loop tracks the last certified point,
  initialised to the closed-form completion, which is certified. Truncation, rejection, an
  LP failure or the round cap therefore degrade to today's activated cut and **cannot** produce
  an invalid one. Uncertified calls are counted in `benders_lpo_certified` vs
  `benders_lpo_calls` — a run with a low ratio is the plain activated oracle wearing a
  different name and must not be reported as a Pareto arm.
- **Priced routes are NOT added to the primal pool.** They cannot improve `Q_s(ŷ)` (the
  activated solve already certified none exists at the original duals, and these price negative
  only against the moved duals), and adding them would invalidate the primal solution and duals
  that `_solve_one_joint_routing_assignment_benders_subproblem` reads after the completion
  returns.

**Why the loop's pricing stays cheap.** The `ρ ≤ 0` rows are IN the LP, so every candidate it
can propose already has `ρ ≤ 0` on every triple through an unbuilt station and
`joint_routing_assignment_pricing_candidates` drops it. Each separation round is therefore a
BUILT-ONLY search — the same one the activated oracle pays for anyway. This is the structural
difference from the row-generation completion removed on 2026-09-10, which recovered the cut
strength and then spent it: it lowered `γ` on the unbuilt stations, re-admitting them to the
pricer's filter, so every round became a full-universe search (6.8 s of built-only pricing
against 501 s of row generation at n=30 s=3).

## Phase 6 — global validity at termination

Two certificates at the end:

1. `Σ_{A^r} ρ ≤ c_route(V^r)` for all `r ∈ R⁺`, from the exhausted pricing round.
2. `ρ_pjk ≤ 0` for every assignment touching `J⁰`, imposed as rows and re-verified on the
   numbers read back from the solver (`worst_rho`), not inferred from the rows.

Under assignment-removal these give `Σ_{A^r} ρ ≤ c_route(V^r)` for all `r ∈ R`, by the
three-line split in Phase 2. With the `x_walk` rows and non-negativity, the point is feasible
for the full dual, so

```
Θ ≥ Σ_p α_p − Σ_j g_j y_j
```

is valid everywhere, and the face equality gives `C(ŷ) = z*` — re-checked as `face_residual`,
and independently by the solver's own strong-duality assertion, which under this oracle IS the
face equality.

## What is NOT claimed

- **Not Cummings' completion.** Theirs reconstructs the omitted linking multipliers
  coordinate-wise; ours are coupled through `ρ_pjk` and are BOUNDED instead. This is their
  local-Pareto principle with our completion mechanism.
- **Not Pareto-optimal over the full dual polyhedron of `R`.** The `ρ ≤ 0` rows are a strict
  tightening, so a true Magnanti-Wong point enforcing the real `R` rows can beat this one. That
  shortfall is what the optional gold-standard comparison on tiny instances measures; production
  never enumerates `R`.
- **Pareto-optimal only if `y^c` is in the relative interior.** Reported as `core_slack`; 0
  degrades the claim to "non-dominated among cuts that agree on that face".

## A defect this experiment found: the core point must have NO ZERO COORDINATE

Recorded first because it invalidated the first run of every measurement below.

`C(y^c) = Σ_p α_p − Σ_j y^c_j g_j` **ignores any coordinate with `y^c_j = 0`**. The auxiliary
LP is then free to put anything on `g_j`; and if `j ∈ J⁺`, `g_j` acquires an *unbounded*
improving ray, because the optimal-face equality pays `+1` per unit of built-station mass
while the objective charges `−y^c_j = 0` for it. The objective stops being a norm on the cut
and becomes a seminorm that ignores whole coordinates.

The original `:max_min_slack` core point produced exactly that. `max t` stops caring once `t`
is capped: one structurally tight face (a required location whose only candidates must all be
built) forces `t* = 0`, and at `t = 0` every `t ≤ y_j ≤ 1 − t` row is slack, so the LP returns
a **vertex** — measured, `y^c ∈ [0.000, 1.000]`, `min slack 0.0000`.

MEASURED damage, n=10 k=5 s=3 seed 44 (every cut still VALID, still tight at its anchor, still
certified — the collapse is purely in strength, which is what a degenerate core point costs):

| arm | mean `g_j` on `J⁰` | mean one-swap gap | one-swap neighbours where the cut says nothing |
| --- | --- | --- | --- |
| closed form | 7.1e3 | 7.0e3 | 5.6% |
| `lpo_baseline`, bad core point | 1.79e6 | 1.74e6 | 49.2% |
| `lpo_baseline`, fixed core point | 7.08e3 | 6.99e3 | 5.6% |

The fix is `lpo_core_point = :relative_interior` (now the default): CONSTRUCT a point with no
zero coordinate rather than optimise for one, by averaging the feasible points that maximise
each `y_j` and each endpoint row's slack, plus the max-min-slack point. The average of
feasible points is feasible and has `y^c_j ≥ (1/m) max y_j`, so every coordinate that *can* be
strictly interior is; a coordinate whose maximum is 0 is fixed at 0 in every feasible `y`, so
the cut's coefficient there is never read and junk on it is harmless. Measured after the fix:
`y^c ∈ [0.053, 1.000]`. `:max_min_slack` is kept for exactly this comparison.

## Results

`benders_activated_completion_audit.jl`, array job 22525353, n=10 p=8 k=5 max_stops=4,
3 seeds x {1, 3} scenarios, 8 anchors spanning the Q ranking. At this size the master's whole
feasible set (56–86 station sets) and the complete column universe (~17.8k columns) are both
enumerable, so `Q(y)` is known exactly everywhere and every cut is scored rather than
described.

### Correctness — established

**34/34 checks pass on every task.** Per arm and over every anchor:

- full-universe dual feasibility against the exhaustive enumerated pool: `min rc ≥ −3.5e-10`
  over 17,816 columns × 8 anchors;
- `α_p ≤ c_walk` (the `x_walk` dual rows): worst excess `+0.000e+00`;
- the restricted solve attains the exact `Q_s(ŷ)`: worst `|Q_arm − Q_exact| = 0.000e+00`;
- the cut is valid at every master-feasible `y`: worst violation `+1.9e-10` over all cuts × all
  station sets;
- the cut is tight at its anchor: worst slack `+4.5e-10`;
- `ρ_pjk ≤ 0` on every triple touching `J⁰`: worst `+4.5e-11`;
- `C(ŷ) = z*`: worst `|C(ŷ) − z*| = 4.5e-10`;
- `C^pareto(y^c) ≥ C^baseline(y^c)`: worst deficit `3.4e-10`;
- no cut over-predicts at any one-swap neighbour: `min Q(y') − C(y') ≥ −4.7e-11`;
- every completion certified (0 fallbacks to the closed form on any task).

Independently, `benders_oracle_smoke.jl` solves the same instance end to end with all five
oracles and they agree on the objective to **0.000e+00** (25771.187908 at n=10 s=3).

So: **the construction produces a genuinely globally valid Benders cut.** That part of the
hypothesis is confirmed.

### Information content away from the incumbent — a real but small gain

Mean one-swap cut gap `Q(y') − C(y')` (lower = more informative), and the fraction of one-swap
neighbours where the cut predicts `≤ 0` and therefore tells the master nothing it did not
already know from `Θ ≥ 0`:

| task | closed form | `lpo_baseline` | `lpo_pareto` | plain CG | recovery |
| --- | --- | --- | --- | --- | --- |
| seed 42 s=1 | 5304.5 / 5.0% | 5257.8 / 2.3% | 4686.1 / **0.0%** | 433.2 / 0.0% | 12.7% |
| seed 43 s=1 | 5739.5 / 10.1% | 5687.9 / 7.7% | 5628.5 / **0.0%** | 208.8 / 0.0% | 2.0% |
| seed 44 s=1 | 5758.8 / 6.0% | 5694.6 / 6.0% | 5653.3 / **0.0%** | 200.5 / 0.0% | 1.9% |
| seed 42 s=3 | 4976.9 / 5.2% | 4941.4 / 2.6% | 4778.3 / **0.0%** | 323.7 / 0.0% | 4.3% |
| seed 43 s=3 | 5734.0 / 9.5% | 5683.2 / 7.3% | 5632.8 / **0.0%** | 206.9 / 0.0% | 1.8% |
| seed 44 s=3 | 7044.3 / 5.6% | 6987.9 / 5.6% | 6931.7 / 3.6% | 395.6 / 0.0% | 1.7% |

`recovery` is `(closed − pareto) / (closed − CG)`: how much of the gap to full-station pricing
the Pareto step closes. Median **2.0%**, range 1.7–12.7% over the six tasks. `lpo_baseline`'s own recovery is
~1% everywhere, so the Pareto step is worth roughly 2–4x the completion LP alone — Method B
beats Method A on every metric of every task, as it must (its feasible set contains the
baseline point), but both are small next to the 20–30x that separates the whole activated
family from plain CG.

**The one qualitative win is `dominated`.** The closed form leaves 5–10% of one-swap
neighbours with a non-positive prediction; `lpo_pareto` drives that to **0.0% on five of six
tasks** (3.6% on the sixth). So the Pareto cut does say something everywhere in the immediate neighbourhood, which
is exactly the property the hypothesis was about — it just does not say very much.

Inactive coefficients move the same way and by the same small amount: mean `g_j` on `J⁰` falls
1–13% (e.g. 5349.8 → 4657.0 at seed 42 s=1; 7144.9 → 7040.7 at seed 44 s=3), against plain
CG's 343–567. **So no: the inactive coefficients did not become materially smaller.**

### Loop behaviour — unchanged at n=10, but it does flip one cell at n=12

`benders_oracle_smoke.jl`, job 22525622, every arm on the same instance and route universe
(`max_stops = 4` explicit on all of them), all agreeing on the objective to 0.000e+00:

| instance | enumeration | plain CG | closed form | `lpo_baseline` | `lpo_pareto` |
| --- | --- | --- | --- | --- | --- |
| n=8 s=1 | 2 / 1 | 2 / 1 | 12 / 11 | 11 / 10 | 11 / 10 |
| n=10 s=1 | 4 / 5 | 4 / 5 | — | — | — |
| n=10 s=3 | 4 / 5 | 4 / 5 | 30 / 77 | 30 / 77 | 30 / 77 |
| n=12 s=1 | 4 / 3 | 6 / 5 | **200 / 200, FEASIBLE** | **200 / 200, FEASIBLE** | **177 / 176, OPTIMAL** |

At n=10 s=3 a 2–4% strength recovery is not enough to change which station sets the master
visits — all three activated arms need the identical 30 / 77. At n=12 s=1 it is enough to
matter qualitatively: the closed form and `lpo_baseline` both hit the 200-iteration cap
without proving optimality, and `lpo_pareto` converges. **That is the only measured case where
the Pareto step changes the OUTCOME rather than the metrics**, and it is still 30x plain CG's
6 iterations, so it should be read as "the completion family is still the wrong place to be",
not as a scaling win.

The earlier 87 iterations / 211 cuts were **entirely** the degenerate core point, not a
property of Pareto cuts; that number should not be repeated.

### Cost — the `ρ ≤ 0` rows do keep separation activated

Per completion at n=10 s=3: 1.4–2.8 Pareto rounds, 43–59 seed rows, **0.8–3.5 generated
rows**, 0.02–0.04 s of LP and 0.02 s of pricing. Most rounds certify on the first pricing
call. This is the structural claim of the design and it holds: the completion rows are in the
LP, so the pricer's `ρ > 0` filter drops every unbuilt-station candidate and each separation
round is a built-only search. The 2026-09-10 row-generation completion, which lowered `γ` on
the unbuilt stations instead, turned 6.8 s of built-only pricing into 501 s at n=30 s=3.

The residual `certified_bound_binding` statuses (5–13 of 24 calls, **1 variable of 54**) are a
degenerate direction the objective is flat along, not a runaway: with the fixed core point the
count fell from 28-of-54 on 24-of-24 calls to 1-of-54 on a minority.

### Conclusion

The hypothesis is **confirmed on validity and refuted on magnitude**. Optimal-face Pareto
re-optimisation with a certified pricing loop does produce a globally valid cut that is
strictly more informative away from the incumbent — and the binding constraint on activated
cut strength is **not** which dual the solver returns from the optimal face. It is the
conservative `ρ_pjk ≤ 0` completion itself, which pins `g_j` near `α_p` on every unbuilt
station because the only term it can credit, `w · walk`, is 0.26–0.95% of `α_p`. Choosing
optimally *within* that constraint buys a few percent; escaping it is what would matter.

### The gold standard confirms the diagnosis, and quantifies it

`benders_lpo_gold_standard.jl`, job 22525969, 10/10 checks on every task. The `gold` arm is the
same optimal-face core-point objective with the REAL route rows of the enumerated `R` and **no
`ρ ≤ 0` condition at all** — a true Magnanti-Wong dual of the actual second stage, and
therefore an upper bound on what any completion-based method can reach. Testing only: it
enumerates `R`.

n=10 p=8 k=5 max_stops=4 seed 42 (pool 16,320 = the complete universe):

| arm | `C(y^c)` | mean one-swap gap | max one-swap gap | mean `g_j` on `J⁰` |
| --- | --- | --- | --- | --- |
| closed form | −3364.7 | 6155.1 | 13163.3 | 5841.4 |
| `baseline` | −3118.7 | 6030.6 | 12830.0 | 5724.2 |
| `pareto` | −1601.8 | 5676.2 | 12662.8 | 5274.6 |
| **`gold`** | **+13771.4** | **219.7** | **1268.2** | **396.6** |

`gold` is a **28x** improvement on the closed form's one-swap gap and lands essentially at
plain full-station CG (433 on the same instance family) — which is the point: escaping the
completion recovers *all* of the strength, and it is available to a dual that does not need
`ρ ≤ 0`.

The split, over four tasks (n=8/10, max_stops 3/4, two seeds):

| | seed 42 ms3 n=8 | seed 43 ms3 n=8 | seed 42 ms3 n=10 | seed 42 ms4 n=10 |
| --- | --- | --- | --- | --- |
| `gold − pareto`, as % of `gold − closed_form` | **98.9%** | **93.7%** | **94.4%** | **89.7%** |
| the Pareto step's own share (`pareto − baseline`) | 61.7 | 396.3 | 808.5 | 1516.9 |

**The conservative `ρ ≤ 0` completion accounts for 90–99% of the lost cut strength; the choice
of dual on the optimal face accounts for the remaining 1–10%.** That is the answer to the
question this whole experiment was built to settle, and it says the Pareto machinery is
working correctly on a problem that was never the bottleneck.

### Where to go instead

The completion has to force `ρ_pjk = α_p − γᴼ_pj − γᴰ_pk − w·walk ≤ 0` on every triple through
an unbuilt station, and the only term it can credit is `w · walk`, which is 0.26–0.95% of
`α_p`. `gold` shows the real dual does not need anything like that much `γ` — it charges the
ROUTE TRAVEL an unbuilt station would cost (median 33% of `α_p`), which no termwise condition
can express because one column can serve several out-triples at the same station.

The obvious candidate was a completion that charges a per-station DETOUR rather than a
per-triple walk. **It was worked out in full, measured, and rejected** — see below.

## The detour credit: designed, measured, NOT VIABLE

`benders_detour_credit_potential.jl`, job 22531459.

### The construction (sound, and worth keeping on the record)

Validity does not need `Σ_{A_out} ρ̃ ≤ 0`; it only needs
`Σ_{A_out} ρ̃ ≤ c_route(V) − c_route(V″)`, and the right side is the detour cost of the unbuilt
stations the route visits. Three parts:

1. **The bound.** `c_route(V) − c_route(V″) ≥ β Σ_{j∈U(r)} δ_j`, by deleting the unbuilt nodes
   one at a time. `δ_j` must be the minimum saving over **every position `j` could occupy**:

       δ_j = min{ min_{a≠b≠j}[t(a,j)+t(j,b)−t(a,b)],  min_b t(j,b),  min_a t(a,j) }

   The two endpoint terms are real because `route` is an open path with no depot leg. **The
   minimum over BUILT pairs — which `benders_activated_completion_audit.jl`'s `β·δ` column
   prints — is larger and crediting it is UNSOUND**: when two unbuilt stations are adjacent,
   deleting the first has an unbuilt neighbour. Measured, one anchor's built-pairs credit is
   113% of the entire gap, which is the over-credit made visible.

2. **The distribution, exactly, with no worst-case denominator.** The detour is paid once per
   station but `ρ̃` accrues once per assignment, so a per-triple credit lets one column claim
   the same cost as many times as it has passengers there. The fix uses that **a demand group
   appears at most once per column**: introduce `u_pj ≥ 0` and impose

       (i)  ρ̃_pjk ≤ u_pj·1[j∈J⁰] + u_pk·1[k∈J⁰]     per triple
       (ii) Σ_p u_pj ≤ b_j                          per unbuilt station

   Any column's draw at `j` is then a *subset* sum of claims already capped by (ii). The LP,
   not a formula, picks the split.

3. **One gap in the telescoping, and its fix.** Each deletion saves `≥ δ_j` only while the node
   still has a neighbour, so deleting down to the EMPTY route performs one deletion too many —
   which happens exactly when the route visits no built station at all. Setting

       b_j := β · max(0, δ_j − δ_min),   δ_min = min_{j∈J⁰} δ_j

   closes it with no case analysis: for an all-unbuilt route, delete down to the smallest-δ
   node, giving `τ_V ≥ Σ_{j∈U} δ_j − min_{j∈U} δ_j ≥ Σ_{j∈U} b_j/β`, while `A_in` is empty so
   the first line of the split contributes 0.

It is pricing-free: `α̂` and the built `γ̂` never move, so the activated certificate transfers
verbatim. `ρ` may go POSITIVE on unbuilt triples under the credit, so the credited point must
never reach the pricer — price at the strict point, emit the cut from the credited one.

### The measurement that killed it

The credit is a single ADDITIVE reduction of `g_j` per station, capped at `β·δ_j`. The bar is
`g_j^closed − g_j^gold`, mean over the unbuilt stations at each of 8 anchors:

| instance | `g_j` closed | `g_j` gold | gap | sound `β·δ̄` | ceiling |
| --- | --- | --- | --- | --- | --- |
| n=10 p=8 s=1 seed 42 | 5971.7 | 484.6 | 5487.1 | 305.0 | **5.6%** |
| n=10 p=8 s=1 seed 43 | 6686.9 | 274.4 | 6412.5 | 474.9 | **7.4%** |
| n=10 p=8 s=3 seed 42 | 5588.8 | 322.0 | 5266.9 | 297.7 | **5.7%** |
| n=8 p=6 s=1 seed 42 | 6472.8 | 287.7 | 6185.2 | 553.8 | **9.0%** |

**5.6–9.0%, and that is an optimistic CEILING** — it assumes every unit of budget converts 1:1
into `g_j` reduction, which the `ρ≤0` rows need not allow. The unsound built-pairs version
would read 13.7–18.2%, which is the size of the error that formula invites.

**Why, and why it is structural rather than instance-specific.** `δ_j` is a minimum over ALL
station pairs, so it is near zero whenever `j` lies almost on the line between *some* pair —
which is most stations in a dense candidate set. Median `δ_j` is 0.144 at n=10 (β·δ = 1.4) and
1.152 at n=8. **It gets worse as the station set grows denser**, i.e. exactly in the direction
that matters. The endpoint terms cost only 0.4% of the credit; the insertion minimum is already
~0 on its own.

Per anchor the spread is wide — 28.1%, 14.6%, 13.2% at the top, 0.0% and 0.6% at the bottom —
so it is not uniformly worthless, just far short.

### What this settles, jointly with the gold standard

Two independent measurements now say the same thing from opposite directions. The missing 90%
of activated cut strength is **not a cost term the completion forgot to charge**. It is
per-column slack — most columns being nowhere near binding in `Σ_{A_in} ρ̃ ≤ c_route(V″)` — and
no route-free, per-station bound can see it. Any completion-shaped fix is capped well below
what is needed.

The live directions are therefore not completions:

- **Row generation with a DECOUPLED pricing point.** The 2026-09-10 separation arm died because
  it re-priced *at the candidate*, whose low unbuilt `γ` re-admits every station to the pricer's
  filter. But the cut point and the pricing point need not be the same point (the detour work
  above establishes the pattern): `P_price`'s exhaustion certifies the `A_in` rows, and that
  certificate depends only on `α̂` and the built `γ̂`, which `P_cut` shares. What is still needed
  is a way to find violated rows that is not a full-universe label search — the relaxed-cluster
  certifier is the obvious candidate, since it bounds every real route's reduced cost without
  searching for one.
- **Before writing any of that**, the cheap decisive experiment: build the gold-style dual from
  the CG oracle's ACCUMULATED POOL rows only, and test against the enumerated universe whether
  that point is (a) dual-feasible and (b) near gold. The pool already carries columns through
  currently-unbuilt stations. If it is both, the whole separation question collapses into
  "warm the pool", which the oracle does for free.

## Dual anatomy: it is a PRICING gap, and gold's dual rests on ~4 columns

`benders_dual_anatomy.jl`, job 22533140, n=10/n=8 at `max_stops` 4/3, three instances, 8
anchors each. The enumerated pool IS the universe at these caps, so every `min rc` below is
dual feasibility itself.

### 1. Plain CG is already ~96% of gold

| dual | `g_j` on `J⁰` | mean one-swap gap | dominated |
| --- | --- | --- | --- |
| closed form (what we ship) | 5350 / 5763 / 5769 | 5559.9 / 5887.2 / 6139.2 | 5.0% / 10.1% / 4.8% |
| **plain CG** (full-station pricing, RAW duals) | 567 / 288 / 288 | 407.1 / 198.7 / 239.1 | **0.0%** |
| gold (MW over the enumerated universe) | 485 / 274 / 288 | 151.2 / 59.0 / 79.7 | 0.0% |

**Plain CG closes 95.3% / 97.6% / 97.4% of the closed→gold distance.** That reframes every
earlier result: `gold` is not a distant theoretical target, it is roughly where the oracle we
already ship lands. The question was never "how do we reach gold" -- it is "what do the
full-station iterations produce, and can it be had cheaper".

### 2. The trajectory: value and dual feasibility arrive TOGETHER

Scored after every inner CG iteration from a cold pool, both arms:

    y=[1,2,3,7,9] s=1  PLAIN CG
      iter  pool  priced       Q_s      g_J0   swap mean     min rc
         1    30      17  15803.00       0.0    -2631.56  -3.55e+03
         2    47      18  15803.00       0.0    -2631.56  -3.55e+03
         3    65       0  12249.40       0.0      922.04  -9.09e-13

Two things worth keeping:

- **Intermediate duals are not merely weak, they are INVALID** -- `min rc` is -3.5e3 and the
  one-swap gap goes NEGATIVE (-2631.6), i.e. the cut would over-predict `Q`. This is the
  "converged rounds only" rule shown rather than argued.
- **`Q_s` is still moving on the final iteration in every trace** (15803 -> 12249 at the last
  step). So "the value is attained early and late iterations buy only duals" is NOT true
  within a single run; it is a statement about ACTIVATED versus PLAIN, which reach the same
  `Q_s` with pools of 50 vs 65 and 65 vs 102. The summary line printed by the script measures
  the within-run version and therefore always reports `value == dual feasibility`; read it as
  a sanity check, not as the premise.

### 3. Gold's dual is held in place by ~4 columns, and the activated pool has NONE of them

The gold LP's route-row duals are the `theta` weights, so a non-zero one names a column that
actually pins the gold dual.

| | n=10 seed 42 | n=10 seed 43 | n=8 seed 42 |
| --- | --- | --- | --- |
| binding columns (mean) | 3.8 | 4.2 | 4.0 |
| as a share of the universe | 0.023% | 0.057% | 2.47% |
| median stops | 4.0 | 3.0 | 3.0 |
| median unbuilt stations touched | 0.8 | 0.9 | 1.0 (max 3) |
| **in the ACTIVATED pool** | **0.0%** | **7.1%** | **8.3%** |
| in the plain-CG pool | 30.8% | 64.3% | 66.7% |

**This is the answer.** The activated pool contains essentially NONE of the columns that hold
a good dual in place, so no amount of reoptimising over its rows can produce one -- which is
exactly why the optimal-face Pareto step could only move 2%. **It is a pricing gap, not a
dual-selection gap.**

Note also that plain CG holds only 31-67% of gold's binding set yet reaches 96% of its
quality: the good dual has many representations, and hitting a few of the right rows is
enough.

### The direction this points at: price at the CORE POINT, not only at the incumbent

The binding columns touch a median of **~1 unbuilt station**, so they are mostly INSIDE the
activated search universe. The activated pricer could find them. It does not, because they
have non-negative reduced cost at the completed duals -- they are **dual-informative, not
primal-improving**, and CG only asks for the latter.

And there is a clean characterisation of which columns they are. Strip the face equality from
the gold LP and what remains,

    max  sum_p alpha_p - sum_j y^c_j g_j   s.t. the route rows and the walk rows,

is exactly the dual of the second stage evaluated at `y^c`. **Gold's binding columns are
approximately the columns that would be priced if the subproblem were solved at the CORE POINT
instead of at the incumbent.** That is Magnanti-Wong's core point applied to *which columns to
price* rather than to *which dual to pick*, and it is a different lever from anything tried
here.

Sketch, and it fits the existing machinery:

1. activated CG at `yhat` to exhaustion -- cheap, built-only; gives the value and the
   certificate, unchanged;
2. a **budget-capped, non-exhaustive** pricing pass with `y` fixed to `y^c` -- pure row
   harvesting, so validity never depends on it finishing;
3. re-solve at `yhat` (`Q_s` is unchanged: those columns are not improving there) and take the
   duals;
4. optionally the optimal-face reoptimisation, which now has rows worth reoptimising over.

The open question is whether a truncated core-point pass actually finds the handful of columns
that matter. That is measurable with this same script.


## The flatness (Lipschitz) cut: measured, middle case, and unprovable

`benders_flatness_bound.jl`, job 22537598. The last direction tried in this thread, and the
only one that never touches dual feasibility:

    Theta >= Q_s(yhat) - V * sum_{j not in yhat} y_j

This is legal for two reasons that no other cut here could use: a cut only has to UNDER-
ESTIMATE `Q`, and -- the master being a MIP -- only at BINARY master-feasible points. It need
not be a supporting hyperplane of the convex relaxation and it need not come from a dual
point. Its ingredients are `Q_s(yhat)` (the ACTIVATED oracle gives it, built-only) and `V`,
a structural constant.

`V` must be the Lipschitz constant in the swap metric,
`V = max over ordered pairs of [Q(yhat) - Q(y)] / |y \ yhat|`.

| instance | mean `Q_s` | `V_1swap` | `V_required` | as % of `Q_s` | flatness-cut one-swap gap |
| --- | --- | --- | --- | --- | --- |
| n=10 p=8 s=1 seed 42 | 13851.3 | 2449.0 | 2449.0 | 17.7% | 2475.9 |
| n=10 p=8 s=1 seed 43 | 9854.7 | 1915.0 | 1915.0 | 19.4% | 1858.5 |
| n=8 p=6 s=1 seed 42 | 10190.0 | 1915.0 | 1915.0 | 18.8% | 1868.7 |

Validity checked, not assumed: worst violation over the whole feasible set is `+0.000e+00`,
and `dominated` is 0.0%.

**`V_required == V_1swap` exactly on all three.** So the value function is subadditive over
swaps here and station complementarity does not bite -- the multi-swap worry that killed the
finite-difference construction is real in principle and absent in this data. An observation on
three instances, not a theorem.

**The verdict is the middle case.** Against the same scale: closed-form completion 5300-6100,
**flatness cut 1859-2476**, plain CG 199-430, gold 59-220. So ~2.5-3x better than the
completion and ~5-9x worse than plain CG.

Two things make it worse than that number looks:

- **It cannot guide.** `g_j = V` for EVERY unbuilt station, so `C(y') = z* - V` at every
  one-swap neighbour and the cut gives the master no reason to prefer one station over
  another. It is a bound-improving device, not a steering one.
- **`V` is not provable by any route I can see.** The only structural argument available --
  every required location has several stations inside `max_walking_distance`, so removing one
  changes each group's options little -- bounds the WALKING part of the change, which is
  0.7-2.2% of `alpha_p`, while route travel is 92.5% of the objective. Swapping a station
  changes which ROUTES are feasible, and that is exactly what cannot be bounded without
  searching routes. Same wall as the detour credit, in a different disguise.

## Closing synthesis: why every direction in this thread landed in the same place

Four mechanisms, all measured on instances where the answer is exactly computable:

| mechanism | what it tried | result |
| --- | --- | --- |
| optimal-face Pareto reoptimisation | pick a better point on the degenerate dual face | **2%** of the gap |
| detour credit | raise the assumed floor with a real cost term | **5.6-9%** ceiling |
| rotating priced set | certify a few stations instead of completing them | partial; does not scale past n=10 |
| flatness / Lipschitz cut | bypass duals entirely | 2.5-3x better, 5-9x short, unprovable |

One sentence covers all four:

> **`g_j` on an unbuilt station is `~ sum_p alpha_p` unless station `j` was in the pricing
> search, and you cannot know the floor at `j` without searching through `j`.**

The floor is set by the cheapest CONCEIVABLE column through `j`, and there is always a
conceivable column that inserts `j` almost for free (measured: median `delta_j` = 0.144). The
cheapest ACTUAL column through `j` is far more expensive -- plain CG discovers that and sets
`gamma_j = 0` -- but only a search establishes it.

And `alpha` cannot be attacked either: `sum_p alpha_p = Q_s(yhat) + sum_{j built} Gamma_j`
exactly, both arms attain the same `Q_s(yhat)`, and their built-station duals agree to under
1% (153.6 vs 131.3 per built station against `sum alpha ~ 12250`).

**Recommendation: treat the activated oracle as a closed question.** Plain CG already reaches
96% of gold and holds to n<=20 all scenarios / n=25 to <=5 / n=30 at s=1. Below that frontier
it is the right tool and is already shipped. The real open problem is not better duals from a
restricted search -- it is **full-universe certification at n >= 30**, which is the
relaxed-cluster line of work and whose known weak point is its certification rate (0/31
one-shot, 4/10 at n=40 with cut rounds).