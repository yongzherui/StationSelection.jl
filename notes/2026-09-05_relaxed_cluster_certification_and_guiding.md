# Relaxed-cluster pricing: certification is dead one-shot, alive with no-good cuts

Status of `src/opt/label_setting/joint_routing_assignment/relaxed_cluster/` as of
2026-09-05. Everything below is measured on Zhuzhou `n=15, p=16, s=3, seed 42` unless
stated; treat every number as one instance, one seed, one dual trajectory.

## What the relaxation is

Partition the stations into `K` cells. Build a cluster graph where travel between cells is
the **minimum** member-to-member time (then a metric closure) and a passenger's reward for
a cell pair is the **maximum** over its real station pairs. Every real route then maps to a
cluster route that is no slower and no less rewarded, so

    min over relaxed cluster routes  <=  min over real station routes

over the full revisit-tolerant universe. Full argument in `relaxed_cluster/types.jl`.

There is **no relaxed pricer** — only a relaxed *graph*. `RelaxedClusterPricingData.inner`
is an ordinary `JointRoutingAssignmentPricingData` whose nodes are cluster-graph nodes, so
`JointRoutingAssignmentSearchContext` runs on it unchanged. An earlier version carried its
own label/seed/extend/context/replay to special-case intra-cluster passengers; making that
an explicit optional service arc deleted all five files.

## Three things that are load-bearing and non-obvious

1. **The metric (Floyd–Warshall) closure is required for validity, not tightness.** A
   metric *station* matrix does not give a metric *cluster* matrix — `min` destroys it.
   Counterexample on a line: `A={0}, B={10,50}, C={60}` gives `min(A,C)=60 > min(A,B) +
   min(B,C) = 20`. Station-age pruning treats `travel(current,dest)` as a lower bound on
   remaining time, so on a non-metric matrix it drops clocks the relaxation still needs,
   the search *under*-collects reward, and the bound turns the wrong way.
2. **Ride limits need their own relaxation.** `R_bar = max` over the cell pair, taken
   independently of the reward max. Using a smaller `R` lets the relaxed image fail a
   ride-limit test the real route passed, losing the reward — same failure direction.
3. **Intra-cluster service must be an OPTIONAL arc.** A passenger served inside one cell
   has no inter-cluster arc to be certified on. Crediting it on arrival is valid but loose;
   charging `tau_intra(C)` on arrival is *invalid* (a real visit touching one station pays
   nothing inside the cell). The fix is a second node `C'` reached only by paying
   `tau_intra(C) = min_{j!=k in C} tau(j,k)`, with the intra candidate rewritten to
   `(p, C, C')`.

## Measured: one-shot certification does not work

`certification_pricing_mode = :relaxed_cluster` — exhaust the relaxation, certify if
nothing prices below `-tol`. **0/31 attempts at every `K < n`**, all *refuted*, never
budget-bound. Slack at the converged duals:

| K | 3 | 6 | 9 | 12 | 15 |
| --- | --- | --- | --- | --- | --- |
| relaxed min rc | −3121 | −2483 | −1008 | −943 | 0 |

The margin it has to hit is **exactly zero**, and that is structural rather than
instance-specific: `theta` carries only a lower bound of 0, so any column the master
actually uses (`theta_j > 0`) has reduced cost exactly 0 at **every** optimal dual by
complementary slackness. No dual stabilization or interior-dual choice creates headroom.

The master is also primal degenerate (27 of 31 basic `theta` at their lower bound), but the
verdict does not depend on that. Do **not** re-derive it via degeneracy: "primal degeneracy
⟺ dual non-unique" is false (only dual multiplicity ⟹ primal degeneracy), and a nonbasic
column with zero reduced cost implies an alternative *basis*, not an alternative
*solution*, when the ratio-test step is zero — which it routinely is under degeneracy.

Charging intra-cluster travel (item 3 above) tightened the bound by **1.2–1.8%** and the
absolute gain was identical within pairs (37.52 at K=3/6, 17.25 at K=9/12 = `beta` × one
within-cell hop), i.e. it lands on exactly one service arc. So the earlier "slack is
dominated by free intra-cluster travel" diagnosis is **disproven**. The real source is the
inter-cluster `rho_bar = max` letting **every passenger independently pick its best station
inside a cell** while a real route must commit to one. K=12 isolates it: eleven singletons
(where the relaxation is exact) plus one 3-cell and one 2-cell still leave 943.

## Measured: no-good cuts DO certify

`certification_pricing_mode = :relaxed_cluster_nogood`. Instead of giving up when the
relaxation finds an improving cluster route, verify it:

    1. relaxed search (respecting all cuts)      -> best improving cluster route, support T
    2. none, and the search exhausted            -> CERTIFIED
    3. exact search over stations(T), exhaustive
         improving column found  -> REFUTED (true negative)
         nothing                 -> T is barren: cut it, go to 1

**Certifies at K=9 and K=12**, same LP objective as baseline (36065.5464), 0 inconclusive.
Cuts needed per scenario: **5, 4, 1** at K=9 and **10, 6, 1** at K=12. The other 30 CG
iterations cost a median of **1 round and 0 cuts** — the relaxed optimum's support really
does hold an improving real column, so the loop refutes immediately. Cut enumeration is
paid only in the iteration where certification is at stake. Coarser K is *cheaper* to
certify (K=9 < K=12), the opposite of what the raw-slack ladder suggests.

Bound trace (K=9 scenario 2), monotone non-decreasing by construction since each cut only
removes routes:

    relaxed_rc:  -1008.4 -> -672.3 -> -521.3 -> -63.2 -> +43.2 (certified)
    real rc in S:    -0.0 ->   -0.0 ->   -0.0 ->  -0.0 -> n/a

Every intermediate `subset_rc` is ≈0: the loop keeps landing on supports whose stations
hold a route at reduced cost exactly zero — the degenerate basic columns. It walks the
degenerate face one support at a time, and `|S|` barely moves between consecutive supports
(7→7→8→8), so it peels a thin local family rather than enumerating a combinatorial space.

### Two correctness traps, both hit and both fixed

- **The cut direction.** `|route ∩ T| <= |T|-1` is *invalid*: a real improving route
  touching `A,B,C,D` was never examined by the exact search over `stations({A,B,C})`, yet
  that cut deletes its image. The sound cut is *"visit at least one cluster **outside** T"*.
  It is also stronger operationally — it kills every route confined to any subset of `T`.
- **Cut-escaping detours.** The exact pricer's candidate generation is *reward-driven*: it
  proposes a node only if visiting it unlocks a new reward layer. Under a cut a route may
  need to visit a cluster purely to *leave* the cut set, gaining nothing. Without widening
  candidate generation the search under-reported (−114.6 vs a true −250.5 on cut `{1,2}`,
  winner `[2,1,4,3]` whose last stop opened no reward), which is the false-certificate
  direction. Found by the randomized brute-force test; four hand-written tests missed it.

Seeding stays restricted to opportunity origins, and that IS sound: truncate a real
improving `R` to its first origin — still improving (the dropped prefix earns nothing) and
still un-confined to every cut set `T`, since a truncation confined to `stations(T)` would
have been found by the exhaustive search that made `T` a cut.

## Measured: guiding works, and needs no tightness at all

`pricing_mode = :relaxed_cluster_guided` — price the cluster graph, take the winning
routes' members as a **station subset**, run the exact pricer restricted to it. Columns are
real, so `round.jl` is unchanged; it is a restriction, so it cannot certify
(`cg_optimality_scope = "relaxed_cluster_station_subset_only"`; pair with
`warm_start_pricing_mode` to certify).

**72/72 rows containment and recovery** across `K ∈ {3,6,9,12}` × iterations
`{1,2,4,8,16,32}` × 3 scenarios, with mean `|S|/n` of 1.00 / 0.88 / 0.58 / **0.43**. At
K=12 the exact pricer gets 43% of the stations and returns a column of identical reduced
cost, at every stage of the dual trajectory.

## Runtime: slower here, and that is expected

At n=15 the no-good loop is *slower* than plain CG, and the mechanism says it must be:
certification runs **before** pricing rather than instead of it, so the 30 non-certifying
iterations pay a relaxed search plus a subset exact search *on top of* the normal round —
and the baseline reported `cg_certifying_rounds = 0`, i.e. it converged without ever paying
the expensive certifying escalation the loop exists to replace. Cost with nothing to
offset it.

The regime where it could pay is where the baseline *does* escalate — Study 5 put the
certifying share at 77.7% at n=30 with runs quitting on budget. Wall-clock numbers at n=15
are worthless anyway: the unchanged baseline measured 154.0 / 334.5 / 419.7 / 412.6 s
across runs (2.7× spread on `mit_preemptable`), while cut counts reproduce exactly.

## Test coverage, and what it is worth

Two mutations are verified to fail, which is what separates "green" from "working":

| mutation | caught by | signal |
| --- | --- | --- |
| `rho_bar` max → min | randomized brute-force bound test | relaxed −65.2 vs exact −73.0; and a false certificate while a real column priced at −84.4 |
| metric closure removed | search-vs-brute-force | search −124.1 vs true relaxed optimum −167.3 |
| reward-driven candidate generation (the real bug) | randomized brute-force cut test | −114.6 vs −250.5 |
| force every support barren (false certificates) | end-to-end objective equality | 1399.90 vs 734.50, at all 3 K |

The bound test needs **both** a `:spread` and a `:tight` station layout: a first version
used only well-separated points and the `rho_bar` mutation did **not** fail it, because
generous travel slack absorbs an under-credited reward. Do not simplify that fixture away.

## Open

- Everything is one instance / one seed. Cut counts at n≥25 are unknown, and that is the
  only regime where the loop replaces something expensive.
- The "only cut on an *exhausted* subset search" guard is inspected, not mutation-tested
  (hard to trigger a timeout deterministically).
- `benchmarks/study9_relaxed_cluster_certification/` (30 jobs, baseline + K ∈ {3,6,9,12,15}
  at n=15) is scaffolded and generated but has never been submitted.
- Subset row cuts on cluster sets remain the untried lever for the `rho_bar` slack itself,
  as opposed to enumerating around it.
