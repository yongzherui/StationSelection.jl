# Reference: `AggregateODRouteModel` Benders variables/constraints and the three "chain" styles

**Fix applied (2026-07-24, same day), status: INSUFFICIENT -- investigation still open.** Rather
than adding the missing `kappa` dual (Section 3's originally-proposed fix), the fix actually applied
removed the nearest-open lower-bound row (4) from both `_endpoint_chain_variable!`/
`_endpoint_big_m_variable!` entirely, and removed `unmet_demand_penalty`/"always feasible" mode from
the whole codebase (16 files) -- that mode was the row's only other reason to exist (`sum(z)<=1`
needed it; `sum(z)==1`, now unconditional everywhere, does not, per the redundancy argument in
Section 3). `test/runtests.jl` passes 1048/1048 both interactively and via `sbatch` on
`mit_quicktest`.

**But this did NOT fix the real-world failure.** Rerunning the exact real Zhuzhou instances that
previously crashed with `completion_infeasible` (`grid_n20_p16_s123`/`grid_n20_p32_s42`, both
`bendersY_zerocomp_ms4` and `bendersY_mw_ms4`) against the fixed code, via `sbatch`
(`scripts/sbatch_method_compare.sh`), reproduces the *identical* crash -- `completion_infeasible`
at `cut_id=1` in the very first Benders iteration (no `aggregate_od_route_benders_iterations.csv`
ever gets written), in every one of the four (instance × cut_derivation) combinations tried. Stack
trace line numbers confirm the fixed code is genuinely what's running (`y_mw_cut.jl:749`, `y.jl:562`
-- shifted from the original `:732`/`:671` by exactly the lines removed), so this is not a stale
build/cache artifact. The row-4 fix was real (verified by direct algebraic proof, not just testing)
and is being kept, but it was evidently not the only cause -- or not a cause at all -- of the
real-instance failures; the small synthetic test fixtures in `test/opt/test_aggregate_od_route_*mw*`
apparently don't exercise whatever the real trigger is.

**Where to look next, and why the core-point construction (Section B / `_y_master_core_point`) is
NOT it, despite looking like a natural suspect**: `y_core` (the core-point construction's output)
only ever appears in `_restricted_mw_completion_lp` via `phi_core_expr`, which is used *exclusively*
in the `@objective(m, Max, phi_core_expr)` branch (`objective_mode == :maximize_core`) -- it never
appears in any `@constraint`. The completion LP's only feasibility-relevant pieces are (a) the
dual-feasibility inequalities (`alpha/rhoO/rhoD/sigma/lambda/mu/nu`, none of which reference `y_core`
or `y_hat`) and (b) the tightness equality `phi_ybar_expr == Q_bar`, which uses `y_hat` only. So a
bug in how `y_core` is computed can change which *feasible* point the `:maximize_core` objective
picks (cut quality), but cannot make the LP *infeasible* -- and the reproduction above fails under
`:zero_completion` too, where `y_core` isn't even meaningfully used (the objective is a flat `0.0`).
This was checked directly against the user's hypothesis that the newly-eager
`_add_default_endpoint_coverage_constraints!` master rows needed to be reflected in the core-point
LP -- they already are (`_restricted_mw_endpoint_rows` independently derives the identical row set
from `_nearest_open_endpoint_candidates` over the same `requests`; the only difference from the
master's own `_aggregate_od_route_endpoint_candidate_sets` is a dedup key that includes `side`
there and not in `_restricted_mw_endpoint_rows` -- harmless, since the constraint's content depends
only on the candidate station set, not on which physical endpoint/side asked for it). **Next place
to look: Sections C/D** (`_certified_route_covering_pi`/`_certified_qbar`, the route-covering dual
certification and `Q_bar` derivation) or the shared `alpha/rhoO/rhoD/sigma` x-dual block -- these are
the only pieces common to both `:zero_completion` and `:restricted_mw_fixed_pi`, matching the
observation that both modes fail identically.

*2026-07-24.* Written before touching any code, to have one place that states exactly which
variables and constraint rows exist in each decomposition/style combination, prior to fixing the
`completion_infeasible` bug tracked in memory (`project_mw_completion_infeasible_open`). Everything
below is read directly off `src/opt/constraints/aggregate_od_route.jl` and
`src/opt/optimize/aggregate_od_route/benders/*.jl` as they exist on `main` right now (after commit
`8d93690`), not from the older design note `notes/2026-07-17_restricted_mw_cut_benders_y.md`, which
predates one of the constraint rows described below.

## 1. The three endpoint-assignment "chain" styles

`NearestOpenAggregateODAssignmentPolicy(feasibility_cut_style)` — `feasibility_cut_style ∈
(:big_m_nearest, :endpoint_chain, :pair_chain)`. This governs how a request `p=(s,o,d)`'s
assignment variable (`x[p,pair]` for BendersY/BendersXY, `h[(o,d),pair]` for BendersYZH) gets tied
to `y`. `:big_m_nearest` and `:endpoint_chain` are "endpoint-nearest" styles
(`_is_endpoint_nearest_style`) that both build a `z` (or `h`-shared) **chain** per physical
endpoint; `:pair_chain` builds no chain at all.

### 1a. `:pair_chain` — no chain, joint pair ranking

No `z` variable. For request `p`, candidate pairs `(j,k)` are ranked by combined walking cost
(`_ranked_request_pairs`, cheapest first, ties broken by `(j,k)` id). Rows (per request, per ranked
pair `r` at rank `rank_r`):

```
x[p,r] <= y[j_r]                                                  -- dual (not part of the MW audit; :pair_chain is unsupported by cut_derivation != :standard)
x[p,r] <= y[k_r]
x[p,r] <= 2 - y[j_r'] - y[k_r']     for every cheaper rank r' < r   -- triangular domination, one row per (r, r') pair
```

No `WALK_ONLY_PAIR`/collision handling (`allow_walk_only=true` is rejected for this style — "no
station-free endpoint-collision representation"). `O(m^2)` rows per request where `m` = number of
feasible pairs (vs. `O(m)` for the endpoint-nearest styles below), since pairs are ranked jointly
rather than per-side. Built by `add_assignment_to_selected_constraints!` +
`add_nearest_open_assignment_constraints!` (compact/master builds) or inlined directly in Benders
subproblem builders' `else` branches (e.g. `y.jl:56-71`).

`cut_derivation ∈ (:zero_completion, :restricted_mw_fixed_pi)` **do not support** this style —
`BendersSolver`/the cut-derivation code throws if requested. Only `:standard` cuts work here.

### 1b/1c. `:big_m_nearest` and `:endpoint_chain` — per-endpoint chain

Both build one `z` chain per **physical endpoint role** `(side, endpoint)` —
`side ∈ (:pickup, :dropoff)` — content-keyed and cached
(`_endpoint_chain_key(side, sorted_candidates, sorted_costs)`) so every request/scenario whose
endpoint has the identical candidate/cost profile shares the *same* `z` variables (not duplicated
per request or scenario). Candidates within `max_walking_distance` are sorted cheapest-first, ties
broken by station id (`_nearest_open_endpoint_candidates`, `_sorted_endpoint_chain`).

Given a chain with sorted stations `station_1 .. station_m` and (tie-break-perturbed) costs
`cost_1 <= ... <= cost_m`, `M_i = cost_m - cost_i`:

| Row | `:endpoint_chain` (`_endpoint_chain_variable!`) | `:big_m_nearest` (`_endpoint_big_m_variable!`) | Dual (in `y_mw_cut.jl`'s completion LP) |
|---|---|---|---|
| (1) simplex | `sum(z) == 1` | `sum(z) == 1` | `lambda[chain]`, free |
| (2) open-only | `z[i] <= y[station_i]`, one row per `i` | same | `mu[chain,i] >= 0` |
| (3) nearest-cost bound | `z[i] <= 1 - y[station_p]` for **every** cheaper `p<i` (triangular, `O(m^2)` rows total) | `selected_cost <= cost_i + M_i*(1-y[station_i])`, one row per `i` (`O(m)` rows, `selected_cost = sum_i' cost_i' z_i'`) | `nu[chain,i] >= 0` (`:big_m_nearest` only — `:endpoint_chain`'s triangular version has no analogue in the completion LP at all; **the completion LP only supports `:big_m_nearest`**, see `_restricted_mw_optimality_cut`'s guard) |
| (4) **nearest-open lower bound** | `z[i] >= y[station_i] - sum(y[station_p] for p<i)`, one row per `i` | **identical formula**, one row per `i` | **MISSING** — no dual variable exists for this row anywhere in `_restricted_mw_completion_lp` (see Section 3) |
| (5) redundant endpoint coverage | `sum(y[station] for station in chain) >= 1` (dropped under `unmet_demand_penalty!==nothing`) | same | none needed — implied by rows (1)+(2), see Section 3 |

Row (4) is **not optional or style-specific dressing** — it is the row that makes `z` actually equal
the nearest *open* candidate. Without it, rows (1)-(3)/(5) alone permit `z` to sit fractional, or to
select a non-nearest open candidate, because nothing lower-bounds `z` against `y`; only the
Big-M/triangular rows (3) upper-bound the *selected cost*, and only once something is already
selected. This was discovered and fixed **on 2026-07-21** (commit `b2905e3`, "Add
unmet_demand_penalty always-feasible mode to AggregateODRouteModel") after `assert_endpoint_chain_near_binary`
caught a fresh `BendersYZ` master returning fractional `z`. It is built **unconditionally** in both
`_endpoint_chain_variable!` and `_endpoint_big_m_variable!` (not gated by `unmet_demand_penalty`),
so every model using either endpoint-nearest style has this row today, regardless of
`unmet_demand_penalty`.

`_add_nearest_open_endpoint_linked_x!`/`_add_endpoint_x_linking!` then links the assignment variable
(`x` or `h`) to its chain's `zp`/`zd`:

```
x[p,(j,k)] <= zp[pickup_rank[j]]                     -- dual rhoO[p,(j,k)] >= 0
x[p,(j,k)] <= zd[dropoff_rank[k]]                     -- dual rhoD[p,(j,k)] >= 0
x[p,(j,k)] >= zp[pickup_rank[j]] + zd[dropoff_rank[k]] - 1   -- dual sigma[p,(j,k)] >= 0
```

plus coverage rows tying `x` to route columns (`pi[p,(j,k)] >= 0`, fixed as a parameter in the
restricted-MW completion LP rather than a free dual).

## 2. Where each decomposition puts `y`, the chain (`z`), assignment (`x`/`h`), and `theta`

| Decomposition | Master variables | Subproblem variables | Chain rows (1)-(5) live in | `cut_derivation` support |
|---|---|---|---|---|
| `BendersY` | `y` (Bin), `theta` | `x`, `z` (chain), `lambda`/`theta`-route | **subproblem** (`_build_nearest_open_y_subproblem_lp` calls `_add_nearest_open_endpoint_linked_x!`, `y` fixed via equality with dual `rho`) | `:standard`, `:zero_completion`, `:restricted_mw_fixed_pi` |
| `BendersXY` | `y` (Bin), `x` (chain-linked via `_add_nearest_open_endpoint_master_x!`), `theta` | `lambda`/route only (`x` fixed via equality) | **master** | `:standard` only (cut_derivation field ignored) |
| `BendersYZ` | `y` (Bin), `z` (chain, via `_add_nearest_open_master_z!`), `theta` | `x`, `lambda`/route (`z` fixed via equality, **no chain rows inside the subproblem at all** — see `yz_mw_cut.jl`'s module docstring) | **master** | `:standard`, `:zero_completion`, `:restricted_mw_fixed_pi` |
| `BendersYZH` | `y` (Bin), `z`+`h` (chain, via `_add_nearest_open_master_h!`, `h` = one var per physical `(o,d)` pair shared across scenarios), `theta` | `lambda`/route only (`h` fixed via equality) | **master** | `:standard`, `:zero_completion` (not `:restricted_mw_fixed_pi` — no free dual block left once `h` is fixed) |

The key structural fact for the audit in Section 3: **`BendersY` is the only decomposition whose
Benders *subproblem* (the thing a restricted-MW/zero-completion cut dualizes) contains the chain
rows (1)-(5) at all.** `BendersXY`/`BendersYZ`/`BendersYZH` all keep the chain in the master and fix
`z`/`h` directly via a single equality constraint in the subproblem — so their completion LPs
(`yz_mw_cut.jl`'s `_yz_completion_lp`; `BendersYZH` skips a completion LP entirely) never need
`lambda`/`mu`/`nu`/anything-for-row-(4)` duals in the first place. `BendersXY` never derives
restricted-MW cuts at all.

## 3. What each cut-derivation completion LP actually dualizes, row by row

For the three files that build a restricted dual-completion LP:

| Row | `y.jl`'s real primal (`_build_nearest_open_y_subproblem_lp`, what `BendersY`'s cut must be dual-feasible for) | `y_mw_cut.jl`'s `_restricted_mw_completion_lp` (`BendersY`'s cut) | `yz_mw_cut.jl`'s `_yz_completion_lp` (`BendersYZ`'s cut) |
|---|---|---|---|
| (1) `sum(z)==1` | present (inside subproblem) | dual `lambda` | n/a — `z` isn't a subproblem variable here |
| (2) `z<=y` | present | dual `mu` | n/a |
| (3) Big-M cost bound | present | dual `nu` | n/a |
| (4) nearest-open lower bound | **present** (unconditional, same builder) | **absent — no dual variable** | n/a |
| (5) redundant coverage | present but redundant (implied by (1)+(2)) | correctly omitted (no dual needed) | n/a |
| `x<=zp`/`x<=zd`/`x>=zp+zd-1` | present | duals `rhoO`/`rhoD`/`sigma` | duals `rhoO`/`rhoD`/`sigma` (identical algebra — `BendersYZ`'s subproblem fixes `z` but still has `x`) |
| coverage `x<=Σ a*theta` | present | `pi_full` fixed as parameter (Sections C/D) | `pi_full` fixed as parameter (same Sections C/D, reused) |
| `y[j]==y_hat[j]` (fixing) | present, dual `rho` — **not used by the completion LP**, which treats `y` as data substituted directly into rows (2)-(4) instead ("parametric elimination") | n/a (no separate `y`-fixing row; `y_hat`/`y_core` are plugged in as numeric parameters) | n/a (`BendersYZ` fixes `z`, not `y`, in its subproblem — different variable entirely, not relevant to this completion LP) |

`yz_mw_cut.jl`'s **core-point** construction (`_yz_joint_core_point`, a different LP from the
completion LP — it builds a relative-interior point of the *master's* structural region, not a
dual) does reproduce row (4) explicitly (`cheaper_sum`/`zvar[idx] >= y[station] - cheaper_sum`,
comment: "nearest-open lower bound"), because that row lives in `BendersYZ`'s *master* and the core
point is a master-polytope point. This is a different LP serving a different purpose than the
completion LP and is not evidence that `_yz_completion_lp` needs (or has) a row-(4) dual — it
correctly doesn't, since row (4) isn't in `BendersYZ`'s *subproblem*.

**Bottom line, stated without yet changing anything**: `y_mw_cut.jl`'s completion LP is missing a
dual variable for a real constraint row that exists, unconditionally, in the exact primal LP it
claims to be dual-feasible for. `yz_mw_cut.jl` has no equivalent gap because its completion LP
dualizes a different (smaller) primal that never contained that row. This matches the memory's
empirical observation that `BendersY` shows many `completion_infeasible` failures and
`BendersYZ`/`BendersYZH` show far fewer/none — it isn't a difference in how thoroughly each was
tested, it's that only `BendersY`'s completion LP had a row to miss.

## 4. Timeline (why this gap exists)

- `93c210d` (2026-07-17): `y_mw_cut.jl` written and audited against `_endpoint_big_m_variable!` as
  it existed then — rows (1)-(3)+(5) only, no row (4).
- `b2905e3` (2026-07-21): row (4) added to both `_endpoint_chain_variable!` and
  `_endpoint_big_m_variable!`, unconditionally, to fix a real fractional-`z` bug. `y_mw_cut.jl` not
  revisited.
- `7395f44` (2026-07-23): `yz_mw_cut.jl` written *after* row (4) existed; its core-point LP
  reproduces it explicitly (since it's building a master-region LP that needs it), but its
  completion LP structurally never needed a row-(4) dual to begin with (see Section 3).
- Present: `y_mw_cut.jl`'s completion LP still has no row-(4) dual. Not yet fixed — this note is the
  documentation checkpoint before that fix.
