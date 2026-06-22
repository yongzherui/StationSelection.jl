# LP-MIP Gap: Hub Routes and Ghost u-Activations
*2026-06-22 03:41 EDT*

## Observation

In `CompatibilitySetAssignmentModel` (and `CompatibilitySetModel`), column generation
selects broad "hub" routes that certify far more station pairs `(j,k)` than are ever
actually used for demand assignment. This drives a large LP-MIP gap (~21% on n=40 instances).

**Concrete example** — n=40, l=20, p=8, ov=1.0, seed=123 (`zhuzhou_set_assignment`):
- Final MIP selection: 5 singleton routes + column 638
- Col 638: route `(1→27→16→3→18→33→36→38→15→35→5→29)`, 12 stops, τ=1051s
- Certifies 36 station pairs; only **3 are actually assigned demand** in the MIP
- LP bound: 1762, IP objective: 2248, **gap = 21.6%**

## Root Cause: Ghost u-Activations

The coverage constraint is:

```
sum_r theta[r,s] * I[r certifies (j,k)] == u[j,k,s]      # assignment model
```

The demand activation constraint is:

```
x[od, j, k] <= demand * u[j,k,s]                          # ONE-SIDED upper bound
```

When route 638 is selected (`theta[638,s] = 1`), the equality forces `u[j,k,s] = 1`
for **all 36** certified pairs. But for 33 of those pairs, either:

- **No demand pair has (j,k) as a valid station pair** — no activation constraint
  is added at all, so u[j,k,s]=1 sits unconstrained and unused.
- **Some demand pair could use (j,k) but is routed elsewhere** — the constraint
  `x[od,j,k] <= demand * u[j,k,s]` is trivially satisfied with x=0, u=1.

These are **ghost u-activations**: u variables forced to 1 by the selected route
but driving zero demand coverage. The `==` in the coverage constraint does NOT force
`x[od,j,k] == demand * u[j,k,s]` — it only forces route coverage to equal u.
Demand assignment is bounded *above* by u, not forced to equal it.

## Why the Gap Persists Under Set Assignment

The LP can fractionally combine hub routes (e.g., 1/3 × route_A + 2/3 × route_B)
to synthetically cover all demand pairs at low average τ. The MIP must activate each
route at integer weight 1, paying the full τ for every station on the route regardless
of which certified pairs actually serve demand.

A 12-stop route paying τ=1051s for 3 demand pairs would be replaced, in an ideal
solution, by a 5–6 stop targeted route paying τ≈350–500s for those same 3 pairs.
The CG pricer never generates that short route because it scores poorly on LP dual
credit (it certifies fewer pairs → lower reduced cost).

## Active Experiments

| Experiment | μ (route reg. weight) | Model | Status |
|---|---|---|---|
| `zhuzhou_set_assignment` | 1.0 | Assignment | Complete (24 jobs) |
| `zz_large_rrw` | 100.0 | Assignment | Running |

Hypothesis: μ=100 penalises long routes heavily enough that the CG pricer generates
shorter, targeted routes — tightening the LP-MIP gap.

## Potential Fix

Post-convergence targeted pricing: after CG converges on the LP, run a secondary
pricing pass restricted to `(j,k)` pairs with non-zero LP x-values. This would
generate short routes covering exactly the subsets the MIP needs, and add them to
the pool before solving the final MIP.
