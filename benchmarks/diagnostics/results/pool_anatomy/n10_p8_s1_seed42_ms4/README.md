# Pool and cut anatomy — n10_p8_s1_seed42_ms4

Zhuzhou n=10, p=8, s=1, seed 42, k=5, max_stops=4.
Walking weight 0.1, driving weight 10.0.
At this `max_stops` the enumerated pool IS the whole column universe, so `columns.csv`
is exhaustive rather than a sample.

Two arms, both solving the identical model at the same incumbents:
- `plain` — `:column_generation`, prices over every station.
- `activated` — `:column_generation_activated`, prices over the built stations only and
  repairs the duals with the closed-form completion. **No row generation.**

## columns.csv — one row per (anchor, scenario, column), universe-wide

| field | meaning |
| --- | --- |
| `anchor_id`, `anchor_stations` | the incumbent, `\|`-joined station ids |
| `route` | visit sequence, `>`-joined |
| `n_stops`, `n_pax` | route length, number of assignments |
| `assignments` | `p:j:k` triples, `\|`-joined — group, pickup station, dropoff station |
| `tau`, `f_c` | driving time, and total column cost `β(τ+ρ) + w·Σ demand·walk` |
| `assign_stations` | every station used by an assignment |
| `unbuilt_assign_stations`, `n_unbuilt_assign` | those outside the incumbent |
| `n_unbuilt_visits`, `visits_unbuilt` | unbuilt stations on the ROUTE, served or not |
| `in_plain`, `in_activated` | whether each arm's pool contains this column |
| `rc_plain`, `rc_activated` | reduced cost at each arm's duals; the dual constraint is `rc >= 0` |
| `excess_over_built` | `Σα − Σ(γ at built stations) − f_c` at the **plain** duals |

`n_unbuilt_assign > 0` means `theta` is pinned to 0 at this incumbent, so the column cannot
improve the objective — it exists only for its dual constraint. `visits_unbuilt` with
`n_unbuilt_assign == 0` is the class the restricted search cannot reach at all, and the
reason the soundness proof compares against the route with the stop DELETED.

Group by `unbuilt_assign_stations` and take `max(excess_over_built)` to get what those
stations' worths must cover between them. For single-station groups that is the per-station
requirement; `station_need.csv` has it pre-aggregated.

## station_need.csv — per unbuilt station

`need_j` (from single-unbuilt-station columns, exhaustive) against `closed_form_j` (what the
activated arm charges) and `plain_cg_j` (what full pricing's duals carry), plus the column
attaining `need_j` in full. `n_cols_multi_pax_at_j` counts columns serving 2+ passengers at
`j` — those constrain a SUM of worths, not one coordinate, so they are excluded from `need_j`
and are the reason a per-station number is a lower bound on the true requirement.

## duals_alpha.csv / duals_gamma.csv / cuts.csv

`alpha` per group with its `c_walk_bound` (`NaN` where the group has no direct-walk option,
which is exactly where `alpha` has no a-priori ceiling). `gamma` per `(group, station, side)`
with `station_built`. `cuts.csv` is the aggregate the master actually sees:
`Θ_s ≥ cut_constant − Σ_j gamma_total·y_j`.

## anchors.csv, pool_not_in_universe.csv

Anchors with their Q rank and per-scenario costs. The second file should be EMPTY — a row in
it means some pooled column is absent from the enumerated universe, i.e. the two are not the
same model and every comparison here is void. Rows written this run: 386.

