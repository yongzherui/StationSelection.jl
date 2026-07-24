# Exceptional fixed-assignment LP/IP-gap cases

*2026-07-24*

This is a desktop-reproduction index for studying the fractional route-covering geometry behind
large AggregateODRoute LP/IP gaps. It consolidates the concrete cases in:

- `2026-07-24_benders_optimal_with_unclosed_gap_grid_reproducer.md`
- `2026-07-23_benders_reports_optimal_with_unclosed_outer_gap.md`
- `2026-06-22_lp_mip_gap_ghost_u.md`

The first two cases hold station selection and nearest-open OD assignments fixed. That removes
Benders cut geometry from the experiment: any remaining gap is the route-column set-covering
polytope itself.

## Case index

| ID | Instance and fixed stations | LP | fixed-assignment IP | Gap | Status |
|---|---|---:|---:|---:|---|
| G10 | `grid_n10_p16_s123`, `[1,2,3,5,6]` | 377.5 | 472.5 | 20.11% of IP | Clean current synthetic reproducer |
| Z10-13 | `zhuzhou_n10_p32_s42`, dump record 13, indices `[3,4,5,7,10]` | 86,461.1733 | 98,904.0704 in saved CG pool; later exhaustive result about 100,888.55 | 12.58% against saved IP; about 14.30% against exhaustive IP | Best saved real-data fixed-assignment artifact |
| Z10-14 | Same instance, dump record 14, indices `[1,3,4,7,10]` | 102,129.7867 | 114,010.3322 | 10.42% of IP | Secondary adjacent saved case |
| Z40-hub | `zz_n40_l20_p8_ov1p0_s123` | 2,480.02 | 3,880.58 | 36.09% in archived CSV | Legacy assignment-model hub example; not a clean fixed-assignment benchmark |

Use G10 first: it needs no external data, is small enough to visualize, and its global station
optimum is independently known. Use Z10-13 next for realistic hub concentration and three
scenarios.

## G10: clean synthetic terminating assignment

### Instance and model

```julia
nx, ny, n_pairs, seed = 2, 5, 16, 123
instance = generate_grid_instance(nx, ny, n_pairs; endpoint_overlap=2.0, seed=seed)
max_walk = 7.0
data = create_grid_problem_data(instance; max_walking_distance=max_walk)
model = AggregateODRouteModel(
    5;
    assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
    route_regularization_weight=10.0,
    walk_cost_weight=0.1,
    repositioning_time=20.0,
    max_walking_distance=max_walk,
    max_wait_time=900.0,
    detour_factor=2.0,
    max_stops=4,
)
open_stations = [1, 2, 3, 5, 6]
```

The fixed nearest-open assignments are:

| `(scenario,o,d)` | assigned `(pickup,dropoff)` |
|---|---|
| `(1,1,6)` | `(1,6)` |
| `(1,1,8)` | `(1,6)` |
| `(1,1,9)` | `(1,5)` |
| `(1,1,10)` | `(1,6)` |
| `(1,2,10)` | `(2,6)` |
| `(1,4,8)` | `(2,6)` |
| `(1,5,1)` | `(5,1)` |
| `(1,5,6)` | `(5,6)` |
| `(1,6,8)` | `(6,6)` |
| `(1,8,1)` | `(6,1)` |
| `(1,8,6)` | `(6,6)` |
| `(1,8,10)` | `(6,6)` |
| `(1,9,8)` | `(5,6)` |
| `(1,10,1)` | `(6,1)` |
| `(1,10,6)` | `(6,6)` |
| `(1,10,8)` | `(6,6)` |

Ignoring same-station pairs, the six distinct required routed pairs are
`(1,6),(1,5),(2,6),(5,1),(5,6),(6,1)`.

The integer solution selects route 7, covering `(1,6),(5,6),(1,5),(2,6)` for cost 240, and route
20, covering `(5,1),(6,1)` for cost 230. Including walking cost 2.5 gives 472.5. The LP pays 377.5
by blending 12+ overlapping columns fractionally. This is the smallest case for plotting the
pair-by-route incidence matrix and comparing the LP support with the two integer routes.

The Benders run can be reproduced through the harness:

```bash
julia --project=. scripts/run_method_compare_task.jl \
  /tmp/g10_exceptional ../Data/base_data \
  grid 10 5 16 123 bendersY_std_reprice_ms4
```

The terminating station set is not the returned incumbent. The run returns incumbent stations
`[6,7,8,9,10]` with objective 462.6, but terminates at `[1,2,3,5,6]`, whose LP/IP values are
377.5/472.5. Reconstruct the assignments above directly when analyzing the fixed subproblem.

## Z10-13: saved realistic fixed assignment

### Authoritative artifact and historical indexing correction

Use:

```text
experiments/2026-07-23_lp_ip_gap_case/yzh_ms4_dump.jls
```

The earlier note calls the investigated case “iteration 14,” but the saved dump record matching
its quoted LP value `86,461.1733` is Julia record `dump[13]`, which also matches CSV iteration 13.
`dump[14]` is a different valid exceptional case with LP `102,129.7867`. Use the numeric LP value
to disambiguate old references.

Z10-13 uses station array indices `[3,4,5,7,10]`. For the generated Zhuzhou station ranking these
correspond to source station IDs `[202,158,138,21,11]` respectively. The scenario-compressed
nearest-open mapping below expands to 96 scenario/OD assignments in the dump; a physical OD pair
has the same assigned station pair in every scenario where it occurs.

| physical `(o,d)` | assigned pair | physical `(o,d)` | assigned pair |
|---|---|---|---|
| `(1,2)` | `(3,4)` | `(1,3)` | `(3,3)` |
| `(1,4)` | `(3,4)` | `(1,5)` | `(3,5)` |
| `(1,6)` | `(3,4)` | `(1,7)` | `(3,7)` |
| `(1,8)` | `(3,10)` | `(1,9)` | `(3,4)` |
| `(1,10)` | `(3,10)` | `(2,1)` | `(4,3)` |
| `(2,3)` | `(4,3)` | `(2,4)` | `(4,4)` |
| `(2,5)` | `(4,5)` | `(2,6)` | `(4,4)` |
| `(2,7)` | `(4,7)` | `(2,8)` | `(4,10)` |
| `(2,9)` | `(4,4)` | `(3,1)` | `(3,3)` |
| `(3,2)` | `(3,4)` | `(3,5)` | `(3,5)` |
| `(3,6)` | `(3,4)` | `(3,7)` | `(3,7)` |
| `(3,8)` | `(3,10)` | `(3,9)` | `(3,4)` |
| `(3,10)` | `(3,10)` | `(4,1)` | `(4,3)` |
| `(4,2)` | `(4,4)` | `(4,3)` | `(4,3)` |
| `(4,7)` | `(4,7)` | `(5,1)` | `(5,3)` |
| `(5,2)` | `(5,4)` | `(5,3)` | `(5,3)` |
| `(5,6)` | `(5,4)` | `(5,8)` | `(5,10)` |
| `(6,1)` | `(4,3)` | `(6,2)` | `(4,4)` |
| `(6,4)` | `(4,4)` | `(6,10)` | `(4,10)` |
| `(7,1)` | `(7,3)` | `(7,2)` | `(7,4)` |
| `(7,3)` | `(7,3)` | `(7,9)` | `(7,4)` |
| `(8,1)` | `(10,3)` | `(8,3)` | `(10,3)` |
| `(8,9)` | `(10,4)` | `(9,1)` | `(4,3)` |
| `(9,3)` | `(4,3)` | `(9,7)` | `(4,7)` |
| `(10,1)` | `(10,3)` | `(10,6)` | `(10,4)` |

Re-run fixed assignments across stop limits with the existing script:

```bash
julia --project=. scripts/analyze_fixed_route_covering_stop_limits.jl \
  experiments/2026-07-23_lp_ip_gap_case/yzh_ms4_dump.jls \
  ../Data/base_data \
  /tmp/z10_exceptional \
  13,14,last
```

The script writes a summary and selected-route table. Julia `Serialization` is version/module
sensitive; use Julia 1.12.x and load `StationSelection` before deserializing if inspecting by hand:

```julia
using StationSelection, Serialization
dump = deserialize("experiments/2026-07-23_lp_ip_gap_case/yzh_ms4_dump.jls")
record = dump[13]
record.assignments
record.generated_columns
record.selected_column_ids
record.v_hat_by_cut
```

## Z40-hub: legacy broad-column exhibit

The older assignment-model case `zz_n40_l20_p8_ov1p0_s123` is useful for visualizing one extreme
hub column, but it is not directly comparable to the current fixed-assignment formulation. Its
archived result reports LP 2480.02 and IP 3880.58 (36.09%). Selected column 638 follows:

```text
(1,27,16,3,18,33,36,38,15,35,5,29)
```

It certifies 36 pairs while only three were described as demand-serving in the original audit.
Artifacts:

```text
experiments/zhuzhou_set_assignment/results/zz_n40_l20_p8_ov1p0_s123.csv
experiments/zhuzhou_set_assignment/selected/zz_n40_l20_p8_ov1p0_s123_selected.csv
experiments/zhuzhou_set_assignment/columns/zz_n40_l20_p8_ov1p0_s123_columns.csv
experiments/zhuzhou_set_assignment/duals/zz_n40_l20_p8_ov1p0_s123_duals.csv
```

The June note quotes older intermediate values (1762/2248, 21.6%); prefer the artifact CSV values
when reproducing. This case also predates later coverage/objective fixes, so use it as a geometric
illustration of broad columns and ghost activations, not as a current correctness benchmark.

## What to export for visualization

For each fixed case, export one row per route and required pair with:

- route ID, stop sequence, route cost, and LP activation;
- integer activation;
- whether the route certifies the required pair;
- coverage dual for the pair;
- route dual credit (`sum` of covered-pair duals);
- reduced cost;
- number of all certified pairs versus number of required assigned pairs certified.

The central plots should be:

1. a binary required-pair-by-route incidence heatmap, ordered by LP support;
2. LP versus IP route activation bars;
3. route cost versus number of required pairs covered;
4. an overlap graph where routes are adjacent when they cover a common required pair;
5. station paths for the LP-support routes, highlighting shared hub subsequences.

The geometric hypothesis to test is that the required-pair incidence matrix contains a dense,
non-balanced overlap pattern: several broad columns each cover different large subsets, fractional
weights cover every row cheaply, but no similarly cheap integral subcollection exists. G10 gives a
six-row matrix small enough to inspect manually; Z10-13 tests whether the same motif repeats at
realistic scale.
