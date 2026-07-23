# DirectSolve route-enumeration audit for AggregateODRouteModel

## Scope and conclusion

This note audits the exhaustive route enumeration used by `DirectSolver` for
`AggregateODRouteModel` under `NearestOpenAggregateODAssignmentPolicy`.

The bounded DFS is exhaustive when the model has an explicit finite `max_stops`, subject to its
documented visit, wait, and detour constraints. As a result of this audit, the DFS now uses the
pricing label transitions directly for pickup eligibility and pair certification, without using
the pricing search's reduced-cost pruning or dominance. The endpoint-only node set is valid for the
production routing-cost pipeline because routing costs have already been replaced by all-pairs
shortest-path costs using Floyd-Warshall.

The audit found a correctness problem in the treatment of `max_stops=nothing`: the model represented
it as `typemax(Int)`, but enumeration silently replaced it with `data.n_stations`. That excluded
feasible routes containing repeated service endpoints and could change the optimum. This is now
fixed when `max_visits_per_node` is finite: both enumeration and pricing use the exact ceiling
`number_of_search_nodes * max_visits_per_node`. If both limits are unbounded, route search rejects
the configuration explicitly because no finite exhaustive DFS depth has been declared.

## How the current DFS works

`enumerate_aggregate_od_route_columns` first builds the aggregate OD map and collects every active
station pair `(j,k)` that requires a vehicle route. It then forms the search-node set from the union
of all origins and destinations in those active pairs.

The DFS starts once at every node in that set. It carries the shared pricing-label state:

- `route`: the station sequence visited so far;
- `time`/`tau`: cumulative arrival time and route cost;
- `station_age`: elapsed ride time for pickup stations that remain eligible;
- `served_pairs`: pairs already certified by a pickup followed by a feasible dropoff;
- `route_length`: used to enforce the stop bound.

Pickup and service state is maintained by the same label transition as column-generation pricing.
A station visit creates or refreshes pickup eligibility only when its cumulative arrival time is at
most `max_wait_time` (with the shared `1e-9` numerical tolerance). A later visit to destination `k`
certifies `(j,k)` only when the remembered pickup at `j` is still within the detour limit:

- arrival at `p` is within `max_wait_time`; and
- `cum_time[q] - cum_time[p]` is at most `detour_factor * routing_cost(j,k)`.

Every resulting label serving at least one pair becomes a provisional column. Its covered-pair set
is the column incidence signature and its `tau` is the total time from the first route node to the
last.

The DFS asks the shared pricing candidate-extension function for useful next endpoints. That
function permits a fresh pickup only if arrival at its origin will be within `max_wait_time`; after
the pickup cutoff, it permits only destinations reachable from a previously eligible pickup. An
extension is also rejected when the next station has reached `max_visits_per_node`. Recursion stops
when the route reaches the resolved stop bound. There is no dominance or reduced-cost pruning in
this enumerator: uniform positive synthetic rewards merely keep every active pair visible to the
shared transition code.

After the complete traversal, singleton seed columns are appended and columns are deduplicated by
covered-pair signature, retaining the smallest-`tau` column for each signature. This final
deduplication is valid: the master model observes a route only through its covered pairs and its
objective coefficient `route_regularization_weight * (tau + repositioning_time)`, so a more
expensive column with identical incidence is dominated.

## Why restricting the search to active endpoints is valid

`read_routing_costs_from_segments` computes Floyd-Warshall all-pairs shortest paths before route
enumeration. Therefore `routing_cost(i,j)` already includes the best possible path through any
number of physical intermediate stations.

Suppose an enumerated service sequence contains a station `v` that is not an origin or destination
of any active pair. Removing `v` and replacing the two legs `(i,v),(v,j)` by the precomputed direct
cost `(i,j)` cannot increase travel time:

```text
routing_cost(i,j) <= routing_cost(i,v) + routing_cost(v,j).
```

The removal does not remove any boarding or alighting event because `v` is not an active endpoint.
It can only weakly improve cumulative waiting times, ride times, detour feasibility, and total
`tau`. Consequently an optimal column always has an equivalent or cheaper representation using
active endpoints only.

This argument depends on the routing-cost contract: enumeration must receive the Floyd-Warshall
closure (or another all-pairs shortest-path metric), not arbitrary raw edge costs. That contract is
satisfied by the production segment-data loader.

## The completeness bug and fix

The constructor maps `max_stops=nothing` to `typemax(Int)`, meaning that the user did not impose a
finite stop limit. The enumerator previously executed:

```julia
max_stops = base_model.max_stops == typemax(Int) ? data.n_stations : base_model.max_stops
```

This is an additional elementary-route assumption, but the model does not require routes to be
elementary. In fact, `max_visits_per_node` defaults to `typemax(Int)` and the DFS itself deliberately
allows repeated nodes up to that separate limit.

Floyd-Warshall does not justify the `n_stations` bound. It proves that non-service transit nodes can
be shortcut, but repeated active endpoints may represent distinct service events and cannot always
be removed without changing the covered-pair set.

The smallest counterexample has two stations and two active directional pairs:

```text
active pairs = {(1,2), (2,1)}
route        = [1,2,1]
```

The route covers both directional pairs. With `max_stops=nothing`, it is allowed by the declared
model semantics whenever station 1 may be visited twice. The enumeration nevertheless resolves
`max_stops` to `data.n_stations == 2`, so it never generates this three-stop route.

Separate routes `[1,2]` and `[2,1]` have the same combined travel time in the symmetric unit-cost
case, but they pay `repositioning_time` twice. The omitted pooled route pays it once. Thus the
omission can strictly increase the DirectSolve objective; it is not merely a difference in route
metadata.

The fix uses the same resolver in DirectSolve enumeration and the column-generation pricing-data
builder. With `N` search nodes and a finite visit limit `V`, no route can contain more than `N*V`
stops, so this is an exact ceiling rather than a heuristic. Checked multiplication prevents silent
integer overflow. When both `max_stops` and `max_visits_per_node` are unbounded, the resolver throws
a clear `ArgumentError`; silently selecting another finite heuristic would again make an incomplete
search look exhaustive.

## Secondary observability issue

`max_enumerated_routes` is checked against provisional serving prefixes before deduplication, while
the result metadata reports the number of deduplicated columns. A run can explore or reject far
more prefixes than `metadata["enumerated_routes"]` suggests. This does not silently change an
otherwise completed result—the limit raises an error—but the metadata should not be interpreted as
the number of physical routes explored.

## Correctness boundary going forward

- Explicit finite `max_stops`: the current DFS is exhaustive over active-endpoint sequences up to
  that bound.
- `max_stops=nothing`, finite `max_visits_per_node`: exhaustive up to the exact product bound.
- Both stop and per-node visit limits unbounded: rejected explicitly as an infinite search space.
- Endpoint-only search: valid under the Floyd-Warshall/all-pairs-shortest-path routing-cost contract.
- Cheapest-column-per-covered-set deduplication: valid for the current master formulation.

DirectSolve enumeration therefore requires at least one finite structural route bound: either an
explicit `max_stops`, or a finite `max_visits_per_node` from which the exact product bound can be
derived.

## Exhaustiveness evidence and runtime expectations

The production DFS is tested against an independent test-only brute-force oracle that enumerates
every endpoint sequence, certifies service by scanning all pickup/dropoff position pairs, and keeps
the minimum `tau` for each coverage signature. The comparison covers several pickup cutoffs,
multiple stop bounds, and repeated visits. This avoids using agreement with column generation as
the sole correctness check now that enumeration intentionally shares its label transitions.

The DFS is exhaustive for the minimum-cost column of every realizable coverage signature under the
finite bounds and shortest-path routing-cost contract above. It does not retain every physical
sequence: feasibility-aware candidate generation omits stops that cannot create a fresh eligible
pickup or complete a live pickup, and final signature deduplication removes dominated routes. Those
omissions are lossless under the all-pairs shortest-path metric.

Worst-case runtime remains exponential in the resolved route-depth bound. The shared candidate
transition normally reduces branching substantially once the pickup cutoff or detour limits bind,
and incremental pair certification avoids the old full route rescan at every prefix. In a fully
permissive instance, label allocation can add constant-factor overhead and the search can still be
large. `max_routes` and the time limit fail loudly rather than returning a partially enumerated
result, so any returned DirectSolve enumeration has completed its declared finite search.
