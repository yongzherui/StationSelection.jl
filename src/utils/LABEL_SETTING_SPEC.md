# Label-setting algorithm

Reference for `generate_routes_from_orders` in `generate_routes_from_orders.jl`.

---

# Label-setting BFS for non-temporal route generation

## Goal

Given:

- a set of demand aggregates (unique (origin, destination) pairs in a scenario),
- routing costs between VBS stations,
- vehicle capacity C,
- detour parameters Δ_t (max detour time) and Δ_r (max detour ratio),

generate **all non-dominated feasible vehicle routes** that can serve subsets of orders,
respecting capacity and in-vehicle detour limits.

This is a **route enumeration** problem, not a feasibility test for a fixed assignment.
The output is a pool of routes used as columns in the MILP covering constraint of
`RouteAlphaCapacityModel` and `RouteVehicleCapacityModel`.

---

# 1. Inputs

## Demand aggregates

A set of orders:

[
\mathcal{J} = \{ (o_j, d_j, q_j, \mathcal{V}_j) \mid j = 1, \dots, n \}
]

where:

- (o_j): geographic origin of order j,
- (d_j): geographic destination of order j,
- (q_j): passenger demand (group size),
- (\mathcal{V}_j \subseteq \text{Stations} \times \text{Stations}): feasible VBS pairs for
  order j — each pair (p, d) is a (pickup VBS station, dropoff VBS station) combination
  that lies within walking distance of the order's origin and destination respectively.

## Routing costs

For any two stations a, b:

[
\tau(a, b)
]

denotes the travel time (seconds) from station a to station b.

## Parameters

- (C): vehicle capacity (max total passengers on board at any time),
- (\Delta_t): maximum extra in-vehicle time (seconds) vs direct trip,
- (\Delta_r): maximum detour ratio, i.e. `in_vehicle / direct - 1 ≤ Δ_r`.

---

# 2. Output

[
\text{Vector}\{\texttt{NonTimedRouteData}\}
]

Each `NonTimedRouteData` contains:

- `route::RouteData`: the station sequence and total route travel time,
- `alpha::Dict{(j_idx, k_idx), Int}`: actual passengers carried from VBS station j to
  VBS station k along this route. Used directly in the MILP covering constraint.

Routes with identical `(station_sequence, picked_bitmask)` are deduplicated.

---

# 3. Core idea

A label represents a **partial vehicle route** — the vehicle is at some station after
serving some subset of orders, with some passengers still on board.

From one label, the algorithm tries two classes of actions:

- **Dropoff**: drop off one on-board order at its chosen VBS dropoff station,
- **Pickup**: pick up one not-yet-served order at one of its feasible VBS pickup stations.

Unlike the simulator's label-setting (which tests feasibility for a fixed vehicle and
fixed request set), this algorithm:

1. **Generates all routes** — it does not stop at the first terminal label.
2. **Enumerates VBS choices** — each order has a set of feasible VBS pairs; the BFS
   chooses which pair to use for each order during expansion.
3. Initialises with **one root label per (order × VBS pair)** instead of one root from
   a vehicle's live state.

---

# 4. Label definition

A label (\ell) stores:

[
\ell = (\text{station},\ \text{cum\_time},\ \text{passengers},\ \text{picked},\ \text{dropped},\
       \text{parent},\ \text{board\_cumtime}[\cdot],\ \text{chosen\_pickup}[\cdot],\ \text{chosen\_dropoff}[\cdot])
]

where:

- (\text{station}): the VBS station the vehicle is currently at,
- (\text{cum\_time}): cumulative travel time from the start of the route (seconds),
- (\text{passengers}): total passengers currently on board,
- (\text{picked}): bitmask — bit j set ⟺ order j has been picked up,
- (\text{dropped}): bitmask — bit j set ⟺ order j has been dropped off,
- (\text{parent}): index of the predecessor label (0 = root),
- (\text{board\_cumtime}[j]): cum_time at which order j was picked up (∞ if not yet),
- (\text{chosen\_pickup}[j]): VBS station ID chosen as pickup for order j (0 if not yet),
- (\text{chosen\_dropoff}[j]): VBS station ID chosen as dropoff for order j (0 if not yet).

## Interpretation of `picked` and `dropped`

For order j (1-indexed; bit j-1 in the bitmask):

- bit j-1 clear in `picked`: order j not yet picked up,
- bit j-1 set in `picked`, clear in `dropped`: order j on board,
- bit j-1 set in both `picked` and `dropped`: order j fully served.

The current on-board set is:

[
\text{on\_board} = \text{picked} \;\&\; (\sim\text{dropped})
]

---

# 5. Initial labels

There is **no single root label**. Instead, one root label is created per feasible
(order j, VBS pair (p, d)) combination:

[
\ell_0^{(j,p,d)} = \bigl(p,\ 0,\ q_j,\ \text{bit}_{j},\ 0,\ 0,\
                   \text{board\_cumtime}[\cdot],\ \text{chosen\_pickup}[\cdot],\ \text{chosen\_dropoff}[\cdot]\bigr)
]

where:

- the vehicle starts at pickup station p with cum_time = 0,
- only order j is on board (`picked = bit_j`, `dropped = 0`),
- `board_cumtime[j] = 0`, all others = ∞,
- `chosen_pickup[j] = p`, `chosen_dropoff[j] = d`, all others = 0.

Orders with no feasible VBS pairs produce no root labels and can never be served.

---

# 6. Terminal condition

A label is **terminal** when every picked order has also been dropped off:

[
\text{picked} = \text{dropped} \quad \text{and} \quad \text{passengers} = 0
]

Equivalently, the on-board set is empty and all served orders are complete.

A terminal label with `picked ≠ 0` represents a complete feasible route. It is recorded
into the route pool (see Section 11).

---

# 7. Allowed actions

From a non-terminal label (\ell):

## Dropoff action

Drop off on-board order j at its chosen dropoff station `chosen_dropoff[j]`.

Requires:
- bit j-1 set in `picked` (picked up),
- bit j-1 clear in `dropped` (still on board).

## Pickup action

Pick up order k (not yet picked) at one of its feasible VBS pairs (p_k, d_k) ∈ (\mathcal{V}_k).

Requires:
- bit k-1 clear in `picked`.

No other actions are permitted.

---

# 8. Dropoff feasibility rule

Consider dropping off order j from label (\ell).

## Condition D1: order j is on board

[
(\text{picked} \gg (j-1)) \,\&\, 1 = 1
\quad\text{and}\quad
(\text{dropped} \gg (j-1)) \,\&\, 1 = 0
]

## Condition D2: in-vehicle time constraint

Let the arrival time at the dropoff be:

[
t_j^{\text{drop}} = \text{cum\_time} + \tau(\text{station},\ \text{chosen\_dropoff}[j])
]

Let the direct travel time from the chosen pickup to the chosen dropoff be:

[
\text{direct}_j = \tau(\text{chosen\_pickup}[j],\ \text{chosen\_dropoff}[j])
]

The in-vehicle time is:

[
\text{in\_vehicle}_j = t_j^{\text{drop}} - \text{board\_cumtime}[j]
]

Require:

[
\text{in\_vehicle}_j - \text{direct}_j \;\le\; \Delta_t
]

## Condition D3: detour ratio constraint

[
\text{direct}_j > 0 \implies \frac{\text{in\_vehicle}_j}{\text{direct}_j} \;\le\; 1 + \Delta_r
]

If both D2 and D3 hold, the new label becomes:

[
\ell' = \bigl(\text{chosen\_dropoff}[j],\ t_j^{\text{drop}},\ \text{passengers} - q_j,\
              \text{picked},\ \text{dropped} \,|\, \text{bit}_{j},\ \ell,\
              \text{board\_cumtime},\ \text{chosen\_pickup},\ \text{chosen\_dropoff}\bigr)
]

(board_cumtime, chosen_pickup, chosen_dropoff arrays are shared, not copied, since the
dropoff action does not modify them.)

---

# 9. Pickup feasibility rule

Consider picking up order k at VBS pair (p_k, d_k) from label (\ell).

## Condition P1: order k not yet picked

[
(\text{picked} \gg (k-1)) \,\&\, 1 = 0
]

## Condition P2: vehicle capacity

[
\text{passengers} + q_k \;\le\; C
]

## Condition P3: forward detour look-ahead

Let the arrival time at the new pickup be:

[
t_k^{\text{pick}} = \text{cum\_time} + \tau(\text{station},\ p_k)
]

For every **currently on-board** order j, check that if the vehicle goes directly to j's
dropoff immediately after picking up k, j's detour constraints still hold:

[
\text{min\_in\_vehicle}_j = \bigl(t_k^{\text{pick}} + \tau(p_k,\ \text{chosen\_dropoff}[j])\bigr)
                           - \text{board\_cumtime}[j]
]

Require for all on-board j:

[
\text{min\_in\_vehicle}_j - \text{direct}_j \;\le\; \Delta_t
\quad\text{and}\quad
\bigl(\text{direct}_j > 0 \implies \frac{\text{min\_in\_vehicle}_j}{\text{direct}_j} \le 1 + \Delta_r\bigr)
]

### Interpretation

This is a look-ahead: even in the most favourable case (drop off j immediately next after
picking up k), j's detour must still be feasible. If any on-board order already fails this
test, picking up k is pruned immediately.

Note: there is no pickup-time deadline (no time window). The only gate is capacity and the
forward detour look-ahead.

If P1–P3 all pass, the new label is:

[
\ell' = \bigl(p_k,\ t_k^{\text{pick}},\ \text{passengers} + q_k,\
              \text{picked} \,|\, \text{bit}_{k},\ \text{dropped},\ \ell,\
              \text{board\_cumtime}',\ \text{chosen\_pickup}',\ \text{chosen\_dropoff}'\bigr)
]

where the primed arrays are copies of the parent's arrays with index k updated:

- `board_cumtime'[k] = t_k^{pick}`,
- `chosen_pickup'[k] = p_k`,
- `chosen_dropoff'[k] = d_k`.

---

# 10. Dominance rule

The dominance key is:

[
\text{key}(\ell) = \bigl(\text{station},\ \text{picked},\ \text{dropped},\
                          \text{chosen\_pickup}[\cdot],\ \text{chosen\_dropoff}[\cdot]\bigr)
]

Two labels with the same key are in identical partial-route states (same location, same
served/on-board orders, same VBS assignments made). The better label should be kept.

Label (\ell^1) **dominates** (\ell^2) (same key) iff:

1. (\ell^1) arrives no later:

[
\text{cum\_time}^1 \;\le\; \text{cum\_time}^2
]

2. Every on-board order j has accumulated no more in-vehicle time under (\ell^1):

[
\forall j \in \text{on\_board}:
\quad
\text{cum\_time}^1 - \text{board\_cumtime}^1[j] \;\le\; \text{cum\_time}^2 - \text{board\_cumtime}^2[j]
]

### Rationale

A label that arrives later with more accumulated in-vehicle time for any on-board
passenger is strictly worse: it has less remaining slack on the detour constraints for
every future extension.

### Implementation

A `dom_dict` maps each key to a list of live label indices. When a new child is generated:

1. Check whether any existing live label with the same key dominates the child — if so,
   discard the child.
2. Otherwise, mark as dead any existing live label that the child dominates.
3. Add the child to the list and to the label array.

---

# 11. Multiple-route goal and recording

Unlike the simulator (which stops at the first terminal label), this algorithm **continues
after finding terminal labels** until the label queue is exhausted or the label limit is
reached.

When a terminal label is reached (all picked = all dropped), `_nontimed_record_route!` is
called:

1. Reconstruct the station sequence by following parent pointers from the terminal label
   back to the root; collect `labels[i].station` for each step.
2. Collapse consecutive duplicate stations (same-station stops after a pickup at a
   station that is also a dropoff).
3. Reject the route if any station appears more than once in the final sequence (no
   cycles).
4. Compute total route travel time as the sum of consecutive arc costs.
5. Insert into `routes_map` keyed by `(station_sequence, picked_bitmask)`. If the key
   already exists, the route is a duplicate and is discarded.

All accepted routes are returned as the route pool for the MILP.

---

# 12. Alpha (α) computation

Each `NonTimedRouteData` carries an `alpha` dictionary:

[
\alpha[(j_{\text{idx}},\ k_{\text{idx}})] = \sum_{\substack{j : \text{order } j \text{ served} \\ \text{chosen\_pickup}[j] \to \text{station } j_{\text{idx}} \\ \text{chosen\_dropoff}[j] \to \text{station } k_{\text{idx}}}} q_j
]

That is, for each served order j in the terminal label, look up its chosen pickup station
(mapped to array index j_idx) and its chosen dropoff station (mapped to k_idx), then add
the order's demand q_j to alpha[(j_idx, k_idx)].

## Use in the MILP covering constraint

**RouteAlphaCapacityModel** uses the actual α:

[
\sum_{r:\, (j,k) \in r} \alpha^r_{j,k} \cdot \theta^r_s
\;\ge\;
\sum_{(o,d)} Q_s[(o,d)] \cdot x[s][\text{od}][(j,k)]
\qquad \forall\, (j,k),\, s
]

**RouteVehicleCapacityModel** replaces α with the flat vehicle capacity C:

[
\sum_{r:\, (j,k) \in r} C \cdot \theta^r_s
\;\ge\;
\sum_{(o,d)} Q_s[(o,d)] \cdot x[s][\text{od}][(j,k)]
\qquad \forall\, (j,k),\, s
]

In both cases, θ^r_s = 1 means route r is activated in scenario s.

---

# 13. Data representation

## Bitmasks

Orders are indexed 1…n (n ≤ 63). Bit j-1 (0-indexed) in `picked` / `dropped`
corresponds to order j. All bitmask operations use `UInt64`.

- On-board set: `picked & (~dropped)`.
- Terminal test: `picked == dropped` (with `passengers == 0`).
- Waiting orders: those with bit j-1 clear in `picked`.

## Label arrays

Labels are stored in a flat `Vector{_NonTimedLabel}`. The `parent` field is the
1-based index of the predecessor in this array (0 = root). Route reconstruction follows
these indices backward.

## Dominance dictionary

```text
dom_dict :: Dict{(station, picked, dropped, chosen_pickup_tuple, chosen_dropoff_tuple),
                 Vector{Int}}
```

Maps each dominance key to a list of live label indices with that key.

---

# 14. Main algorithm

## Step 1: initialise root labels

For each order j in 1…n:
  For each feasible VBS pair (p, d) in order j's feasible_vbs set:
    Create a root label at station p, cum_time = 0, passengers = q_j,
    picked = bit_{j-1}, dropped = 0, board_cumtime[j] = 0,
    chosen_pickup[j] = p, chosen_dropoff[j] = d.
  Add root label to label array and dom_dict.

## Step 2: BFS expansion loop

Process labels in insertion order (idx = 1, 2, …):

While idx ≤ length(labels) and idx ≤ max_labels:
  1. Fetch label ℓ = labels[idx]; skip if alive[idx] = false.
  2. **Try all dropoffs** (Stage 1):
     For each order j on board (bit set in picked, clear in dropped):
       Compute arrival time t_drop = cum_time + τ(station, chosen_dropoff[j]).
       Check D2 and D3 (in-vehicle time and ratio constraints).
       If feasible, create child label. Call _nt_push_if_not_dominated!.
       If child was pushed and child.picked == child.dropped: call _nontimed_record_route!.
  3. **Try all pickups** (Stage 2):
     Skip if passengers == 0 (no on-board orders — pickup only follows dropoff or root).
     For each order k not yet picked:
       Check P2 (capacity).
       For each feasible VBS pair (p_k, d_k) for order k:
         Compute arrival arr = cum_time + τ(station, p_k).
         Check P3 (forward look-ahead) for all on-board orders.
         If feasible, create child label. Call _nt_push_if_not_dominated!.

## Step 3: collect and renumber results

Sort accepted routes by their internal ID. Renumber route IDs 1…m sequentially.
Return the vector of NonTimedRouteData.

---

# 15. Full pseudocode

```text
Procedure GenerateRoutesFromOrders(orders, data, capacity, Δ_t, Δ_r)

    # -------------------------------------------------------
    # Step 1: root label initialisation
    # -------------------------------------------------------

    labels   = []
    alive    = []
    dom_dict = {}
    routes_map = {}

    for j in 1..n:
        if orders[j].feasible_vbs is empty: continue
        for (p_id, d_id) in orders[j].feasible_vbs:
            bct    = [Inf, ..., Inf];  bct[j] = 0.0
            cp     = [0, ..., 0];     cp[j]  = p_id
            cd     = [0, ..., 0];     cd[j]  = d_id
            L0 = Label(station=p_id, cum_time=0.0, passengers=q_j,
                       picked=bit_{j-1}, dropped=0, parent=0,
                       board_cumtime=bct, chosen_pickup=cp, chosen_dropoff=cd)
            push labels, L0
            push alive, true
            dom_dict[dom_key(L0)] += [index of L0]

    # -------------------------------------------------------
    # Step 2: BFS expansion
    # -------------------------------------------------------

    idx = 1
    while idx <= len(labels) and idx <= max_labels:

        L = labels[idx];  idx += 1
        if not alive[idx-1]: continue

        # ── Stage 1: dropoffs ──────────────────────────────

        for j in 0..n-1:
            if not (picked_bit(L,j) and not dropped_bit(L,j)): continue

            d_id   = L.chosen_dropoff[j+1]
            p_id   = L.chosen_pickup[j+1]
            t_drop = L.cum_time + τ(L.station, d_id)

            in_veh = t_drop - L.board_cumtime[j+1]
            direct = τ(p_id, d_id)

            if in_veh - direct > Δ_t: continue
            if direct > 0 and in_veh / direct > 1 + Δ_r: continue

            child = Label(station=d_id, cum_time=t_drop,
                          passengers=L.passengers - q_{j+1},
                          picked=L.picked, dropped=L.dropped | bit_j,
                          parent=idx-1,
                          board_cumtime=L.board_cumtime,       # shared
                          chosen_pickup=L.chosen_pickup,        # shared
                          chosen_dropoff=L.chosen_dropoff)      # shared

            pushed = push_if_not_dominated!(child, labels, alive, dom_dict)
            if pushed and child.picked == child.dropped:
                record_route!(labels, len(labels), orders, data, routes_map)

        # ── Stage 2: pickups ───────────────────────────────

        if L.passengers == 0: continue    # no on-board, skip pickups

        for k in 0..n-1:
            if picked_bit(L, k): continue

            if L.passengers + q_{k+1} > capacity: continue

            for (p_id, d_id) in orders[k+1].feasible_vbs:

                arr = L.cum_time + τ(L.station, p_id)

                feasible = true
                for j in 0..n-1:
                    if not (picked_bit(L,j) and not dropped_bit(L,j)): continue
                    j_drop  = L.chosen_dropoff[j+1]
                    j_pick  = L.chosen_pickup[j+1]
                    min_iv  = (arr + τ(p_id, j_drop)) - L.board_cumtime[j+1]
                    direct  = τ(j_pick, j_drop)
                    if min_iv - direct > Δ_t: feasible = false; break
                    if direct > 0 and min_iv / direct > 1 + Δ_r: feasible = false; break
                if not feasible: continue

                new_bct    = copy(L.board_cumtime);  new_bct[k+1]  = arr
                new_cp     = copy(L.chosen_pickup);  new_cp[k+1]   = p_id
                new_cd     = copy(L.chosen_dropoff); new_cd[k+1]   = d_id

                child = Label(station=p_id, cum_time=arr,
                              passengers=L.passengers + q_{k+1},
                              picked=L.picked | bit_k, dropped=L.dropped,
                              parent=idx-1,
                              board_cumtime=new_bct,
                              chosen_pickup=new_cp,
                              chosen_dropoff=new_cd)

                push_if_not_dominated!(child, labels, alive, dom_dict)

    # -------------------------------------------------------
    # Step 3: collect results
    # -------------------------------------------------------

    routes = sort(values(routes_map), by=route.id)
    renumber routes 1..m
    return routes
```

---

# 16. Route reconstruction

When a terminal label is found, the full station sequence is recovered by following
parent pointers backward from the terminal label to the root:

1. Start at terminal label index `terminal_idx`.
2. Repeatedly read `labels[cur].station`, then set `cur = labels[cur].parent`.
3. Stop when `cur == 0` (root has `parent = 0`).
4. Reverse the collected station list.
5. Collapse consecutive duplicate stations.
6. Reject if any station appears more than once (cycle check).

Then compute total travel time and build the alpha dictionary from the terminal label's
`chosen_pickup` and `chosen_dropoff` arrays.

---

# 17. Correctness intuition

The algorithm is correct for completeness because:

1. Every feasible route for any subset of orders corresponds to a sequence of legal pickup
   and dropoff actions starting from one of the root labels.
2. The BFS enumerates all such legal partial action sequences (subject to the label limit).
3. Every extension is checked for local feasibility: capacity, detour constraints (D2/D3
   for dropoffs, P2/P3 for pickups), and the forward look-ahead (P3) prunes pickups that
   would immediately make an on-board order's detour infeasible.
4. Dominance removes only labels that are provably never better than retained labels with
   the same key (same location, same orders served, same VBS assignments). No feasible
   route is pruned by dominance.
5. Terminal labels correspond exactly to complete feasible routes.

---

# 18. Key differences from the simulator's label-setting

| Aspect | Simulator (`label_setting.jl`) | This BFS (`generate_routes_from_orders.jl`) |
|--------|-------------------------------|---------------------------------------------|
| Purpose | Feasibility test for a fixed vehicle and fixed request assignment | Route enumeration for MILP column generation |
| Deadline | Absolute: L_r^{pick}, L_r^{drop} | Relative detour: Δ_t (seconds), Δ_r (ratio) |
| VBS | Fixed node per request (physical stop) | Each order has a set of feasible VBS pairs; chosen during BFS |
| Initialization | Single root from vehicle live state | One root per (order j × VBS pair) |
| Output | One feasible route, then stop | All non-dominated feasible routes |
| Dominance key | (loc, picked_mask, onboard_mask) | (station, picked, dropped, chosen_pickup, chosen_dropoff) — VBS choices included because the same (picked, dropped) state with different VBS choices can yield different future feasibility |
| Dominance criterion | Earliest arrival time | Earliest arrival AND no greater accumulated in-vehicle time per on-board order |
| Pickup rule | P3: arrival ≤ L_r^{pick}; P4: immediate-next-dropoff deadline check | P3: forward detour look-ahead (both Δ_t and Δ_r; no absolute deadline) |

---

# 19. Implementation notes

- Bitmasks are `UInt64`; at most 63 orders are supported per call.
- Labels are stored in a flat `Vector{_NonTimedLabel}`. No priority queue is used (BFS
  processes labels in insertion order, not by priority). This is intentional: pickup
  labels naturally follow dropoff labels for the same parent.
- The `board_cumtime`, `chosen_pickup`, `chosen_dropoff` arrays are **shared** between a
  dropoff child and its parent (since a dropoff action does not modify these arrays).
  Pickup children always get **fresh copies** with the new order's entries filled in.
- `dom_dict` stores a list of live label indices per key, not just the best time, because
  the richer dominance criterion (Section 10) requires comparing all on-board passengers'
  accumulated in-vehicle times — a single scalar per key is insufficient.
- The `max_labels` parameter hard-limits total labels processed; the BFS prints a warning
  and returns the routes found so far if the limit is hit.

---

# 20. Compact description for another agent

> Implement a BFS label-setting DP that generates all feasible vehicle routes for a set
> of demand aggregates (origin-destination groups with feasible VBS pairs). A label stores
> the current station, cumulative travel time, passenger load, picked/dropped bitmasks,
> parent index, the boarding time of each on-board order, and the VBS pickup/dropoff
> station chosen for each order. Initialise one root label per (order, VBS pair)
> combination. From each label, try all dropoffs (checked against max detour time and
> ratio relative to the direct trip) and all pickups (checked for capacity and a
> look-ahead that every on-board order remains detour-feasible if dropped immediately
> next). Apply dominance keyed on (station, picked, dropped, chosen_pickup_tuple,
> chosen_dropoff_tuple): label 1 dominates label 2 iff it arrives no later AND has no
> greater accumulated in-vehicle time for every on-board order. Record every terminal
> label (picked == dropped) as a route, deduplicating by (station_sequence,
> picked_bitmask). Return all accepted routes as a pool for the MILP covering constraint,
> each carrying an alpha dictionary mapping (pickup_station_idx, dropoff_station_idx) to
> actual passenger counts.
