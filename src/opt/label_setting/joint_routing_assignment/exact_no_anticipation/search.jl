# `:exact_no_anticipation` search: data, rules, seeding, extension, bound, dominance, replay
# and the label-setting hooks. See `types.jl` for the timing rules.

const _NA_EMPTY = Int[]

@inline _na_travel(pd::NoAnticipationPricingData, u::Int, v::Int) =
    u == v ? 0.0 : _joint_routing_assignment_travel(pd.base, u, v)

"""
A lower bound on the time to get from `u` to `v` along any path, for the tests that prune clocks, bound the remaining
reward or block dominance (never for the rules that credit a rider). The v2 drive times break the triangle inequality
by up to 0.001 s on 126 of 6,972 station pairs (rounding), so the direct drive is not quite a lower bound; 0.01 s of
slack keeps those tests on the safe side.
"""
const NO_ANTICIPATION_TRI_TOL = 0.01
@inline _na_travel_lb(pd::NoAnticipationPricingData, u::Int, v::Int) =
    max(_na_travel(pd, u, v) - NO_ANTICIPATION_TRI_TOL, 0.0)

# ── data ──────────────────────────────────────────────────────────────────────
"""
    create_no_anticipation_pricing_data(base, data, mapping, scenario, detour_factor, first_boarding,
                                        max_pickup_time = 900.0, pickup_window = :route)

Attach each opportunity's walk to its pickup stop, its arrival there (`ready`, from its request time), its G1 and G2
limits and its in-vehicle limit. `base` is the `exact` pricer's pricing data for the same candidates.
"""
function create_no_anticipation_pricing_data(
    base::JointRoutingAssignmentPricingData,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    scenario::Int,
    detour_factor::Float64,
    first_boarding::Symbol,
    max_pickup_time::Float64=900.0,
    pickup_window::Symbol=:route,
)::NoAnticipationPricingData
    omega = mapping.Omega_s[scenario]
    ratio = mapping.door_to_door_ratio
    opps = NoAnticipationOpportunity[]
    for o in base.opportunities
        od_o, od_d = omega[o.p]
        r = aggregate_od_route_request_time(mapping, scenario, o.p)
        first_boarding === :start_now && r > 0 && throw(ArgumentError(
            "first_boarding = :start_now boards first-stop riders at t = 0; it cannot take request times (group $(o.p) requests at $(r) s)"))
        walk_in = get_walking_cost(data, od_o, o.origin)
        egress = get_walking_cost(data, o.destination, od_d)
        direct = get_routing_cost(data, od_o, od_d)
        deadline = (isfinite(ratio) && isfinite(direct)) ? r + ratio * direct - egress : Inf
        cap = detour_factor * get_routing_cost(data, o.origin, o.destination)
        push!(opps, NoAnticipationOpportunity(o.p, o.origin, o.destination, cap, deadline, walk_in, r + walk_in,
                                              r + max_pickup_time, o.reward, o.layer_mask))
    end
    by_origin = Dict{Int, Vector{Int}}()
    by_destination = Dict{Int, Vector{Int}}()
    for (i, o) in enumerate(opps)
        push!(get!(() -> Int[], by_origin, o.origin), i)
        push!(get!(() -> Int[], by_destination, o.destination), i)
    end
    for v in values(by_origin)
        sort!(v; by = i -> (opps[i].ready, i))
    end
    horizon = isempty(opps) ? max_pickup_time : maximum(o.latest for o in opps)
    return NoAnticipationPricingData(base, first_boarding in (:after_first_walker, :after_last_walker),
                                     first_boarding === :after_last_walker, max_pickup_time, pickup_window === :route,
                                     horizon, opps, by_origin, by_destination, sort!(collect(keys(by_origin))))
end

"Latest time a pickup clock may open: the route window after leaving the first stop (`:route`), capped by the last G1."
@inline _na_window_end(pd::NoAnticipationPricingData, t0::Float64) =
    pd.route_window ? min(t0 + pd.base.max_wait_time, pd.horizon) : pd.horizon

# ── the rules ─────────────────────────────────────────────────────────────────
"Is `o` at its stop by `v` and is its G1 still open?"
@inline _na_admits(o::NoAnticipationOpportunity, v::Float64) =
    o.ready <= v + NO_ANTICIPATION_EPS && v <= o.latest + NO_ANTICIPATION_EPS

"Can a later-visit clock opened at `v`, of age `age` (ride so far), credit `o` on arriving at `o.destination` at `T`?"
@inline _na_later_ok(o::NoAnticipationOpportunity, v::Float64, age::Float64, T::Float64) =
    _na_admits(o, v) && age <= o.ride_cap + 1e-9 && T <= o.deadline + 1e-9

"First-stop admission with departure `t0`: M3 ready <= t0, M2 ready in (t0 - 5 s, t0], both with G1 open; M1 everyone."
@inline _na_root_admits(pd::NoAnticipationPricingData, t0::Float64, ready::Float64, latest::Float64) =
    !pd.first_walker || (ready <= t0 + NO_ANTICIPATION_EPS && t0 <= latest + NO_ANTICIPATION_EPS &&
                         (pd.gather || ready > t0 - NO_ANTICIPATION_FIRST_WALKER_TOL - NO_ANTICIPATION_EPS))

"Can the first stop's clock (vehicle left at `t0`; ride `age`) credit `o` at drop-off time `T`?"
@inline function _na_first_ok(pd::NoAnticipationPricingData, o::NoAnticipationOpportunity, t0::Float64, age::Float64, T::Float64)
    age <= o.ride_cap + 1e-9 || return false
    pd.first_walker && return _na_root_admits(pd, t0, o.ready, o.latest) && T <= o.deadline + 1e-9
    return T <= o.deadline - o.walk_in + 1e-9          # M1: the `exact` G2 (walk counted, no wait; every r_p = 0)
end

"""
Could a later-visit clock at `j` (visit time `v`, age `age`, now `T`, vehicle at `cur`) still credit anyone? `vnewer`
is the next visit of `j` after `v` (Inf if none): it admits every rider `v` does whose G1 is still open then, with a
shorter ride, so `v` is needed only for riders with `latest < vnewer`.
"""
function _na_later_useful(pd::NoAnticipationPricingData, j::Int, v::Float64, age::Float64, T::Float64, cur::Int,
                          vnewer::Float64=Inf)
    for i in get(pd.by_origin, j, _NA_EMPTY)
        o = pd.opps[i]
        o.ready <= v + NO_ANTICIPATION_EPS || break               # sorted by ready: nobody later has arrived either
        o.latest + NO_ANTICIPATION_EPS < vnewer || continue
        t = _na_travel_lb(pd, cur, o.destination)
        _na_later_ok(o, v, age + t, T + t) && return true
    end
    return false
end

function _na_first_useful(pd::NoAnticipationPricingData, j::Int, t0::Float64, age::Float64, T::Float64, cur::Int)
    for i in get(pd.by_origin, j, _NA_EMPTY)
        o = pd.opps[i]
        t = _na_travel_lb(pd, cur, o.destination)
        _na_first_ok(pd, o, t0, age + t, T + t) && return true
    end
    return false
end

"Keep the useful clocks among `ages` (youngest first) of visits to `j`; now `T`, vehicle at `cur`."
function _na_prune_clocks(pd::NoAnticipationPricingData, j::Int, ages::Vector{Float64}, T::Float64, cur::Int)
    kept = Float64[]
    vnewer = Inf
    for a in ages
        v = T - a
        _na_later_useful(pd, j, v, a, T, cur, vnewer) && push!(kept, a)
        vnewer = v
    end
    return kept
end

# ── seeding ───────────────────────────────────────────────────────────────────
function _na_initial_labels(pd::NoAnticipationPricingData)::Vector{NoAnticipationLabel}
    base = pd.base
    rc0 = base.route_regularization_weight * base.repositioning_time
    labels = NoAnticipationLabel[]
    for j in pd.origins
        starts = pd.first_walker ?
            unique(pd.opps[i].ready for i in pd.by_origin[j] if pd.opps[i].ready <= pd.opps[i].latest + 1e-9) : [0.0]
        for t0 in starts
            root_age = _na_first_useful(pd, j, t0, 0.0, t0, j) ? 0.0 : Inf
            push!(labels, NoAnticipationLabel(j, [j], t0, t0, root_age, Dict{Int, Vector{Float64}}(), RewardLayerBitset(),
                                              0.0, rc0, 1))
        end
    end
    return labels
end

# ── candidate next stops ──────────────────────────────────────────────────────
function _na_candidate_next_nodes(label::NoAnticipationLabel, pd::NoAnticipationPricingData)::Vector{Int}
    T, t0, cur, root = label.time, label.start_time, label.current, label.route[1]
    act = label.activated_reward_layers
    cand = Set{Int}()
    # pickups: a stop reached inside the window where some rider (unpaid layer) is waiting, G1 open, by then
    wend = _na_window_end(pd, t0)
    if T <= wend + 1e-9
        for j in pd.origins
            j == cur && continue
            arr = T + _na_travel(pd, cur, j)
            arr <= wend + 1e-9 || continue
            for i in pd.by_origin[j]
                o = pd.opps[i]
                o.ready <= arr + NO_ANTICIPATION_EPS || break
                arr <= o.latest + NO_ANTICIPATION_EPS || continue
                _has_inactive_layer(o.layer_mask, act) || continue
                d = _na_travel_lb(pd, j, o.destination)
                (d <= o.ride_cap + 1e-9 && arr + d <= o.deadline + 1e-9) || continue
                push!(cand, j); break
            end
        end
    end
    # drop-offs reachable in time from a live clock
    for (j, ages) in label.station_age, age in ages
        v = T - age
        for i in pd.by_origin[j]
            o = pd.opps[i]
            o.ready <= v + NO_ANTICIPATION_EPS || break
            k = o.destination
            (k == cur || k in cand) && continue
            _has_inactive_layer(o.layer_mask, act) || continue
            t = _na_travel_lb(pd, cur, k)
            _na_later_ok(o, v, age + t, T + t) && push!(cand, k)
        end
    end
    if isfinite(label.root_age)
        for i in get(pd.by_origin, root, _NA_EMPTY)
            o = pd.opps[i]
            k = o.destination
            (k == cur || k in cand) && continue
            _has_inactive_layer(o.layer_mask, act) || continue
            t = _na_travel_lb(pd, cur, k)
            _na_first_ok(pd, o, t0, label.root_age + t, T + t) && push!(cand, k)
        end
    end
    return sort!(collect(cand))
end

# ── extension ─────────────────────────────────────────────────────────────────
function _na_extend(label::NoAnticipationLabel, next::Int, pd::NoAnticipationPricingData)::NoAnticipationLabel
    base = pd.base
    travel = _na_travel(pd, label.current, next)
    T = label.time + travel
    t0 = label.start_time
    root = label.route[1]
    # credit riders dropped off at `next`
    reachable = RewardLayerBitset()
    for i in get(pd.by_destination, next, _NA_EMPTY)
        o = pd.opps[i]
        ok = false
        for a in get(label.station_age, o.origin, Float64[])
            age = a + travel
            _na_later_ok(o, T - age, age, T) && (ok = true; break)
        end
        if !ok && o.origin == root && isfinite(label.root_age)
            ok = _na_first_ok(pd, o, t0, label.root_age + travel, T)
        end
        ok && union!(reachable, o.layer_mask)
    end
    new_layers = setdiff(reachable, label.activated_reward_layers)
    reward = isempty(new_layers) ? 0.0 : _sum_layer_weights(base, new_layers)
    layers = isempty(new_layers) ? label.activated_reward_layers : union(label.activated_reward_layers, new_layers)
    # age the clocks; a visit inside the window opens a fresh clock at `next`
    clocks = Dict{Int, Vector{Float64}}()
    for (j, ages) in label.station_age
        aged = ages .+ travel
        j == next && T <= _na_window_end(pd, t0) + 1e-9 && pushfirst!(aged, 0.0)
        kept = _na_prune_clocks(pd, j, aged, T, next)
        isempty(kept) || (clocks[j] = kept)
    end
    if !haskey(label.station_age, next) && T <= _na_window_end(pd, t0) + 1e-9
        _na_later_useful(pd, next, T, 0.0, T, next) && (clocks[next] = [0.0])
    end
    root_age = Inf
    if isfinite(label.root_age)
        ra = label.root_age + travel
        _na_first_useful(pd, root, t0, ra, T, next) && (root_age = ra)
    end
    return NoAnticipationLabel(next, vcat(label.route, next), T, t0, root_age, clocks, layers,
                               label.tau + travel,
                               label.reduced_cost + base.route_regularization_weight * travel - reward,
                               label.route_length + 1)
end

# ── remaining-reward bound (frontier priority / pruning) ──────────────────────
"""
`NA_BOUND[]` (experiments, 2026-11-18): `:plain` -- the weight of every unpaid layer some completion could still credit;
`:travel` -- that weight net of the driving needed to collect it (`_na_remaining_bound`). Both are admissible.
"""
const NA_BOUND = Ref(:plain)

"""
Admissible bound on the reward a completion can still collect, net (`:travel`) of the driving it must pay for.

Every collectable opportunity `o` -- from a live clock (its own rules, earliest possible drop-off), the first-stop
clock, or a fresh clock at a stop reachable inside the pickup window (a visit no earlier than the vehicle can get
there and the rider is there, before the rider's G1 closes; earliest drop-off from it) -- gets a minimum remaining
drive c_o: drive(cur, k) for a rider on a live clock; max(drive(cur, j), ready - T) + drive(j, k) for a fresh one.
The `ready - T` term holds because the vehicle never waits after its first stop, so every second until it reaches j
at time >= ready is driven. A completion collecting a set S drives at least max over S of c_o, so its reduced cost is
at least rc - max over c of [sum of free weight of the passengers with some c_o <= c - beta * c] (one sorted sweep,
each passenger counted at its best free weight so far). `:plain` drops the driving term (c = 0 for all).
"""
function _na_remaining_bound(label::NoAnticipationLabel, pd::NoAnticipationPricingData)::Float64
    base = pd.base
    T, t0, cur, root = label.time, label.start_time, label.current, label.route[1]
    act = label.activated_reward_layers
    ev = Tuple{Float64, Int, Float64}[]               # (min remaining drive, passenger, free weight)
    function add!(o, c)
        w = 0.0
        for l in o.layer_mask
            l in act || (w += base.layer_weight[l])
        end
        w > 0.0 && push!(ev, (c, o.p, w))
    end
    for (j, ages) in label.station_age, age in ages
        v = T - age
        for i in pd.by_origin[j]
            o = pd.opps[i]
            o.ready <= v + NO_ANTICIPATION_EPS || break
            _has_inactive_layer(o.layer_mask, act) || continue
            t = _na_travel_lb(pd, cur, o.destination)
            _na_later_ok(o, v, age + t, T + t) && add!(o, t)
        end
    end
    if isfinite(label.root_age)
        for i in get(pd.by_origin, root, _NA_EMPTY)
            o = pd.opps[i]
            _has_inactive_layer(o.layer_mask, act) || continue
            t = _na_travel_lb(pd, cur, o.destination)
            _na_first_ok(pd, o, t0, label.root_age + t, T + t) && add!(o, t)
        end
    end
    wend = _na_window_end(pd, t0)
    if T <= wend + 1e-9
        for j in pd.origins
            dj = _na_travel_lb(pd, cur, j)
            arr = T + dj
            arr <= wend + 1e-9 || continue
            for i in pd.by_origin[j]
                o = pd.opps[i]
                o.ready <= wend + NO_ANTICIPATION_EPS || break
                _has_inactive_layer(o.layer_mask, act) || continue
                v = max(arr, o.ready)
                v <= o.latest + NO_ANTICIPATION_EPS || continue
                d = _na_travel_lb(pd, j, o.destination)
                (d <= o.ride_cap + 1e-9 && v + d <= o.deadline + 1e-9) && add!(o, (v - T) + d)
            end
        end
    end
    isempty(ev) && return 0.0
    beta = NA_BOUND[] === :travel ? base.route_regularization_weight : 0.0
    sort!(ev; by = first)
    best = 0.0; total = 0.0
    bestw = Dict{Int, Float64}()
    for (c, p, w) in ev
        prev = get(bestw, p, 0.0)
        if w > prev
            total += w - prev; bestw[p] = w
        end
        best = max(best, total - beta * max(c - NO_ANTICIPATION_TRI_TOL, 0.0))
    end
    return best
end

# ── dominance ─────────────────────────────────────────────────────────────────
function _na_make_bitsets(label::NoAnticipationLabel, pd::NoAnticipationPricingData)::NoAnticipationBitsets
    act = label.activated_reward_layers
    T, cur = label.time, label.current
    stations = sort!(collect(keys(label.station_age)))
    ages = Vector{Float64}[label.station_age[j] for j in stations]
    riders = Vector{NTuple{3, Float64}}[]
    for j in stations
        rs = NTuple{3, Float64}[]
        for i in pd.by_origin[j]
            o = pd.opps[i]
            _has_inactive_layer(o.layer_mask, act) || continue
            t = _na_travel_lb(pd, cur, o.destination)
            T + t <= o.deadline + 1e-9 || continue
            push!(rs, (o.ready, o.latest, o.ride_cap - t))
        end
        push!(riders, rs)
    end
    fl = -Inf; fr = Inf; fd = Inf
    for j in pd.origins
        dj = _na_travel_lb(pd, cur, j)
        reach = T + dj
        for i in pd.by_origin[j]
            o = pd.opps[i]
            o.ready > reach + NO_ANTICIPATION_EPS || continue
            o.ready <= o.latest + NO_ANTICIPATION_EPS || continue
            _has_inactive_layer(o.layer_mask, act) || continue
            djk = _na_travel_lb(pd, j, o.destination)
            (djk <= o.ride_cap + 1e-9 && o.ready + djk <= o.deadline + 1e-9) || continue
            fl = max(fl, o.latest - dj); fr = min(fr, o.ready); fd = min(fd, dj)
        end
    end
    rr = NTuple{2, Float64}[]
    if isfinite(label.root_age)
        for i in get(pd.by_origin, label.route[1], _NA_EMPTY)
            o = pd.opps[i]
            _has_inactive_layer(o.layer_mask, act) || continue
            t = _na_travel_lb(pd, cur, o.destination)
            (label.root_age + t <= o.ride_cap + 1e-9 && T + t <= o.deadline + 1e-9) || continue
            push!(rr, (o.ready, o.latest))
        end
    end
    return NoAnticipationBitsets(stations, ages, riders, fl, fr, fd, rr)
end

"Does every unpaid, deliverable first-stop rider that departure `tb` admits also get admitted by `ta`?"
function _na_root_covers(pd::NoAnticipationPricingData, root_riders::Vector{NTuple{2, Float64}}, ta::Float64, tb::Float64)
    abs(ta - tb) <= 1e-9 && return true
    for (r, l) in root_riders
        _na_root_admits(pd, tb, r, l) && !_na_root_admits(pd, ta, r, l) && return false
    end
    return true
end

"""
`a` dominates `b` (same current stop) when every completion of `b` is matched by `a` at no
worse reduced cost. On top of the `exact` conditions (no later, compensated reward), the walking rules need:
  - the pickup window: a has no less of it left;
  - b's first-stop clock (if live): a has the same first stop with a live clock no older that
    admits every first-stop rider b's admits, among those a has not paid and could still deliver;
  - every live later-visit clock of b at stop j: a has clocks at j, and every rider b's clock admits (ready by its
    visit, G1 still open), that a has not paid and whose G2 and ride limit b could still meet, is admitted by some
    clock of a at j no older than b's;
  - stops visited later: a, being earlier, reaches them earlier on b's path and may find a rider not yet there. A
    rider a cannot take even arriving directly (ready > time + drive) is a risk only if b could reach that stop
    with the rider's G1 and b's window still open; safe if b is too late for all of them (time + drive > latest),
    or every such rider is ready only after b's window, or b cannot reach any of them inside its window.
Riders whose layers a has already paid are covered by the compensation term, as in `exact`.
"""
function _na_dominates_hard(pd::NoAnticipationPricingData, x::PricingLabelEntry, y::PricingLabelEntry)::Symbol
    a = x.label; b = y.label
    A = x.bitsets; B = y.bitsets
    a.time <= b.time + 1e-9 || return :later
    wa = _na_window_end(pd, a.start_time); wb = _na_window_end(pd, b.start_time)
    wa - a.time >= wb - b.time - 1e-9 || return :window                   # a has no less pickup window left
    (!pd.base.bounded_max_stops || a.route_length <= b.route_length) || return :stops
    budget = b.reduced_cost - a.reduced_cost + 1e-9
    budget >= 0.0 || return :budget
    if isfinite(b.root_age)
        (a.route[1] == b.route[1] && isfinite(a.root_age) && a.root_age <= b.root_age + 1e-9) || return :root
        _na_root_covers(pd, A.root_riders, a.start_time, b.start_time) || return :root
    end
    ia = 1
    for (ib, j) in enumerate(B.stations)
        while ia <= length(A.stations) && A.stations[ia] < j
            ia += 1
        end
        (ia <= length(A.stations) && A.stations[ia] == j) || return :clock
        aages = A.ages[ia]
        for agb in B.ages[ib]
            vb = b.time - agb
            # fast path: a has a clock no older at the same visit time
            any(aga -> aga <= agb + 1e-9 && abs((a.time - aga) - vb) <= 1e-9, aages) && continue
            for (r, l, slack) in A.riders[ia]
                (r <= vb + NO_ANTICIPATION_EPS && vb <= l + NO_ANTICIPATION_EPS && agb <= slack + 1e-9) || continue
                covered = false
                for aga in aages
                    va = a.time - aga
                    if aga <= agb + 1e-9 && r <= va + NO_ANTICIPATION_EPS && va <= l + NO_ANTICIPATION_EPS
                        covered = true; break
                    end
                end
                covered || return :clock
            end
        end
    end
    (a.time >= b.time - 1e-9 || b.time > A.future_latest + 1e-9 || A.future_min_ready > wb + NO_ANTICIPATION_EPS ||
     b.time + A.future_min_d > wb + 1e-9) || return :future
    _joint_routing_assignment_compensation(
        a.activated_reward_layers, b.activated_reward_layers, pd.base.layer_weight, budget,
        pd.base.compensated_dominance,
    ) <= budget || return :compensation
    return :ok
end


# ── dominance, second rule: lost-reward compensation (2026-11-18) ─────────────
"""
Profiling and rule switch for `:exact_no_anticipation` dominance (experiments; not a config field yet).
`NA_DOM_RULE[]`: `:hard` -- the rule above only; `:loss` -- when it fails, try `_na_dominates_loss`.
`NA_PROFILE[]`: count each comparison's outcome in `NA_DOM_STATS` (the hard rule's first failing condition, and
whether the loss rule then succeeded).
"""
const NA_DOM_RULE = Ref(:hard)
const NA_PROFILE = Ref(false)
const NA_DOM_STATS = Dict{Symbol, Int}()
@inline _na_count!(k::Symbol) = (NA_DOM_STATS[k] = get(NA_DOM_STATS, k, 0) + 1; nothing)

"Weight of `o`'s layers neither label has paid (masks are per-passenger prefixes, so a passenger's loss is its max)."
function _na_free_weight(pd::NoAnticipationPricingData, o::NoAnticipationOpportunity, act_a, act_b)::Float64
    w = 0.0
    for l in o.layer_mask
        (l in act_a || l in act_b) && continue
        w += pd.base.layer_weight[l]
    end
    return w
end

"""
`a` dominates `b` (same stop) by lost-reward compensation. Every reward layer `b` can still collect on a completion
sigma is collected by `a` on sigma, or already paid by `a` (`w(A_a ∖ A_b)`, as in `exact`), or lost: a layer of an
opportunity `b` could credit on some sigma that `a` cannot on the same sigma. With Lost an over-approximation,

    rc_a + w(A_a ∖ A_b) + w(Lost ∖ (A_a ∪ A_b)) <= rc_b

makes every completion of `a` no worse (both pay the same travel). No ordering of the two times is needed: an earlier
`a` loses riders not yet at their stop, a later one loses riders whose G1, window or G2 closes in between. Lost is
built per opportunity `o` (unpaid by `a`), over the three ways `b` can credit it:
  (i)  a live later-visit clock of `b` at `o.origin` (visit vb, age agb) that admits `o` and could deliver it
       (agb + drive <= ride cap, T_b + drive <= deadline). Covered if `a` has a clock there no older that admits
       `o`, and `a` is no later (or its latest drop-off, T_a + ride cap - age, still meets the deadline).
  (ii) `b`'s first-stop clock: covered if `a` has the same first stop, a live first-stop clock no older that admits
       `o` at its own departure, and the same lateness test.
  (iii) a fresh visit of `o.origin` after a path of length L >= d = drive(cur, origin): `b` admits and can still
       deliver for L in I_b = [max(d, ready - T_b), min(latest, wend_b, deadline - drive(j, k)) - T_b]. `a` follows
       sigma minus the stops that are not candidates for it, so it reaches j at some time in [T_a + d, T_a + L].
       Covered if `o` is at j by T_a + d and every L in I_b keeps T_a + L within o's G1 and a's window (and, if
       `a` is later, L <= deadline - cap - T_a, so its drop-off meets G2 whatever the ride).
Lower-bound drives make I_b larger, never smaller, so Lost only grows. Hard conditions kept: max_stops, rc_a <= rc_b.
"""
function _na_dominates_loss(pd::NoAnticipationPricingData, x::PricingLabelEntry, y::PricingLabelEntry)::Bool
    a = x.label; b = y.label
    (!pd.base.bounded_max_stops || a.route_length <= b.route_length) || return false
    budget = b.reduced_cost - a.reduced_cost + 1e-9
    budget >= 0.0 || return false
    act_a = a.activated_reward_layers; act_b = b.activated_reward_layers
    comp = _joint_routing_assignment_compensation(act_a, act_b, pd.base.layer_weight, budget,
                                                  pd.base.compensated_dominance)
    comp <= budget || return false
    room = budget - comp
    lost = Dict{Int, Float64}()
    total = Ref(0.0)
    function lose!(o)
        w = _na_free_weight(pd, o, act_a, act_b)
        w > 0.0 || return true
        prev = get(lost, o.p, 0.0)
        if w > prev
            total[] += w - prev; lost[o.p] = w
            total[] > room && return false
        end
        return true
    end
    Ta, Tb, cur = a.time, b.time, a.current
    later = Ta > Tb + 1e-9
    wa = _na_window_end(pd, a.start_time); wb = _na_window_end(pd, b.start_time)
    # (i) b's later-visit clocks
    for (j, agbs) in b.station_age
        aages = get(a.station_age, j, Float64[])
        for agb in agbs
            vb = Tb - agb
            for i in pd.by_origin[j]
                o = pd.opps[i]
                o.ready <= vb + NO_ANTICIPATION_EPS || break
                vb <= o.latest + NO_ANTICIPATION_EPS || continue
                _has_inactive_layer(o.layer_mask, act_a) || continue
                t = _na_travel_lb(pd, cur, o.destination)
                (agb + t <= o.ride_cap + 1e-9 && Tb + t <= o.deadline + 1e-9) || continue
                covered = false
                for aga in aages
                    if aga <= agb + 1e-9 && _na_admits(o, Ta - aga) &&
                            (!later || Ta + o.ride_cap - aga <= o.deadline + 1e-9)
                        covered = true; break
                    end
                end
                covered || lose!(o) || return false
            end
        end
    end
    # (ii) b's first-stop clock
    if isfinite(b.root_age)
        same_root = a.route[1] == b.route[1] && isfinite(a.root_age) && a.root_age <= b.root_age + 1e-9
        for i in get(pd.by_origin, b.route[1], _NA_EMPTY)
            o = pd.opps[i]
            _has_inactive_layer(o.layer_mask, act_a) || continue
            t = _na_travel_lb(pd, cur, o.destination)
            _na_first_ok(pd, o, b.start_time, b.root_age + t, Tb + t) || continue
            de = pd.first_walker ? o.deadline : o.deadline - o.walk_in
            covered = same_root && _na_root_admits(pd, a.start_time, o.ready, o.latest) &&
                      (!later || Ta + o.ride_cap - a.root_age <= de + 1e-9)
            covered || lose!(o) || return false
        end
    end
    # (iii) fresh visits
    for j in pd.origins
        d = _na_travel_lb(pd, cur, j)
        for i in pd.by_origin[j]
            o = pd.opps[i]
            _has_inactive_layer(o.layer_mask, act_a) || continue
            djk = _na_travel_lb(pd, j, o.destination)
            djk <= o.ride_cap + 1e-9 || continue
            lo_b = max(d, o.ready - Tb)
            hi_b = min(o.latest, wb, o.deadline - djk) - Tb
            lo_b <= hi_b + 1e-9 || continue
            # `a` may have to skip a stop of sigma that is a candidate for `b` but not for itself, and then reaches
            # j earlier than T_a + L -- as early as T_a + d. So `a` covers `o` only if `o` is already at j then.
            # (-5c, 2026-11-18: comparing with T_a + L instead missed exactly these riders when `a` is later.)
            hi_a = min(o.latest, wa) - Ta
            later && (hi_a = min(hi_a, o.deadline - o.ride_cap - Ta))
            (o.ready <= Ta + d + NO_ANTICIPATION_EPS && hi_b <= hi_a + 1e-9) || lose!(o) || return false
        end
    end
    return true
end

function _na_dominates(pd::NoAnticipationPricingData, x::PricingLabelEntry, y::PricingLabelEntry)::Bool
    r = _na_dominates_hard(pd, x, y)
    if r === :ok
        NA_PROFILE[] && _na_count!(:ok)
        return true
    end
    NA_PROFILE[] && _na_count!(r)
    NA_DOM_RULE[] === :loss || return false
    ok = _na_dominates_loss(pd, x, y)
    NA_PROFILE[] && _na_count!(ok ? Symbol("loss_ok_", r) : Symbol("loss_no_", r))
    return ok
end

# ── replay of a finished route ────────────────────────────────────────────────
"""
Replay `route` from t0 under the same rules and return each rider's best credited assignment
`p => (j, k, reward, pickup_position, dropoff_position)` (ties as in `exact`'s replay). A later-visit rider boards at
the latest visit of j that admits them (shortest ride).
"""
function _na_replay(route::Vector{Int}, t0::Float64, pd::NoAnticipationPricingData)
    best = Dict{Int, Tuple{Int, Int, Float64, Int, Int}}()
    isempty(route) && return best
    root = route[1]
    T = t0
    visits = Dict{Int, Vector{Tuple{Float64, Int}}}()       # in-window visits (time, position), oldest first
    for idx in 2:length(route)
        node = route[idx]
        T += _na_travel(pd, route[idx - 1], node)
        for i in get(pd.by_destination, node, _NA_EMPTY)
            o = pd.opps[i]
            pos = 0
            for (v, vpos) in Iterators.reverse(get(visits, o.origin, Tuple{Float64, Int}[]))
                _na_later_ok(o, v, T - v, T) && (pos = vpos; break)
            end
            if pos == 0 && o.origin == root && _na_first_ok(pd, o, t0, T - t0, T)
                pos = 1
            end
            pos == 0 && continue
            cur = get(best, o.p, nothing)
            if isnothing(cur) || o.reward > cur[3] + 1e-9 ||
                    (abs(o.reward - cur[3]) <= 1e-9 && (o.origin, o.destination) < (cur[1], cur[2]))
                best[o.p] = (o.origin, o.destination, o.reward, pos, idx)
            end
        end
        if T <= _na_window_end(pd, t0) + 1e-9
            push!(get!(() -> Tuple{Float64, Int}[], visits, node), (T, idx))
        end
    end
    return best
end

function _na_column_from_route(route::Vector{Int}, t0::Float64, pd::NoAnticipationPricingData;
                               label_reduced_cost::Union{Float64, Nothing}=nothing)
    best = _na_replay(route, t0, pd)
    assignments = Tuple{Int, Int, Int}[(p, v[1], v[2]) for (p, v) in best]
    positions = Dict{Int, Tuple{Int, Int}}(p => (v[4], v[5]) for (p, v) in best)
    reward_sum = sum((v[3] for (_p, v) in best); init=0.0)
    tau = length(route) < 2 ? 0.0 : sum(_na_travel(pd, route[i], route[i + 1]) for i in 1:(length(route) - 1))
    base = pd.base
    reduced_cost = base.route_regularization_weight * (tau + base.repositioning_time) - reward_sum
    if !isnothing(label_reduced_cost)
        @assert isapprox(reduced_cost, label_reduced_cost; atol=1e-6) (
            "no-anticipation replay reduced cost $(reduced_cost) != searched label's $(label_reduced_cost) " *
            "for route $(route), t0 $(t0)")
    end
    return assignments, tau, reduced_cost, positions
end

# ── context + hooks ───────────────────────────────────────────────────────────
function NoAnticipationSearchContext(pd::NoAnticipationPricingData)
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) = _na_dominates(pd, x, y)
    return NoAnticipationSearchContext(pd, dominates)
end

_pricing_initial_labels(ctx::NoAnticipationSearchContext) = _na_initial_labels(ctx.pricing_data)
_pricing_make_bitsets(ctx::NoAnticipationSearchContext, label::NoAnticipationLabel) = _na_make_bitsets(label, ctx.pricing_data)
"""
`NA_STATE[]` (experiments, 2026-11-18): the dominance bucket. `:node` -- labels at the same stop are compared (the
original); `:keyed` -- also the set of stops with a live later-visit clock and the first stop if its clock is live, so
labels that differ there are never compared. Skipping comparisons is always sound (fewer dominances, never a wrong
one); it loses only dominances where `a`'s clock stops strictly contain `b`'s.
"""
const NA_STATE = Ref(:node)

function _pricing_state(::NoAnticipationSearchContext, label::NoAnticipationLabel, ::NoAnticipationBitsets)
    NA_STATE[] === :keyed || return (label.current, UInt64(0))
    h = hash(isfinite(label.root_age) ? label.route[1] : 0)
    for j in sort!(collect(keys(label.station_age)))
        h = hash(j, h)
    end
    return (label.current, h)
end
_pricing_label_priority(ctx::NoAnticipationSearchContext, label::NoAnticipationLabel, ::NoAnticipationBitsets)::Float64 =
    label.reduced_cost - _na_remaining_bound(label, ctx.pricing_data)
_pricing_best_signature(::NoAnticipationSearchContext, label::NoAnticipationLabel) =
    isempty(label.activated_reward_layers) ? nothing : label.activated_reward_layers
_pricing_route_length(::NoAnticipationSearchContext, label::NoAnticipationLabel) = label.route_length
_pricing_max_route_length(ctx::NoAnticipationSearchContext) = ctx.pricing_data.base.max_stops
_pricing_candidate_next_nodes(ctx::NoAnticipationSearchContext, label::NoAnticipationLabel) =
    _na_candidate_next_nodes(label, ctx.pricing_data)
_pricing_extend_label(ctx::NoAnticipationSearchContext, label::NoAnticipationLabel, next_node::Int) =
    _na_extend(label, next_node, ctx.pricing_data)
_pricing_dominates_fn(ctx::NoAnticipationSearchContext) = ctx.dominates

function _pricing_candidate_from_label(ctx::NoAnticipationSearchContext, label::NoAnticipationLabel)
    assignments, tau, reduced_cost, positions = _na_column_from_route(
        label.route, label.start_time, ctx.pricing_data; label_reduced_cost=label.reduced_cost)
    isempty(assignments) && return nothing
    return (signature=_joint_routing_assignment_column_signature(assignments), tau=tau, reduced_cost=reduced_cost,
            payload=(route=label.route, assignments=assignments, positions=positions, start_time=label.start_time))
end

_pricing_pool_signature(::NoAnticipationSearchContext, existing::JointRoutingAssignmentRouteColumn) =
    _joint_routing_assignment_column_signature(existing)

_pricing_make_column(ctx::NoAnticipationSearchContext, column_id::Int, candidate) =
    JointRoutingAssignmentRouteColumn(
        column_id, candidate.payload.route, candidate.payload.assignments, candidate.tau;
        metadata=Dict{String, Any}(
            "scenario" => ctx.pricing_data.base.scenario,
            "route" => Tuple(candidate.payload.route),
            "reduced_cost" => candidate.reduced_cost,
            "assignment_positions" => candidate.payload.positions,
            # when the vehicle leaves the route's first stop (0 under first_boarding = :start_now)
            "start_time" => candidate.payload.start_time,
        ),
    )

function _pricing_verify_column(::NoAnticipationSearchContext, column::JointRoutingAssignmentRouteColumn, m::JuMP.Model, mapping, duals)
    alpha, gamma_o, gamma_d = duals
    data = m[:joint_routing_assignment_data]
    return _verify_joint_routing_assignment_master_reduced_cost(column, m, data, mapping, alpha, gamma_o, gamma_d)
end

# ── master seed columns under first_boarding = :after_first_walker ────────────
"""
Two-stop seed columns when the vehicle waits at its first stop (M2; with `gather = true`, M3 -- a bundle per distinct
t0 carries every rider with walk <= t0 who still makes their deadline). M2:
a bundle on `[j, k]` carries only riders who reach `j` within 5 s of the bundle's last walker
(t0 = that walk) and still make their door-to-door deadline from t0; plus one singleton per
rider, so every per-group assignment stays representable (see `seeding.jl`). A singleton
boards at t0 = its own walk and meets G2 because the pair filter kept walk + drive + walk
within budget.
"""
function joint_routing_assignment_two_stop_seed_columns_first_walker(
    data::StationSelectionData,
    mapping::AggregateODRouteMap;
    next_column_id::Int=1,
    max_pickup_time::Float64=900.0,
    gather::Bool=false,
)::Vector{JointRoutingAssignmentRouteColumn}
    ratio = mapping.door_to_door_ratio
    by_route = Dict{Tuple{Int, Int, Int}, Vector{NTuple{4, Float64}}}()   # (ready, deadline, p, latest)
    for s in 1:n_scenarios(data)
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            mapping.Q_s[s][p] > 0 || continue
            r = aggregate_od_route_request_time(mapping, s, p)
            for pair in get_valid_jk_pairs(mapping, o, d)
                is_walk_only_pair(pair) && continue
                j, k = pair
                isfinite(get_routing_cost(data, j, k)) || continue
                direct = get_routing_cost(data, o, d)
                dl = (isfinite(ratio) && isfinite(direct)) ? r + ratio * direct - get_walking_cost(data, k, d) : Inf
                w = get_walking_cost(data, o, j)
                w <= max_pickup_time + 1e-9 || continue           # G1: cannot board in time
                push!(get!(by_route, (s, j, k), NTuple{4, Float64}[]), (r + w, dl, Float64(p), r + max_pickup_time))
            end
        end
    end
    columns = JointRoutingAssignmentRouteColumn[]
    id = next_column_id
    mk(s, j, k, t0, ps, seed) = JointRoutingAssignmentRouteColumn(
        id, [j, k], [(p, j, k) for p in sort(ps)], get_routing_cost(data, j, k);
        metadata=Dict{String, Any}("scenario" => s, "seed" => seed, "start_time" => t0,
                                   "assignment_positions" => Dict{Int, Tuple{Int, Int}}(p => (1, 2) for p in ps)))
    for key in sort!(collect(keys(by_route)))
        s, j, k = key
        tau = get_routing_cost(data, j, k)
        riders = sort(by_route[key]; rev = true)            # longest walk first
        n = length(riders)
        if gather
            # M3: one bundle per distinct t0 = some rider's walk, carrying every rider who has arrived by t0 and
            # still makes their deadline from there.
            for t0 in unique(r[1] for r in riders)
                ps = [Int(r[3]) for r in riders if r[1] <= t0 + NO_ANTICIPATION_EPS && t0 <= r[4] + NO_ANTICIPATION_EPS &&
                                                   t0 + tau <= r[2] + 1e-9]
                length(ps) > 1 && (push!(columns, mk(s, j, k, t0, ps, "two_stop_gathered")); id += 1)
            end
        end
        start = gather ? n + 1 : 1
        while start <= n
            t0 = riders[start][1]
            stop = start
            while stop < n && riders[stop + 1][1] > t0 - NO_ANTICIPATION_FIRST_WALKER_TOL - NO_ANTICIPATION_EPS
                stop += 1
            end
            ps = [Int(r[3]) for r in riders[start:stop] if t0 + tau <= r[2] + 1e-9 && t0 <= r[4] + NO_ANTICIPATION_EPS]
            if length(ps) > 1
                push!(columns, mk(s, j, k, t0, ps, "two_stop_first_walker")); id += 1
            end
            start = stop + 1
        end
        for (w, _dl, p, _l) in riders
            push!(columns, mk(s, j, k, w, [Int(p)], "two_stop_singleton")); id += 1
        end
    end
    return columns
end
