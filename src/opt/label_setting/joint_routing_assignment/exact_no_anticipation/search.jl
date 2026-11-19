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
    create_no_anticipation_pricing_data(base, data, mapping, scenario, detour_factor, first_boarding)

Attach each opportunity's walk to its pickup stop, its G2 deadline from the request and its
in-vehicle limit. `base` is the `exact` pricer's pricing data for the same candidates.
"""
function create_no_anticipation_pricing_data(
    base::JointRoutingAssignmentPricingData,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    scenario::Int,
    detour_factor::Float64,
    first_boarding::Symbol,
)::NoAnticipationPricingData
    omega = mapping.Omega_s[scenario]
    ratio = mapping.door_to_door_ratio
    opps = NoAnticipationOpportunity[]
    for o in base.opportunities
        od_o, od_d = omega[o.p]
        walk_in = get_walking_cost(data, od_o, o.origin)
        egress = get_walking_cost(data, o.destination, od_d)
        direct = get_routing_cost(data, od_o, od_d)
        deadline = (isfinite(ratio) && isfinite(direct)) ? ratio * direct - egress : Inf
        cap = detour_factor * get_routing_cost(data, o.origin, o.destination)
        push!(opps, NoAnticipationOpportunity(o.p, o.origin, o.destination, cap, deadline, walk_in, o.reward, o.layer_mask))
    end
    by_origin = Dict{Int, Vector{Int}}()
    by_destination = Dict{Int, Vector{Int}}()
    for (i, o) in enumerate(opps)
        push!(get!(() -> Int[], by_origin, o.origin), i)
        push!(get!(() -> Int[], by_destination, o.destination), i)
    end
    for v in values(by_origin)
        sort!(v; by = i -> (opps[i].walk_in, i))
    end
    return NoAnticipationPricingData(base, first_boarding in (:after_first_walker, :after_last_walker),
                                     first_boarding === :after_last_walker, opps, by_origin, by_destination,
                                     sort!(collect(keys(by_origin))))
end

"The pickup deadline (`pickup_deadline_s`), the same for every rider since all request at t = 0: no pickup clock opens later."
@inline _na_pickup_deadline(pd::NoAnticipationPricingData) = pd.base.pickup_deadline_s

# ── the rules ─────────────────────────────────────────────────────────────────
"Can a later-visit clock opened at `v`, of age `age` (ride so far), credit `o` on arriving at `o.destination` at `T`?"
@inline _na_later_ok(o::NoAnticipationOpportunity, v::Float64, age::Float64, T::Float64) =
    o.walk_in <= v + NO_ANTICIPATION_EPS && age <= o.ride_cap + 1e-9 && T <= o.deadline + 1e-9

"Can the first stop's clock (vehicle left at `t0`; ride `age`) credit `o` at drop-off time `T`?"
@inline function _na_first_ok(pd::NoAnticipationPricingData, o::NoAnticipationOpportunity, t0::Float64, age::Float64, T::Float64)
    age <= o.ride_cap + 1e-9 || return false
    if pd.first_walker
        return o.walk_in <= t0 + NO_ANTICIPATION_EPS &&
               (pd.gather || o.walk_in > t0 - NO_ANTICIPATION_FIRST_WALKER_TOL - NO_ANTICIPATION_EPS) &&
               T <= o.deadline + 1e-9
    end
    return T <= o.deadline - o.walk_in + 1e-9          # M1: the `exact` G2 (walk counted, no wait)
end

"Could a later-visit clock at `j` (visit time `v`, age `age`, now `T`, vehicle at `cur`) still credit anyone?"
function _na_later_useful(pd::NoAnticipationPricingData, j::Int, v::Float64, age::Float64, T::Float64, cur::Int)
    for i in get(pd.by_origin, j, _NA_EMPTY)
        o = pd.opps[i]
        o.walk_in <= v + NO_ANTICIPATION_EPS || break               # sorted by walk: nobody later has arrived either
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

# ── seeding ───────────────────────────────────────────────────────────────────
function _na_initial_labels(pd::NoAnticipationPricingData)::Vector{NoAnticipationLabel}
    base = pd.base
    rc0 = base.route_regularization_weight * base.repositioning_time
    labels = NoAnticipationLabel[]
    for j in pd.origins
        starts = pd.first_walker ?
            unique(pd.opps[i].walk_in for i in pd.by_origin[j] if pd.opps[i].walk_in <= pd.base.pickup_deadline_s + 1e-9) : [0.0]
        for t0 in starts
            root_age = _na_first_useful(pd, j, t0, 0.0, t0, j) ? 0.0 : Inf
            push!(labels, NoAnticipationLabel(j, [j], t0, t0, root_age, Dict{Int, Float64}(), RewardLayerBitset(),
                                              0.0, rc0, 1))
        end
    end
    return labels
end

# ── candidate next stops ──────────────────────────────────────────────────────
function _na_candidate_next_nodes(label::NoAnticipationLabel, pd::NoAnticipationPricingData)::Vector{Int}
    base = pd.base
    T, t0, cur, root = label.time, label.start_time, label.current, label.route[1]
    act = label.activated_reward_layers
    cand = Set{Int}()
    # pickups: a stop reached by the pickup deadline where some rider (unpaid layer) has arrived by then
    deadline = _na_pickup_deadline(pd)
    if T <= deadline + 1e-9
        for j in pd.origins
            j == cur && continue
            arr = T + _na_travel(pd, cur, j)
            arr <= deadline + 1e-9 || continue
            for i in pd.by_origin[j]
                o = pd.opps[i]
                o.walk_in <= arr + NO_ANTICIPATION_EPS || break
                _has_inactive_layer(o.layer_mask, act) || continue
                d = _na_travel_lb(pd, j, o.destination)
                (d <= o.ride_cap + 1e-9 && arr + d <= o.deadline + 1e-9) || continue
                push!(cand, j); break
            end
        end
    end
    # drop-offs reachable in time from a live clock
    for (j, age) in label.station_age
        v = T - age
        for i in pd.by_origin[j]
            o = pd.opps[i]
            o.walk_in <= v + NO_ANTICIPATION_EPS || break
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
        a = get(label.station_age, o.origin, Inf)
        if isfinite(a)
            age = a + travel
            ok = _na_later_ok(o, T - age, age, T)
        end
        if !ok && o.origin == root && isfinite(label.root_age)
            ok = _na_first_ok(pd, o, t0, label.root_age + travel, T)
        end
        ok && union!(reachable, o.layer_mask)
    end
    new_layers = setdiff(reachable, label.activated_reward_layers)
    reward = isempty(new_layers) ? 0.0 : _sum_layer_weights(base, new_layers)
    layers = isempty(new_layers) ? label.activated_reward_layers : union(label.activated_reward_layers, new_layers)
    # age the clocks; a visit by the pickup deadline opens a fresh clock at `next`
    ages = Dict{Int, Float64}()
    for (j, a) in label.station_age
        j == next && continue
        age = a + travel
        _na_later_useful(pd, j, T - age, age, T, next) && (ages[j] = age)
    end
    if T <= _na_pickup_deadline(pd) + 1e-9
        _na_later_useful(pd, next, T, 0.0, T, next) && (ages[next] = 0.0)
    elseif haskey(label.station_age, next)
        age = label.station_age[next] + travel
        _na_later_useful(pd, next, T - age, age, T, next) && (ages[next] = age)
    end
    root_age = Inf
    if isfinite(label.root_age)
        ra = label.root_age + travel
        _na_first_useful(pd, root, t0, ra, T, next) && (root_age = ra)
    end
    return NoAnticipationLabel(next, vcat(label.route, next), T, t0, root_age, ages, layers,
                               label.tau + travel,
                               label.reduced_cost + base.route_regularization_weight * travel - reward,
                               label.route_length + 1)
end

# ── remaining-reward bound (frontier priority / pruning) ──────────────────────
"""
Admissible: the weight of every unpaid layer some completion could still credit -- from a live
clock (its own rules, earliest possible drop-off), or from a fresh clock at any stop reachable
by the pickup deadline (a rider who has arrived by then, earliest drop-off).
"""
function _na_remaining_bound(label::NoAnticipationLabel, pd::NoAnticipationPricingData)::Float64
    base = pd.base
    T, t0, cur, root = label.time, label.start_time, label.current, label.route[1]
    act = label.activated_reward_layers
    seen = RewardLayerBitset()
    total = 0.0
    add!(o) = for l in o.layer_mask
        (l in act || l in seen) && continue
        push!(seen, l); total += base.layer_weight[l]
    end
    for (j, age) in label.station_age
        v = T - age
        for i in pd.by_origin[j]
            o = pd.opps[i]
            o.walk_in <= v + NO_ANTICIPATION_EPS || break
            _has_inactive_layer(o.layer_mask, act) || continue
            t = _na_travel_lb(pd, cur, o.destination)
            _na_later_ok(o, v, age + t, T + t) && add!(o)
        end
    end
    if isfinite(label.root_age)
        for i in get(pd.by_origin, root, _NA_EMPTY)
            o = pd.opps[i]
            _has_inactive_layer(o.layer_mask, act) || continue
            t = _na_travel_lb(pd, cur, o.destination)
            _na_first_ok(pd, o, t0, label.root_age + t, T + t) && add!(o)
        end
    end
    latest = _na_pickup_deadline(pd)
    if T <= latest + 1e-9
        for j in pd.origins
            arr = T + _na_travel_lb(pd, cur, j)
            arr <= latest + 1e-9 || continue
            for i in pd.by_origin[j]
                o = pd.opps[i]
                o.walk_in <= latest + NO_ANTICIPATION_EPS || break
                _has_inactive_layer(o.layer_mask, act) || continue
                d = _na_travel_lb(pd, j, o.destination)
                (d <= o.ride_cap + 1e-9 && arr + d <= o.deadline + 1e-9) && add!(o)
            end
        end
    end
    return total
end

# ── dominance ─────────────────────────────────────────────────────────────────
function _na_make_bitsets(label::NoAnticipationLabel, pd::NoAnticipationPricingData)::NoAnticipationBitsets
    act = label.activated_reward_layers
    T, cur = label.time, label.current
    stations = sort!(collect(keys(label.station_age)))
    ages = Float64[label.station_age[j] for j in stations]
    gaps = Vector{Float64}(undef, length(stations))
    for (n, j) in enumerate(stations)
        v = T - ages[n]
        g = Inf
        for i in pd.by_origin[j]
            o = pd.opps[i]
            o.walk_in > v + NO_ANTICIPATION_EPS || continue
            _has_inactive_layer(o.layer_mask, act) || continue
            t = _na_travel_lb(pd, cur, o.destination)
            (ages[n] + t <= o.ride_cap + 1e-9 && T + t <= o.deadline + 1e-9) || continue
            g = o.walk_in; break
        end
        gaps[n] = g
    end
    fw = Inf
    for j in pd.origins
        dj = _na_travel_lb(pd, cur, j)
        reach = T + dj
        for i in pd.by_origin[j]
            o = pd.opps[i]
            o.walk_in >= fw && break
            o.walk_in > reach + NO_ANTICIPATION_EPS || continue
            _has_inactive_layer(o.layer_mask, act) || continue
            djk = _na_travel_lb(pd, j, o.destination)
            (djk <= o.ride_cap + 1e-9 && o.walk_in + djk <= o.deadline + 1e-9 && reach + djk <= o.deadline + 1e-9) || continue
            fw = o.walk_in; break
        end
    end
    rw = Float64[]
    if isfinite(label.root_age)
        for i in get(pd.by_origin, label.route[1], _NA_EMPTY)
            o = pd.opps[i]
            _has_inactive_layer(o.layer_mask, act) || continue
            t = _na_travel_lb(pd, cur, o.destination)
            (label.root_age + t <= o.ride_cap + 1e-9 && T + t <= o.deadline + 1e-9) || continue
            push!(rw, o.walk_in)
        end
    end
    return NoAnticipationBitsets(stations, ages, gaps, fw, rw)        # rw is sorted: by_origin is sorted by walk
end

"First-stop riders admitted with departure `t0`: M3 walk <= t0, M2 walk in (t0 - 5 s, t0], M1 everyone."
@inline _na_root_admits(pd::NoAnticipationPricingData, t0::Float64, w::Float64) =
    !pd.first_walker || (w <= t0 + NO_ANTICIPATION_EPS &&
                         (pd.gather || w > t0 - NO_ANTICIPATION_FIRST_WALKER_TOL - NO_ANTICIPATION_EPS))

"Does every unpaid, deliverable first-stop rider that departure `tb` admits also get admitted by `ta`?"
function _na_root_covers(pd::NoAnticipationPricingData, root_walks::Vector{Float64}, ta::Float64, tb::Float64)
    abs(ta - tb) <= 1e-9 && return true
    for w in root_walks
        _na_root_admits(pd, tb, w) && !_na_root_admits(pd, ta, w) && return false
    end
    return true
end

"""
`a` dominates `b` (same current stop) when every completion of `b` is matched by `a` at no
worse reduced cost. On top of the `exact` conditions (no later, no older clocks, compensated
reward), the walking rules need:
  - b's first-stop clock (if live): a has the same first stop with a live clock no older that
    admits every first-stop rider b's admits, among those a has not paid and could still
    deliver (M2/M3: departure times may differ when no such rider walks in between);
  - every live later-visit clock of b: a's clock at that stop is no older and takes every
    rider b's can -- a visited no earlier, or no unpaid rider (for a) has a walk between the
    two visit times;
  - stops visited later: a, being earlier, reaches them earlier on b's path and may find a
    rider not yet arrived. Safe if a is not earlier, or if every unpaid rider a could not take
    even arriving directly, and that a later visit could still deliver, walks longer than b's
    latest possible pickup (the pickup deadline).
Clock and first-stop gaps likewise ignore riders a could not deliver in time even if it could
take them (b, no earlier and with no younger clocks, cannot deliver them either).
Riders whose layers a has already paid are covered by the compensation term, as in `exact`.
"""
function _na_dominates(pd::NoAnticipationPricingData, x::PricingLabelEntry, y::PricingLabelEntry)::Bool
    a = x.label; b = y.label
    A = x.bitsets; B = y.bitsets
    a.time <= b.time + 1e-9 || return false
    deadline = _na_pickup_deadline(pd)        # common to every label, so a.time <= b.time already covers it
    (!pd.base.bounded_max_stops || a.route_length <= b.route_length) || return false
    budget = b.reduced_cost - a.reduced_cost + 1e-9
    budget >= 0.0 || return false
    if isfinite(b.root_age)
        (a.route[1] == b.route[1] && isfinite(a.root_age) && a.root_age <= b.root_age + 1e-9) || return false
        _na_root_covers(pd, A.root_walks, a.start_time, b.start_time) || return false
    end
    ia = 1
    for (ib, j) in enumerate(B.stations)
        while ia <= length(A.stations) && A.stations[ia] < j
            ia += 1
        end
        (ia <= length(A.stations) && A.stations[ia] == j) || return false
        A.ages[ia] <= B.ages[ib] + 1e-9 || return false
        va = a.time - A.ages[ia]; vb = b.time - B.ages[ib]
        (va >= vb - 1e-9 || A.gaps[ia] > vb + NO_ANTICIPATION_EPS) || return false
    end
    (a.time >= b.time - 1e-9 || A.future_walk > deadline + NO_ANTICIPATION_EPS) || return false
    _joint_routing_assignment_compensation(
        a.activated_reward_layers, b.activated_reward_layers, pd.base.layer_weight, budget,
        pd.base.compensated_dominance,
    ) <= budget || return false
    return true
end

# ── replay of a finished route ────────────────────────────────────────────────
"""
Replay `route` from t0 under the same rules and return each rider's best credited assignment
`p => (j, k, reward, pickup_position, dropoff_position)` (ties as in `exact`'s replay).
"""
function _na_replay(route::Vector{Int}, t0::Float64, pd::NoAnticipationPricingData)
    best = Dict{Int, Tuple{Int, Int, Float64, Int, Int}}()
    isempty(route) && return best
    base = pd.base
    root = route[1]
    T = t0
    ptime = Dict{Int, Float64}(); ppos = Dict{Int, Int}()
    for idx in 2:length(route)
        node = route[idx]
        T += _na_travel(pd, route[idx - 1], node)
        for i in get(pd.by_destination, node, _NA_EMPTY)
            o = pd.opps[i]
            pos = 0
            if haskey(ptime, o.origin)
                v = ptime[o.origin]
                _na_later_ok(o, v, T - v, T) && (pos = ppos[o.origin])
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
        if T <= _na_pickup_deadline(pd) + 1e-9
            ptime[node] = T; ppos[node] = idx
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
_pricing_state(::NoAnticipationSearchContext, label::NoAnticipationLabel, ::NoAnticipationBitsets) = label.current
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
    pickup_deadline_s::Float64,
    gather::Bool=false,
)::Vector{JointRoutingAssignmentRouteColumn}
    ratio = mapping.door_to_door_ratio
    by_route = Dict{Tuple{Int, Int, Int}, Vector{Tuple{Float64, Float64, Int}}}()   # (walk, deadline, p)
    for s in 1:n_scenarios(data)
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            mapping.Q_s[s][p] > 0 || continue
            for pair in get_valid_jk_pairs(mapping, o, d)
                is_walk_only_pair(pair) && continue
                j, k = pair
                isfinite(get_routing_cost(data, j, k)) || continue
                direct = get_routing_cost(data, o, d)
                dl = (isfinite(ratio) && isfinite(direct)) ? ratio * direct - get_walking_cost(data, k, d) : Inf
                w = get_walking_cost(data, o, j)
                w <= pickup_deadline_s + 1e-9 || continue         # cannot board before the pickup deadline
                push!(get!(by_route, (s, j, k), Tuple{Float64, Float64, Int}[]), (w, dl, p))
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
                ps = [r[3] for r in riders if r[1] <= t0 + NO_ANTICIPATION_EPS && t0 + tau <= r[2] + 1e-9]
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
            ps = [r[3] for r in riders[start:stop] if t0 + tau <= r[2] + 1e-9]
            if length(ps) > 1
                push!(columns, mk(s, j, k, t0, ps, "two_stop_first_walker")); id += 1
            end
            start = stop + 1
        end
        for (w, _dl, p) in riders
            push!(columns, mk(s, j, k, w, [p], "two_stop_singleton")); id += 1
        end
    end
    return columns
end
