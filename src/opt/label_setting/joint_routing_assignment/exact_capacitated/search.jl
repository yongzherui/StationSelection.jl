"""
Search logic for `:exact_capacitated` (see `types.jl` for the model): pricing data, seed
labels, candidate actions (next stop x critical riders credited there), extension,
dominance, the exact capacitated optimum of a fixed route, and the loop growing `S`.
"""

# ── pricing data ─────────────────────────────────────────────────────────────
"""
    create_joint_routing_assignment_capacitated_pricing_data(scenario, nodes, travel_cost,
        candidates; capacity, load, critical = Bool[], stop_at_first_column = true,
        growth_routes = 5, kwargs...)

`load[p]` is rider group `p`'s number of requests. Candidates whose group alone exceeds
`capacity` are dropped (the map already drops such groups when a capacity is set).
`critical` is the starting `S` (indexed by `p`); the search grows it IN PLACE, so a caller
that passes the same vector again (e.g. the next CG iteration) starts from the grown set.
"""
function create_joint_routing_assignment_capacitated_pricing_data(
    scenario::Int,
    nodes::Vector{Int},
    travel_cost::Dict{Tuple{Int, Int}, Float64},
    candidates::AbstractVector{PassengerAssignmentCandidate};
    capacity::Int,
    load::Dict{Int, Int},
    critical::Vector{Bool}=Bool[],
    stop_at_first_column::Bool=true,
    growth_routes::Int=5,
    initial_critical::Symbol=:all,
    kwargs...,
)::JointRoutingAssignmentCapacitatedPricingData
    capacity >= 1 || throw(ArgumentError("vehicle capacity must be >= 1, got $capacity"))
    kept = PassengerAssignmentCandidate[c for c in candidates if get(load, c.p, 1) <= capacity]
    base = create_joint_routing_assignment_pricing_data(scenario, nodes, travel_cost, kept; kwargs...)
    max_reward = Dict{Int, Float64}()
    full_mask = Dict{Int, RewardLayerBitset}()
    for opp in base.opportunities
        max_reward[opp.p] = max(get(max_reward, opp.p, 0.0), opp.reward)
        full_mask[opp.p] = union(get(full_mask, opp.p, RewardLayerBitset()), opp.layer_mask)
    end
    maxp = maximum(keys(max_reward); init = 0)
    n0 = length(critical)
    n0 < maxp && append!(critical, falses(maxp - n0))
    # riders seen for the first time join S unless `initial_critical == :empty` (then S grows
    # from empty, `hooks.jl`'s loop). On the instances tried the relaxation with S empty is far
    # from the capacitated optimum and S grows to almost every rider, so `:all` is the default.
    if initial_critical === :all
        for p in keys(max_reward)
            p > n0 && (critical[p] = true)
        end
    end
    opportunity = Dict((o.p, o.origin, o.destination) => o for o in base.opportunities)
    node_idx = Dict(node => i for (i, node) in enumerate(base.nodes))
    n = length(base.nodes)
    sp = fill(Inf, n, n)
    for (i, u) in enumerate(base.nodes), (j, v) in enumerate(base.nodes)
        i == j ? (sp[i, j] = 0.0) : haskey(base.travel_cost, (u, v)) && (sp[i, j] = base.travel_cost[(u, v)])
    end
    @inbounds for k in 1:n, i in 1:n
        dik = sp[i, k]
        isfinite(dik) || continue
        for j in 1:n
            v = dik + sp[k, j]
            v < sp[i, j] && (sp[i, j] = v)
        end
    end
    # travel times already shortest paths (triangle inequality holds): then a stop that
    # credits nobody after the pickup window is never needed (`_capacitated_candidate_actions`)
    metric = all(sp[node_idx[u], node_idx[v]] >= c - 1e-9 for ((u, v), c) in base.travel_cost
                 if haskey(node_idx, u) && haskey(node_idx, v))
    opps_by_p = Dict{Int, Vector{PassengerAssignmentOpportunity}}()
    for o in base.opportunities
        push!(get!(() -> PassengerAssignmentOpportunity[], opps_by_p, o.p), o)
    end
    return JointRoutingAssignmentCapacitatedPricingData(
        base, capacity, Dict(p => get(load, p, 1) for p in keys(max_reward)), max_reward, full_mask,
        opportunity, critical, stop_at_first_column, growth_routes, sp, node_idx, opps_by_p, metric,
    )
end

# ── seed ─────────────────────────────────────────────────────────────────────
function initial_joint_routing_assignment_capacitated_pricing_labels(
    pd::JointRoutingAssignmentCapacitatedPricingData,
)::Vector{JointRoutingAssignmentCapacitatedPricingLabel}
    base = pd.base
    origins = Set{Int}(opp.origin for opp in base.opportunities)
    labels = JointRoutingAssignmentCapacitatedPricingLabel[]
    for node in base.nodes
        node in origins || continue
        push!(labels, JointRoutingAssignmentCapacitatedPricingLabel(
            node, [node], 0.0,
            Dict(node => 0.0), Dict(node => 0), Dict(node => 1),
            RewardLayerBitset(), RewardLayerBitset(), CapacitatedCredit[], BitSet(),
            0.0, base.route_regularization_weight * base.repositioning_time, 1,
            Float64[0.0], Int[],
        ))
    end
    return labels
end

# ── load updates ─────────────────────────────────────────────────────────────
"""`_capacitated_credit_load!` on vectors (clock `ji`; the caller has checked the fit)."""
@inline function _capacitated_credit_vec!(lv::Vector{Int}, av::Vector{Float64}, ji::Int, q::Int)
    fj = lv[ji]
    aj = av[ji]
    @inbounds for c in eachindex(lv)
        lv[c] = av[c] <= aj + 1e-12 ? lv[c] + q : max(lv[c], fj + q)
    end
    return nothing
end

"""Put a critical rider of load `q` on every arc since clock `j`'s visit (see `types.jl`).
`ages` are the clocks' ages at the current stop. Returns `false` (and changes nothing) if
that would exceed `capacity` on some arc."""
function _capacitated_credit_load!(loads::Dict{Int, Int}, ages::Dict{Int, Float64}, j::Int, q::Int, capacity::Int)::Bool
    fj = loads[j]
    fj + q <= capacity || return false
    aj = ages[j]
    for (c, ac) in ages
        loads[c] = ac <= aj + 1e-12 ? loads[c] + q : max(loads[c], fj + q)
    end
    return true
end

# ── candidate actions ────────────────────────────────────────────────────────
"""`exact/`'s next-stop rule, read against this label's activated layers (a credited
critical rider has its full mask active, an uncredited one none)."""
_capacitated_candidate_next_nodes(label::JointRoutingAssignmentCapacitatedPricingLabel, base::JointRoutingAssignmentPricingData) =
    _joint_routing_assignment_candidate_next_nodes_generic(label, base)

function _joint_routing_assignment_candidate_next_nodes_generic(label, base::JointRoutingAssignmentPricingData)
    candidate_nodes = Set{Int}()
    past_cutoff = label.time > base.max_wait_time + 1e-9
    past_cutoff && !_has_useful_live_joint_routing_assignment_origin(label, base) && return Int[]
    if !past_cutoff
        for (origin, mask) in base.origin_layer_mask
            origin == label.current && continue
            _has_inactive_layer(mask, label.activated_reward_layers) || continue
            label.time + _joint_routing_assignment_travel(base, label.current, origin) <= base.max_wait_time + 1e-9 &&
                push!(candidate_nodes, origin)
        end
    end
    for (origin, age) in label.station_age
        for opp in get(base.assignments_by_origin, origin, PassengerAssignmentOpportunity[])
            opp.destination == label.current && continue
            opp.destination in candidate_nodes && continue
            _has_inactive_layer(opp.layer_mask, label.activated_reward_layers) || continue
            age + _joint_routing_assignment_travel(base, label.current, opp.destination) <= opp.ride_limit + 1e-9 || continue
            push!(candidate_nodes, opp.destination)
        end
    end
    return sort!(collect(candidate_nodes))
end

"""Every (next stop, credited critical riders) action. At stop `n`, critical rider `p` can be
credited through a live clock `j` with a `(p, j, n)` opportunity within its ride limit; per
rider only the Pareto options (higher reward, younger clock) are kept, since a younger
clock's ride covers a subset of an older one's arcs. Non-critical riders are never branched
on (the extension certifies them as `exact/` does)."""
function _capacitated_candidate_actions(label::JointRoutingAssignmentCapacitatedPricingLabel, pd::JointRoutingAssignmentCapacitatedPricingData;
                                       ctx = nothing, thr::Float64 = Inf)
    base = pd.base
    actions = JointRoutingAssignmentCapacitatedAction[]
    for n in _capacitated_candidate_next_nodes(label, base)
        t = _joint_routing_assignment_travel(base, label.current, n)
        # After the pickup window a stop only matters for what it credits; with metric travel
        # times, driving straight on is never worse than a stop that credits nobody (same
        # clocks, no later, one stop fewer). Non-critical credits count too.
        skip_empty = pd.metric && label.time + t > base.max_wait_time + 1e-9 &&
                     !_capacitated_noncritical_gain(label, n, t, pd)
        options = nothing
        for opp in get(base.assignments_by_destination, n, PassengerAssignmentOpportunity[])
            p = opp.p
            _is_critical(pd, p) || continue
            p in label.credited_set && continue
            a = get(label.station_age, opp.origin, Inf) + t
            a <= opp.ride_limit + 1e-9 || continue
            label.station_load[opp.origin] + pd.load[p] <= pd.capacity || continue
            options === nothing && (options = Dict{Int, Vector{Tuple{Int, Float64}}}())
            push!(get!(() -> Tuple{Int, Float64}[], options, p), (opp.origin, opp.reward))
        end
        if options === nothing
            skip_empty || push!(actions, JointRoutingAssignmentCapacitatedAction(n, Tuple{Int, Int, Float64}[]))
            continue
        end
        ages = Dict{Int, Float64}(c => a + t for (c, a) in label.station_age)
        riders = sort!(collect(keys(options)))
        for p in riders
            opts = options[p]
            keep = Tuple{Int, Float64}[]
            for (j, r) in opts
                dominated = any(((j2, r2),) -> (j2, r2) != (j, r) && r2 >= r - 1e-12 && ages[j2] <= ages[j] + 1e-12 &&
                                    (r2 > r + 1e-12 || ages[j2] < ages[j] - 1e-12 || j2 < j), opts)
                dominated || push!(keep, (j, r))
            end
            options[p] = keep
        end
        credits = Tuple{Int, Int, Float64}[]
        loads = copy(label.station_load)
        optional_riders = Int[p for p in riders if !isempty(options[p])]
        # Pruning the subset search (when the caller passes its threshold `thr`): the child
        # that credits nobody here has reduced cost `rc0` and remaining bound `B0`. More
        # credits only shrink the bound (fewer riders left, fuller arcs), so a child with
        # extra credits worth `c` has priority at least `rc0 - c - B0`: it is cut unless
        # `c > gap = rc0 - B0 - thr`. A branch whose credits so far plus the best each
        # remaining rider can add stay within `gap` is dropped; each finished subset is then
        # scored with its own bound from one scan of that child (`_capacitated_scan!`).
        _capacitated_subsets!(actions, label, pd, ctx, thr, n, t, ages, loads, credits, optional_riders, options, skip_empty)
    end
    return actions
end

struct _CapSubsetSearch{X}
    n::Int
    riders::Vector{Int}
    q::Vector{Int}
    opts::Vector{Vector{Tuple{Int, Int, Float64}}}   # per rider: (clock slot in `lv`, clock, reward)
    rem::Vector{Float64}
    gap::Float64
    lv::Vector{Int}
    av::Vector{Float64}
    bufs::Vector{Vector{Int}}
    credits::Vector{Tuple{Int, Int, Float64}}
    nforced::Int
    skip_empty::Bool
    cap::Int
    scan::Bool
    rc0::Float64
    noncrit0::Float64
    thr::Float64
    slot_parent::Vector{Int}
    slot_load::Vector{Int}
    excluded::BitSet
    ctx::X
    probe::Union{Nothing, JointRoutingAssignmentCapacitatedPricingLabel}
end

function _capacitated_subsets!(actions, label, pd::JointRoutingAssignmentCapacitatedPricingData, ctx, thr::Float64,
                               n::Int, t::Float64, ages::Dict{Int, Float64}, loads::Dict{Int, Int},
                               credits::Vector{Tuple{Int, Int, Float64}}, riders::Vector{Int},
                               options::Dict{Int, Vector{Tuple{Int, Float64}}}, skip_empty::Bool)
    base = pd.base
    clk = collect(keys(ages))
    av = Float64[ages[c] for c in clk]
    lv = Int[loads[c] for c in clk]
    opts = [Tuple{Int, Int, Float64}[(findfirst(==(j), clk)::Int, j, r) for (j, r) in options[p]] for p in riders]
    rem = zeros(length(riders) + 1)
    for i in length(riders):-1:1
        rem[i] = rem[i + 1] + maximum(x -> x[3], opts[i])
    end
    scan = ctx !== nothing && isfinite(thr) && !isempty(riders)
    gap = -Inf; rc0 = 0.0; noncrit0 = 0.0
    slot_parent = Int[]; slot_load = Int[]; excluded = BitSet()
    probe = nothing
    if scan
        probe = _extend_joint_routing_assignment_capacitated_label(label, JointRoutingAssignmentCapacitatedAction(n, credits), pd)
        p_idx, p_val, _ = _make_sparse_station_ages(probe.station_age, ctx.search_index.node_index)
        noncrit0 = _capacitated_scan!(ctx.cap_scan, ctx, probe, p_idx, p_val)
        fresh = label.time + t <= base.max_wait_time + 1e-9
        for i in p_idx
            c = base.nodes[i]
            push!(slot_parent, (c == n && fresh) ? 0 : findfirst(==(c), clk)::Int)
            push!(slot_load, probe.station_load[c])
        end
        rc0 = probe.reduced_cost
        gap = rc0 - noncrit0 - _capacitated_scan_eval!(ctx.cap_scan, ctx, excluded, slot_load,
                                                       sum((pd.load[c[1]] for c in credits); init = 0)) - thr - 1e-9
    end
    st = _CapSubsetSearch(n, riders, Int[pd.load[p] for p in riders], opts, rem, gap, lv, av,
                          [similar(lv) for _ in 1:length(riders)], credits, length(credits), skip_empty,
                          pd.capacity, scan, rc0, noncrit0, thr, slot_parent, slot_load, excluded, ctx, probe)
    _capacitated_subsets_dfs!(actions, st, 1, 0.0)
    return actions
end

function _capacitated_subsets_dfs!(actions, st::_CapSubsetSearch, i::Int, csum::Float64)
    csum + st.rem[i] <= st.gap && return
    credits = st.credits
    if i > length(st.riders)
        (st.skip_empty && isempty(credits)) && return
        if st.scan && csum > 0
            excluded = st.excluded
            empty!(excluded)
            for k in st.nforced+1:length(credits)
                push!(excluded, credits[k][1])
            end
            @inbounds for k in eachindex(st.slot_parent)
                sp_ = st.slot_parent[k]
                st.slot_load[k] = sp_ == 0 ? 0 : st.lv[sp_]
            end
            last = 0
            for c in credits
                last += st.ctx.pricing_data.load[c[1]]
            end
            b = st.noncrit0 + _capacitated_scan_eval!(st.ctx.cap_scan, st.ctx, excluded, st.slot_load, last)
            if st.rc0 - csum - b >= st.thr
                return
            end
        end
        push!(actions, length(credits) == st.nforced && st.probe !== nothing ?
              JointRoutingAssignmentCapacitatedAction(st.n, copy(credits), st.probe) :
              JointRoutingAssignmentCapacitatedAction(st.n, copy(credits)))
        return
    end
    _capacitated_subsets_dfs!(actions, st, i + 1, csum)            # skip rider i
    p = st.riders[i]
    q = st.q[i]
    buf = st.bufs[i]
    lv = st.lv
    for (ji, j, r) in st.opts[i]
        csum + r + st.rem[i + 1] <= st.gap && continue
        lv[ji] + q <= st.cap || continue
        copyto!(buf, lv)
        _capacitated_credit_vec!(lv, st.av, ji, q)
        push!(credits, (p, j, r))
        _capacitated_subsets_dfs!(actions, st, i + 1, csum + r)
        pop!(credits)
        copyto!(lv, buf)
    end
    return
end

"""Does stop `n` (reached after driving `t`) certify some new non-critical reward layer?"""
function _capacitated_noncritical_gain(label::JointRoutingAssignmentCapacitatedPricingLabel, n::Int, t::Float64,
                                       pd::JointRoutingAssignmentCapacitatedPricingData)::Bool
    for opp in get(pd.base.assignments_by_destination, n, PassengerAssignmentOpportunity[])
        _is_critical(pd, opp.p) && continue
        get(label.station_age, opp.origin, Inf) + t <= opp.ride_limit + 1e-9 || continue
        _has_inactive_layer(opp.layer_mask, label.activated_reward_layers) && return true
    end
    return false
end

# ── extension ────────────────────────────────────────────────────────────────
function _extend_joint_routing_assignment_capacitated_label(
    label::JointRoutingAssignmentCapacitatedPricingLabel,
    action::JointRoutingAssignmentCapacitatedAction,
    pd::JointRoutingAssignmentCapacitatedPricingData,
)::JointRoutingAssignmentCapacitatedPricingLabel
    base = pd.base
    n = action.node
    t = _joint_routing_assignment_travel(base, label.current, n)
    arrival = label.time + t
    route = vcat(label.route, n)
    pos = length(route)
    pos_time = vcat(label.pos_time, arrival)
    arc_load = vcat(label.arc_load, 0)

    # Non-critical riders: exact/'s certification (running max over reward layers).
    activated = label.activated_reward_layers
    noncrit = label.noncrit_layers
    reward = 0.0
    reachable = nothing
    for opp in get(base.assignments_by_destination, n, PassengerAssignmentOpportunity[])
        _is_critical(pd, opp.p) && continue
        get(label.station_age, opp.origin, Inf) + t <= opp.ride_limit + 1e-9 || continue
        reachable === nothing && (reachable = RewardLayerBitset())
        union!(reachable, opp.layer_mask)
    end
    if reachable !== nothing
        new_layers = setdiff(reachable, activated)
        if !isempty(new_layers)
            reward += _sum_layer_weights(base, new_layers)
            activated = union(activated, new_layers)
            noncrit = union(noncrit, new_layers)
        end
    end

    # Critical riders: the action's fixed credits.
    if isempty(action.credits)
        loads = label.station_load
        credited = label.credited
        cset = label.credited_set
        ages = nothing
    else
        ages = Dict{Int, Float64}(c => a + t for (c, a) in label.station_age)
        loads = copy(label.station_load)
        credited = copy(label.credited)
        cset = copy(label.credited_set)
        for (p, j, r) in action.credits
            ok = _capacitated_credit_load!(loads, ages, j, pd.load[p], pd.capacity)
            @assert ok "capacitated action over capacity: rider $p via clock $j at stop $n"
            push!(credited, (p, j, n, label.station_pos[j], pos))
            for i in label.station_pos[j]:pos-1
                arc_load[i] += pd.load[p]
            end
            push!(cset, p)
            activated = union(activated, pd.full_mask[p])
            reward += r
        end
    end

    # Age, prune, and refresh clocks exactly as `exact/` does. (A clock at capacity still
    # serves non-critical riders, so it is not dropped.)
    new_age = Dict{Int, Float64}(); new_load = Dict{Int, Int}(); new_pos = Dict{Int, Int}()
    for (c, a) in label.station_age
        c == n && continue
        aged = a + t
        _joint_routing_assignment_age_is_useful(c, aged, base, n) || continue
        new_age[c] = aged; new_load[c] = loads[c]; new_pos[c] = label.station_pos[c]
    end
    if arrival <= base.max_wait_time + 1e-9
        if _joint_routing_assignment_age_is_useful(n, 0.0, base, n)
            new_age[n] = 0.0; new_load[n] = 0; new_pos[n] = pos
        end
    elseif haskey(label.station_age, n)
        aged = label.station_age[n] + t
        if _joint_routing_assignment_age_is_useful(n, aged, base, n)
            new_age[n] = aged; new_load[n] = loads[n]; new_pos[n] = label.station_pos[n]
        end
    end

    return JointRoutingAssignmentCapacitatedPricingLabel(
        n, route, arrival, new_age, new_load, new_pos, activated, noncrit, credited, cset,
        label.tau + t,
        label.reduced_cost + base.route_regularization_weight * t - reward,
        label.route_length + 1, pos_time, arc_load,
    )
end

# ── dominance ────────────────────────────────────────────────────────────────
function _make_joint_routing_assignment_capacitated_label_bitsets(
    label::JointRoutingAssignmentCapacitatedPricingLabel, ctx,
)::JointRoutingAssignmentCapacitatedLabelBitsets
    node_index = ctx.search_index.node_index
    age_idx, age_val, age_mask = _make_sparse_station_ages(label.station_age, node_index)
    max_load = isempty(label.station_load) ? 0 : maximum(values(label.station_load))
    load_val = if max_load == 0
        Int[]
    else
        lv = zeros(Int, length(age_idx))
        for (c, l) in label.station_load
            k = searchsortedfirst(age_idx, Int32(node_index[c]))
            lv[k] = l
        end
        lv
    end
    reach = zeros(Float64, length(ctx.cap_bound.v_best))
    bound = _capacitated_remaining_bound(ctx, label, age_idx, age_val, load_val, reach)
    max_load == 0 && return JointRoutingAssignmentCapacitatedLabelBitsets(
        label.noncrit_layers, label.credited_set, reach, bound,
        age_idx, age_val, age_mask, Int[], Float64[], Int[], 0)
    # F at every arc start, youngest first: arc i started `time - pos_time[i]` ago
    L = length(label.arc_load)
    ages_sorted = Float64[label.time - label.pos_time[i] for i in L:-1:1]
    loads_sorted = accumulate(max, label.arc_load[L:-1:1]; init = 0)
    return JointRoutingAssignmentCapacitatedLabelBitsets(
        label.noncrit_layers, label.credited_set, reach, bound,
        age_idx, age_val, age_mask, load_val, ages_sorted, loads_sorted, max_load,
    )
end

JointRoutingAssignmentCapacitatedDominanceFilters(label::JointRoutingAssignmentCapacitatedPricingLabel, bs::JointRoutingAssignmentCapacitatedLabelBitsets) =
    JointRoutingAssignmentCapacitatedDominanceFilters(label.reduced_cost, label.time, bs.age_mask,
                                                      Int32(label.route_length), Int32(length(bs.age_idx)))

PricingLabelEntry(id::Int, label::JointRoutingAssignmentCapacitatedPricingLabel, bs::JointRoutingAssignmentCapacitatedLabelBitsets) =
    PricingLabelEntry(JointRoutingAssignmentCapacitatedDominanceFilters(label, bs), id, label, bs)

"""`F_b(τ)`: the largest load on an arc of `b`'s that started at most `τ` ago (0 if none); at a
clock's age this is the clock's stored load. Read from the arc loads, so it is exact at every
arc start, not only at clock ages."""
function _capacitated_load_lower_bound(bs::JointRoutingAssignmentCapacitatedLabelBitsets, tau::Float64)::Int
    bs.max_load == 0 && return 0
    i = searchsortedlast(bs.ages_sorted, tau + 1e-9)
    return i == 0 ? 0 : bs.loads_sorted[i]
end

"""
Can `a` discard `b` (same current stop)? Sufficient conditions, each needed by the proof:

1. `time_a <= time_b` (and `route_length_a <= route_length_b` when stops are bounded): any
   completion of `b` is also on time for `a`, opens the same new clocks, and fits `max_stops`.
2. every live clock `j` of `b` is live in `a`, no older, with `F_a(age_a(j)) <= G_b(age_a(j))`.
   Invariant "`F_a <= F_b` at the ages of `b`'s clocks" survives driving (all ages shift
   alike) and crediting critical rider `q` through clock `j` (three cases on `τ` vs
   `age_a(j) <= age_b(j)`, using that `F_b` is nondecreasing), and new clocks start at 0 in
   both; so every critical credit `b` makes later fits in `a` too, as do the ride limits.
3. reward: `a` copies `b`'s completion. Non-critical riders: `a` certifies at least what `b`
   does (ages no older), except for layers `a` already holds -- `exact/`'s compensation
   `w(noncrit_a ∖ noncrit_b)`. Critical riders: `a` copies `b`'s later credits except those of
   riders `a` already credited (fewer loads, so the rest still fit); `b` can earn at most
   `reach_b[p]` from such a rider (its largest reward over the options still open to `b`;
   0 if none). So it suffices that `rc_a + w(noncrit_a ∖ noncrit_b) + sum(reach_b[p] : p ∈
   cred_a ∖ cred_b)
   <= rc_b` (plain: both extra terms must be empty).
"""
function _capacitated_dominates(
    fa::JointRoutingAssignmentCapacitatedDominanceFilters, ba::JointRoutingAssignmentCapacitatedLabelBitsets,
    fb::JointRoutingAssignmentCapacitatedDominanceFilters, bb::JointRoutingAssignmentCapacitatedLabelBitsets,
    pd::JointRoutingAssignmentCapacitatedPricingData,
    ::JointRoutingAssignmentCapacitatedDominanceRules{BoundedStops, Compensated},
)::Bool where {BoundedStops, Compensated}
    fa.time <= fb.time + 1e-9 || return false
    # live-clock support from the inline filters only (no pointer chase into the bitsets):
    # `b`'s clocks must all be live in `a`. The mask folds stations mod 64, so it is only a
    # necessary test; the age walk below is the exact one.
    fa.n_live_ages >= fb.n_live_ages || return false
    fb.age_mask & ~fa.age_mask == 0 || return false
    BoundedStops && (fa.route_length <= fb.route_length || return false)
    budget = fb.reduced_cost - fa.reduced_cost + 1e-9
    budget >= 0.0 || return false
    if ba.max_load == 0
        _sparse_station_age_values_dominate(ba.age_idx, ba.age_val, bb.age_idx, bb.age_val) || return false
    else
        ia, na = 1, length(ba.age_idx)
        @inbounds for ib in eachindex(bb.age_idx)
            idx = bb.age_idx[ib]
            while ia <= na && ba.age_idx[ia] < idx
                ia += 1
            end
            (ia <= na && ba.age_idx[ia] == idx) || return false
            age_a = ba.age_val[ia]
            age_a <= bb.age_val[ib] + 1e-9 || return false
            ba.load_val[ia] <= _capacitated_load_lower_bound(bb, age_a) || return false
        end
    end
    comp = _joint_routing_assignment_compensation(
        ba.noncrit_bits, bb.noncrit_bits, pd.base.layer_weight, budget, Val(Compensated))
    comp <= budget || return false
    if !isempty(ba.credited_set)
        for p in ba.credited_set
            p in bb.credited_set && continue
            v = p <= length(bb.reach) ? bb.reach[p] : 0.0
            v > 0.0 || continue
            Compensated || return false
            comp += v
            comp <= budget || return false
        end
    end
    return true
end

# ── exact capacitated optimum of a fixed route ───────────────────────────────
"""
    _capacitated_route_options(route, pd) -> (times, Dict{p => [(a, b, j, k, reward)]})

Every way rider `p` can ride `route`: board at a visit `a` of `j` inside the pickup window,
alight at a later visit `b` of `k` within its ride limit. For a given dropoff visit `b` only
the latest valid boarding visit of `j` is kept (it rides a subset of the arcs at the same
reward), and options riding a superset of another's arcs for no more reward are dropped.
"""
function _capacitated_route_options(route::Vector{Int}, pd::JointRoutingAssignmentCapacitatedPricingData)
    base = pd.base
    L = length(route)
    times = zeros(L)
    for i in 2:L
        times[i] = times[i - 1] + _joint_routing_assignment_travel(base, route[i - 1], route[i])
    end
    last_visit = Dict{Int, Int}()       # station => latest visit inside the pickup window so far
    opts = Dict{Int, Vector{Tuple{Int, Int, Int, Int, Float64}}}()
    for b in 1:L
        if b >= 2
            for opp in get(base.assignments_by_destination, route[b], PassengerAssignmentOpportunity[])
                a = get(last_visit, opp.origin, 0)
                a == 0 && continue
                times[b] - times[a] <= opp.ride_limit + 1e-9 || continue
                push!(get!(() -> Tuple{Int, Int, Int, Int, Float64}[], opts, opp.p), (a, b, opp.origin, opp.destination, opp.reward))
            end
        end
        times[b] <= base.max_wait_time + 1e-9 && (last_visit[route[b]] = b)
    end
    for (p, v) in opts
        keep = Tuple{Int, Int, Int, Int, Float64}[]
        for o in v
            dominated = any(o2 -> o2 !== o && o2[5] >= o[5] - 1e-12 && o2[1] >= o[1] && o2[2] <= o[2] &&
                                 (o2[5] > o[5] + 1e-12 || o2[1] > o[1] || o2[2] < o[2] || (o2[3], o2[4]) < (o[3], o[4])), v)
            dominated || push!(keep, o)
        end
        sort!(keep; by = o -> (-o[5], o[2] - o[1], o[1]))
        opts[p] = keep
    end
    return times, opts
end

"""
    _capacitated_route_optimum(route, pd) -> (assignments, tau, rc, positions, max_load)

The best assignment of riders to `route` with every rider's load counted against the
capacity: branch and bound over riders (most valuable first), each skipped or given one of
its options, pruned by the sum of the remaining riders' best rewards. Tries the greedy
"everyone at their best option" assignment first, which is optimal whenever it fits.
"""
function _capacitated_route_optimum(route::Vector{Int}, pd::JointRoutingAssignmentCapacitatedPricingData)
    base = pd.base
    times, opts = _capacitated_route_options(route, pd)
    L = length(route)
    riders = sort!(collect(keys(opts)); by = p -> (-opts[p][1][5], p))
    nR = length(riders)
    arc = zeros(Int, max(L - 1, 0))
    fits(a, b, q) = all(e -> arc[e] + q <= pd.capacity, a:(b - 1))
    put!(a, b, q) = (for e in a:(b - 1); arc[e] += q; end)
    best_val = 0.0
    best_pick = zeros(Int, nR)
    # greedy: everyone at their best option
    greedy_ok = true
    for (i, p) in enumerate(riders)
        a, b = opts[p][1][1], opts[p][1][2]
        fits(a, b, pd.load[p]) || (greedy_ok = false; break)
        put!(a, b, pd.load[p])
    end
    if greedy_ok
        best_pick .= 1
        best_val = sum(opts[p][1][5] for p in riders; init = 0.0)
    else
        fill!(arc, 0)
        suffix = zeros(nR + 1)
        for i in nR:-1:1
            suffix[i] = suffix[i + 1] + opts[riders[i]][1][5]
        end
        pick = zeros(Int, nR)
        function bb(i::Int, val::Float64)
            if val > best_val + 1e-12
                best_val = val
                best_pick .= pick
                for k in i:nR; best_pick[k] = 0; end
            end
            i > nR && return
            val + suffix[i] <= best_val + 1e-12 && return
            p = riders[i]
            q = pd.load[p]
            for (oi, (a, b, _, _, r)) in enumerate(opts[p])
                fits(a, b, q) || continue
                put!(a, b, q); pick[i] = oi
                bb(i + 1, val + r)
                put!(a, b, -q); pick[i] = 0
            end
            bb(i + 1, val)
        end
        bb(1, 0.0)
        fill!(arc, 0)
        for (i, p) in enumerate(riders)
            best_pick[i] == 0 && continue
            o = opts[p][best_pick[i]]
            put!(o[1], o[2], pd.load[p])
        end
        @assert all(<=(pd.capacity), arc)
    end
    assignments = Tuple{Int, Int, Int}[]
    positions = Dict{Int, Tuple{Int, Int}}()
    for (i, p) in enumerate(riders)
        best_pick[i] == 0 && continue
        a, b, j, k, _ = opts[p][best_pick[i]]
        push!(assignments, (p, j, k))
        positions[p] = (a, b)
    end
    tau = times[end]
    rc = base.route_regularization_weight * (tau + base.repositioning_time) - best_val
    return assignments, tau, rc, positions, maximum(arc; init = 0)
end

"""The label's own relaxed assignment on its route (critical riders as credited, every other
rider at its best option by `exact/`'s replay) and the riders on its over-capacity arcs who
are not yet critical. Empty when the relaxed assignment fits."""
function _capacitated_overload_riders(label::JointRoutingAssignmentCapacitatedPricingLabel, pd::JointRoutingAssignmentCapacitatedPricingData)
    route = label.route
    L = length(route)
    arc = zeros(Int, max(L - 1, 0))
    for (p, _, _, a, b) in label.credited
        for e in a:(b - 1); arc[e] += pd.load[p]; end
    end
    free = Tuple{Int, Int, Int}[]
    for (p, v) in _replay_joint_routing_assignment_route(route, pd.base)
        _is_critical(pd, p) && continue
        a, b = v[4], v[5]
        push!(free, (p, a, b))
        for e in a:(b - 1); arc[e] += pd.load[p]; end
    end
    over = findall(>(pd.capacity), arc)
    isempty(over) && return Int[]
    return sort!(unique!([p for (p, a, b) in free if any(e -> a <= e < b, over)]))
end

# ── capacity-aware remaining-reward bound ────────────────────────────────────
function _build_capacitated_bound_index(pd::JointRoutingAssignmentCapacitatedPricingData, index::JointRoutingAssignmentSearchIndex)
    opps = pd.base.opportunities
    @assert all(index.node_index[nd] == pd.node_idx[nd] for nd in pd.base.nodes)
    np = maximum(keys(pd.max_reward); init = 0)
    bi = CapacitatedBoundIndex(
        [o.p for o in opps], [o.reward for o in opps], [pd.load[o.p] for o in opps],
        [index.node_index[o.origin] for o in opps], pd.sp,
        Bool[], Vector{Int}[], RewardLayerBitset[],
        zeros(np), fill(Inf, np), zeros(np), zeros(np), zeros(np), fill(Inf, np), Int[],
        zeros(np), zeros(np),
        Int[], Float64[], Float64[], Float64[], Int[], zeros(np), zeros(np),
        Float64[], Float64[], Float64[], Float64[], Float64[], Int[],
    )
    _refresh_capacitated_bound_index!(bi, pd, index)
    return bi
end

function _refresh_capacitated_bound_index!(bi::CapacitatedBoundIndex, pd::JointRoutingAssignmentCapacitatedPricingData,
                                           index::JointRoutingAssignmentSearchIndex)
    n = length(index.opps_by_origin_idx)
    bi.opp_critical = [_is_critical(pd, p) for p in bi.opp_p]
    bi.crit_opps_by_origin_idx = [Int[] for _ in 1:n]
    bi.noncrit_origin_union_mask = [RewardLayerBitset() for _ in 1:n]
    for (i, c) in enumerate(bi.opp_critical)
        o = bi.opp_origin_idx[i]
        c ? push!(bi.crit_opps_by_origin_idx[o], i) : union!(bi.noncrit_origin_union_mask[o], index.opp_layer_mask[i])
    end
    return bi
end

"""
Admissible bound on what a completion of `label` can still add to its reward net of future
travel cost, for the relaxed problem (capacity on critical riders only); also fills `reach`
with the uncredited critical riders that have an option passing the necessary tests.

Non-critical riders: `exact/`'s bound (every reachable inactive layer), on their layers only.

Critical riders. The future drive time `T` costs `w * T`, and riders credited in the future
ride only future arcs from now on, at most `C` per arc, so `w * T >= (w / C) * sum Q_p d_p`
with `d_p` a lower bound on `p`'s future ride (`sp[current, k]` through a live clock, `sp[j, k]`
through a clock opened later). So each option is worth at most `v = r - (w / C) * Q * d`, and the
future credits are a set of riders, each with one option, worth at most the sum of their `v`.
That sum is bounded by the smallest of these relaxations, each valid alone:

  1. every reachable rider at its best `v`;
  2. ride time: all future rides fit in `[now, horizon_end]` at `C` per instant
     (fractional knapsack, weight `Q * d`);
  3. stops: a stop drops only riders aboard on the arc into it, at most `C`, so `sum Q <=
     C * (stops left)`;
  4. by class, with `tstar = max(now, max_wait_time)` (no pickup after `max_wait_time`):
     riders credited through a live clock are all aboard on the next arc (`sum Q <= C`);
     riders boarding at a later clock either finish by `tstar`, riding at least `d` inside
     `[now, tstar]` (`sum Q d <= C (tstar - now)`), or are aboard at `tstar` (`sum Q <= C`).
     A knapsack per class, summed (dropping "one class per rider" only loosens it).
"""
function _capacitated_remaining_bound(ctx, label::JointRoutingAssignmentCapacitatedPricingLabel,
                                      age_idx::Vector{Int32}, age_val::Vector{Float64}, load_val::Vector{Int},
                                      reach::Vector{Float64})::Float64
    pd = ctx.pricing_data; base = pd.base; index = ctx.search_index; bi = ctx.cap_bound
    now = label.time
    past_cutoff = now > base.max_wait_time + 1e-9
    cur = index.node_index[label.current]
    activated = label.activated_reward_layers
    layer_weight = base.layer_weight
    scratch = ctx.bound_workspace.layer_scratch
    empty!(scratch)
    # per rider (indexed by p; reset through `touched` on exit):
    vb = bi.v_best; vw = bi.v_wmin     # largest v / smallest Q d over reachable options
    vL = bi.v_live                     # largest v through a live clock
    vN = bi.v_later                    # largest v through a later clock
    vNA = bi.v_later_a; vwNA = bi.v_later_aw  # ... of those that can finish by tstar
    touched = bi.touched; empty!(touched)
    cred = label.credited_set
    lam = base.route_regularization_weight / pd.capacity
    horizon_end = now
    tstar = max(now, base.max_wait_time)
    total = 0.0
    empty!(bi.lrec_p); empty!(bi.lrec_age); empty!(bi.lrec_v); empty!(bi.slot_age); empty!(bi.slot_load)
    @inbounds for t in eachindex(age_idx)
        o = Int(age_idx[t]); age = age_val[t]
        room = pd.capacity - (isempty(load_val) ? 0 : load_val[t])  # F only grows: a full clock stays full
        push!(bi.slot_age, age); push!(bi.slot_load, isempty(load_val) ? 0 : load_val[t])
        for i in index.opps_by_origin_idx[o]
            dest = index.opp_dest_idx[i]
            if bi.opp_critical[i]
                p = bi.opp_p[i]
                p in cred && continue
                q = bi.opp_q[i]
                q <= room || continue
                d = bi.sp[cur, dest]
                age + d <= index.opp_ride_limit[i] + 1e-9 || continue
                r = bi.opp_reward[i]
                if reach[p] == 0.0 && vw[p] == Inf
                    push!(touched, p)
                end
                r > reach[p] && (reach[p] = r)
                v = r - lam * q * d
                horizon_end = max(horizon_end, now + index.opp_ride_limit[i] - age)
                v > 0 || continue
                v > vb[p] && (vb[p] = v)
                q * d < vw[p] && (vw[p] = q * d)
                v > vL[p] && (vL[p] = v)
                push!(bi.lrec_p, p); push!(bi.lrec_age, age); push!(bi.lrec_v, v)
                if age > 1e-9
                    v > bi.v_live_old[p] && (bi.v_live_old[p] = v)
                else
                    v > bi.v_live_new[p] && (bi.v_live_new[p] = v)
                end
            else
                age + index.travel_matrix[cur, dest] <= index.opp_ride_limit[i] + 1e-9 || continue
                issubset(index.opp_layer_mask[i], activated) && continue
                for layer in index.opp_layer_mask[i]
                    (layer in activated || layer in scratch) && continue
                    push!(scratch, layer)
                    total += layer_weight[layer]
                end
            end
        end
    end
    if !past_cutoff
        @inbounds for o in eachindex(bi.noncrit_origin_union_mask)
            if now + index.travel_matrix[cur, o] <= base.max_wait_time + 1e-9
                for layer in bi.noncrit_origin_union_mask[o]
                    (layer in activated || layer in scratch) && continue
                    push!(scratch, layer)
                    total += layer_weight[layer]
                end
            end
            reach_o = now + bi.sp[cur, o]
            reach_o <= base.max_wait_time + 1e-9 || continue
            for i in bi.crit_opps_by_origin_idx[o]
                p = bi.opp_p[i]
                p in cred && continue
                r = bi.opp_reward[i]
                if reach[p] == 0.0 && vw[p] == Inf
                    push!(touched, p)
                end
                r > reach[p] && (reach[p] = r)
                q = bi.opp_q[i]
                d = bi.sp[o, index.opp_dest_idx[i]]
                v = r - lam * q * d
                horizon_end = max(horizon_end, base.max_wait_time + index.opp_ride_limit[i])
                v > 0 || continue
                v > vb[p] && (vb[p] = v)
                q * d < vw[p] && (vw[p] = q * d)
                v > vN[p] && (vN[p] = v)
                if reach_o + d <= tstar + 1e-9
                    v > vNA[p] && (vNA[p] = v)
                    q * d < vwNA[p] && (vwNA[p] = q * d)
                end
            end
        end
    end
    return total + _capacitated_compose!(bi, pd, now, horizon_end, tstar, label.route_length,
                                         isempty(label.arc_load) ? 0 : label.arc_load[end])
end

"""The critical part of the remaining bound from the per-rider values gathered in `bi`
(`touched` riders; see `_capacitated_remaining_bound`, relaxations 1-4); resets them."""
function _capacitated_compose!(bi::CapacitatedBoundIndex, pd::JointRoutingAssignmentCapacitatedPricingData,
                               now::Float64, horizon_end::Float64, tstar::Float64, route_length::Int,
                               last_load::Int)::Float64
    base = pd.base
    touched = bi.touched
    vb = bi.v_best; vw = bi.v_wmin; vL = bi.v_live; vN = bi.v_later; vNA = bi.v_later_a; vwNA = bi.v_later_aw
    unique!(touched)
    C = Float64(pd.capacity)
    vals = bi.kv; empty!(vals); wts = bi.kw; empty!(wts); qs = bi.kq; empty!(qs)
    b = 0.0
    @inbounds for p in touched
        vb[p] > 0 || continue
        b += vb[p]
        push!(vals, vb[p]); push!(wts, vw[p]); push!(qs, Float64(pd.load[p]))
    end
    if !isempty(vals)
        b = min(b, _fractional_knapsack!(bi.korder, vals, wts, C * (horizon_end - now)))
        if base.bounded_max_stops
            b = min(b, _fractional_knapsack!(bi.korder, vals, qs, C * (base.max_stops - route_length)))
        end
        b4 = _knapsack_from!(bi, touched, vL, nothing, pd, C)
        # riders credited later through a clock visited before this stop rode the arc into it,
        # together with the riders credited here (`last_load`): they fit in `C - last_load`
        if last_load > 0 && b4 < b
            b4 = min(b4, _knapsack_from!(bi, touched, bi.v_live_old, nothing, pd, C - last_load) +
                         _knapsack_from!(bi, touched, bi.v_live_new, nothing, pd, C))
        end
        # the same at every live clock `c` with load `F_c > 0`: riders credited later through
        # `c` or an older clock ride every arc since `c`'s visit, so they fit in `C - F_c`
        if b4 < b
            vo = bi.v_tmp_old; vn = bi.v_tmp_new
            @inbounds for k in eachindex(bi.slot_age)
                Fk = bi.slot_load[k]
                (Fk > 0 && bi.slot_age[k] > 1e-9) || continue
                ak = bi.slot_age[k]
                for r in eachindex(bi.lrec_p)
                    p = bi.lrec_p[r]; v = bi.lrec_v[r]
                    if bi.lrec_age[r] >= ak - 1e-9
                        v > vo[p] && (vo[p] = v)
                    else
                        v > vn[p] && (vn[p] = v)
                    end
                end
                bk = _knapsack_from!(bi, touched, vo, nothing, pd, C - Fk) +
                     _knapsack_from!(bi, touched, vn, nothing, pd, C)
                for p in touched
                    vo[p] = 0.0; vn[p] = 0.0
                end
                b4 = min(b4, bk)
            end
        end
        if b4 < b
            b4 += _knapsack_from!(bi, touched, vN, nothing, pd, C)
            b4 < b && (b4 += _knapsack_from!(bi, touched, vNA, vwNA, pd, C * (tstar - now)))
        end
        b = min(b, b4)
    end
    @inbounds for p in touched
        vb[p] = 0.0; vw[p] = Inf; vL[p] = 0.0; vN[p] = 0.0; vNA[p] = 0.0; vwNA[p] = Inf
        bi.v_live_old[p] = 0.0; bi.v_live_new[p] = 0.0
    end
    return b
end

"""
Per-stop scan for scoring many children of one parent at the same next stop at once (they
share clocks, time, position and stop count; they differ in which optional riders are
credited and so in clock loads). Fills `sc` with every critical option of the credits-nobody
child `probe` that passes the tests of `_capacitated_remaining_bound` except the load test
(slot = its clock's index in `age_idx`, 0 for a later clock), and returns the non-critical
part of the bound. `_capacitated_scan_eval!` then gives, for a child with extra credits and
fuller clocks, an upper bound on its remaining reward (its options are a subset of these).
"""
function _capacitated_scan!(sc::CapacitatedScan, ctx, probe::JointRoutingAssignmentCapacitatedPricingLabel,
                            age_idx::Vector{Int32}, age_val::Vector{Float64})::Float64
    pd = ctx.pricing_data; base = pd.base; index = ctx.search_index; bi = ctx.cap_bound
    empty!(sc.p); empty!(sc.slot); empty!(sc.q); empty!(sc.v); empty!(sc.w); empty!(sc.cls)
    now = probe.time
    past_cutoff = now > base.max_wait_time + 1e-9
    cur = index.node_index[probe.current]
    activated = probe.activated_reward_layers
    layer_weight = base.layer_weight
    scratch = ctx.bound_workspace.layer_scratch
    empty!(scratch)
    cred = probe.credited_set
    lam = base.route_regularization_weight / pd.capacity
    horizon_end = now
    tstar = max(now, base.max_wait_time)
    total = 0.0
    @inbounds for t in eachindex(age_idx)
        o = Int(age_idx[t]); age = age_val[t]
        for i in index.opps_by_origin_idx[o]
            dest = index.opp_dest_idx[i]
            if bi.opp_critical[i]
                p = bi.opp_p[i]
                p in cred && continue
                q = bi.opp_q[i]
                q <= pd.capacity || continue
                d = bi.sp[cur, dest]
                age + d <= index.opp_ride_limit[i] + 1e-9 || continue
                horizon_end = max(horizon_end, now + index.opp_ride_limit[i] - age)
                v = bi.opp_reward[i] - lam * q * d
                v > 0 || continue
                push!(sc.p, p); push!(sc.slot, t); push!(sc.q, q); push!(sc.v, v); push!(sc.w, q * d)
                push!(sc.cls, age > 1e-9 ? 0x04 : 0x01)
            else
                age + index.travel_matrix[cur, dest] <= index.opp_ride_limit[i] + 1e-9 || continue
                issubset(index.opp_layer_mask[i], activated) && continue
                for layer in index.opp_layer_mask[i]
                    (layer in activated || layer in scratch) && continue
                    push!(scratch, layer)
                    total += layer_weight[layer]
                end
            end
        end
    end
    if !past_cutoff
        @inbounds for o in eachindex(bi.noncrit_origin_union_mask)
            if now + index.travel_matrix[cur, o] <= base.max_wait_time + 1e-9
                for layer in bi.noncrit_origin_union_mask[o]
                    (layer in activated || layer in scratch) && continue
                    push!(scratch, layer)
                    total += layer_weight[layer]
                end
            end
            reach_o = now + bi.sp[cur, o]
            reach_o <= base.max_wait_time + 1e-9 || continue
            for i in bi.crit_opps_by_origin_idx[o]
                p = bi.opp_p[i]
                p in cred && continue
                q = bi.opp_q[i]
                d = bi.sp[o, index.opp_dest_idx[i]]
                horizon_end = max(horizon_end, base.max_wait_time + index.opp_ride_limit[i])
                v = bi.opp_reward[i] - lam * q * d
                v > 0 || continue
                push!(sc.p, p); push!(sc.slot, 0); push!(sc.q, q); push!(sc.v, v); push!(sc.w, q * d)
                push!(sc.cls, reach_o + d <= tstar + 1e-9 ? 0x03 : 0x02)
            end
        end
    end
    sc.now = now; sc.horizon_end = horizon_end; sc.tstar = tstar; sc.route_length = probe.route_length
    sc.slot_age = copy(age_val)
    return total
end

"""Upper bound on the critical remaining reward of a child scanned by `_capacitated_scan!`,
with extra credited riders `excluded` and clock loads `slot_load` (per scan slot)."""
function _capacitated_scan_eval!(sc::CapacitatedScan, ctx, excluded::BitSet, slot_load::Vector{Int}, last_load::Int)::Float64
    pd = ctx.pricing_data; bi = ctx.cap_bound
    vb = bi.v_best; vw = bi.v_wmin; vL = bi.v_live; vN = bi.v_later; vNA = bi.v_later_a; vwNA = bi.v_later_aw
    touched = bi.touched; empty!(touched)
    C = pd.capacity
    empty!(bi.lrec_p); empty!(bi.lrec_age); empty!(bi.lrec_v); empty!(bi.slot_age); empty!(bi.slot_load)
    append!(bi.slot_age, sc.slot_age); append!(bi.slot_load, slot_load)
    @inbounds for k in eachindex(sc.p)
        p = sc.p[k]
        p in excluded && continue
        s = sc.slot[k]
        s > 0 && (sc.q[k] <= C - slot_load[s] || continue)
        v = sc.v[k]; w = sc.w[k]
        (vb[p] == 0.0 && vw[p] == Inf) && push!(touched, p)
        v > vb[p] && (vb[p] = v)
        w < vw[p] && (vw[p] = w)
        c = sc.cls[k]
        if c == 0x01 || c == 0x04
            v > vL[p] && (vL[p] = v)
            push!(bi.lrec_p, p); push!(bi.lrec_age, sc.slot_age[s]); push!(bi.lrec_v, v)
            if c == 0x04
                v > bi.v_live_old[p] && (bi.v_live_old[p] = v)
            else
                v > bi.v_live_new[p] && (bi.v_live_new[p] = v)
            end
        else
            v > vN[p] && (vN[p] = v)
            if c == 0x03
                v > vNA[p] && (vNA[p] = v)
                w < vwNA[p] && (vwNA[p] = w)
            end
        end
    end
    return _capacitated_compose!(bi, pd, sc.now, sc.horizon_end, sc.tstar, sc.route_length, last_load)
end

"""Fractional knapsack over riders `ps` with values `vals[p] > 0` (weights `wts[p]`, or `Q_p`
when `nothing`)."""
function _knapsack_from!(bi::CapacitatedBoundIndex, ps::Vector{Int}, vals::Vector{Float64}, wts, pd, cap::Float64)::Float64
    va = bi.kv2; empty!(va); wa = bi.kw2; empty!(wa)
    @inbounds for p in ps
        vals[p] > 0 || continue
        push!(va, vals[p]); push!(wa, wts === nothing ? Float64(pd.load[p]) : wts[p])
    end
    isempty(va) && return 0.0
    return _fractional_knapsack!(bi.korder, va, wa, cap)
end

"""Value of the fractional knapsack (items `vals`/`wts`, capacity `cap`); `order` is scratch."""
function _fractional_knapsack!(order::Vector{Int}, vals::Vector{Float64}, wts::Vector{Float64}, cap::Float64)::Float64
    resize!(order, length(vals))
    for i in eachindex(order); order[i] = i; end
    sort!(order; by = i -> wts[i] <= 0 ? -Inf : -vals[i] / wts[i])
    acc = 0.0
    left = max(cap, 0.0)
    @inbounds for i in order
        w = wts[i]
        if w <= left
            acc += vals[i]; left -= w
        else
            acc += vals[i] * left / w
            break
        end
    end
    return acc
end
