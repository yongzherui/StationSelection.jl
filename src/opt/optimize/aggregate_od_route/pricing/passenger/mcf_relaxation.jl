"""
A time-expanded multi-commodity-flow **relaxation** of the passenger
free-assignment pricing problem, used as a cheap termination certificate for
column generation.

# Why

`column_generation.jl` only claims `:optimality_proven` when the exhaustive
certification pass -- an unbounded-`n_candidates` label search -- comes back
empty *and* exhausted. That pass is the part that fails to terminate: pricing
gets harder as the duals converge (43k labels at iteration 1 growing to 1.38M by
iteration 22 on n=20), and a timed-out certification leaves `lp_bound` as no
bound at all.

But the last question CG asks is not "what is the best column?", it is "is there
any column with negative reduced cost?". That only needs a valid **lower bound**
on the minimum reduced cost. This file computes one with a single LP: if

    L_s >= -reduced_cost_tol

then no improving column exists in scenario `s`, and the label search can be
skipped entirely.

# The network

With `beta = route_regularization_weight`, `d(i,j) = travel_cost`,
`W = max_wait_time`, `R_pjk = opp.ride_limit` and `rho_pjk = opp.reward`:

  - time step `delta`, arc time advance `dbar(i,j) = delta*floor(d(i,j)/delta)`,
    horizon `H = W + max_opp R_pjk` (nothing past `H` can certify anything, and
    travelling further only costs money);
  - states `(i, t)` for `t` on the grid, restricted to those actually reachable
    from a start node -- the same forward reachability the label search has;
  - travel arcs `(i,t) -> (j, t + dbar(i,j))` at cost `beta*d(i,j)`, i.e. the
    **true** travel cost, not the rounded one;
  - a source `sigma -> (i, 0)` per start node at cost `beta*repositioning_time`,
    and a sink arc out of every state, so a route may start and end anywhere.

# The variables

  - `f_a >= 0`: one unit of vehicle flow from `sigma` to the sink.
  - `h^{j,b}_a in [0, f_a]`: a **suffix flow** per (origin station `j` carrying
    opportunities, boarding-time bucket `b`).
  - `u_pjk in [0, 1]`: passenger `p` collects the reward of assignment `(j, k)`.

`sum_{(j,k)} u_pjk <= 1` is exactly the per-passenger *maximum* semantics that
the label search encodes with reward layers; here it needs no layers at all.

# Why the suffix flow, and why it is valid

For a route `r` with real arrival times `T_0 = 0 < T_1 < ... < T_L`, set `f` to
the path's incidence vector, `u` to `r`'s per-passenger argmax assignments, and
`h^{j,b}` to the path *suffix* starting at the first visit to `j` inside bucket
`b`. Grid times satisfy `t_m = sum_{i<m} dbar <= sum_{i<m} d = T_m`, hence

  - pickup window: `t_1 <= T_1 <= W`;
  - ride limit: `t_k - t_j = sum_{j<=i<k} dbar <= sum d = T_k - T_j <= R_pjk`.

Both endpoints of the ride are cumulative sums over the *same* arcs, so flooring
does **not** accumulate drift here -- which is why travel times are rounded down
rather than up, and why no window inflation is needed anywhere.

That solution's objective is exactly `beta*(tau_r + repo) - sum_p rho_p = rc_r`,
so `L_s <= min_r rc_r` for every route the label search could return.

A suffix flow rather than an ordinary origin-to-destination commodity is what
makes the coupling `u_pjk <= inflow of h at (k, t2)` simultaneously tight for
*every* downstream node of the path: one unit of flow routed to a single sink
could only certify one dropoff, which would under-count reward and break
validity in the dangerous direction (an over-estimated `L_s` could wrongly
certify).

# Where it is loose

  1. Fractional `f` -- a convex combination of paths collects reward from
     several paths at once, limited only by `h <= f`. Not tunable; this is the
     main gap.
  2. Bucket deadlines -- `sup(b) + R_pjk` instead of `t_1 + R_pjk`. Tightened by
     `n_boarding_buckets`, exact at one bucket per grid step.
  3. Grid coarseness -- only ever makes the vehicle optimistically fast.

None of these can invalidate the certificate; they can only stop it from firing.

# Preconditions and guards

`delta <= min_{i != j} d(i,j)` is required: otherwise `dbar = 0` for the
shortest arc, the network stops being a DAG, and `h` admits circulations that
manufacture reward. `config.time_step` is therefore an *upper* bound on the step
actually used -- the builder clamps it down and reports the effective value.

Because the clamp can force a fine grid, `config.max_arcs` bounds the network
size; over it, the function declines to certify rather than building a model
that costs more than the label search it is meant to replace. Every failure path
returns `(-Inf, false, ...)`: a bound that was not computed must never certify.
"""

export PassengerMCFRelaxationConfig
export passenger_free_assignment_mcf_lower_bound

"""
    PassengerMCFRelaxationConfig(; kwargs...)

  - `enabled`: master switch; off by default, like every other pricing variant here.
  - `time_step`: *upper bound* on the grid step. `<= 0` means "use the smallest
    positive travel time", which is the coarsest grid the DAG argument allows.
  - `n_boarding_buckets`: how finely boarding times are bucketed. `1` is cheapest
    and loosest; more buckets tighten the ride-limit deadlines.
  - `lp_time_limit_sec`: Gurobi time limit for the relaxation LP.
  - `max_arcs`: refuse to build networks whose arc count (times commodities)
    exceeds this.
  - `use_as_certificate`: whether a non-negative bound is allowed to skip the
    label search. `false` computes and logs the bound without changing pricing,
    which is how it should be measured first.
"""
struct PassengerMCFRelaxationConfig
    enabled::Bool
    time_step::Float64
    n_boarding_buckets::Int
    lp_time_limit_sec::Float64
    max_arcs::Int
    use_as_certificate::Bool

    function PassengerMCFRelaxationConfig(;
            enabled::Bool=false,
            time_step::Float64=0.0,
            n_boarding_buckets::Int=1,
            lp_time_limit_sec::Float64=60.0,
            max_arcs::Int=4_000_000,
            use_as_certificate::Bool=true,
        )
        n_boarding_buckets > 0 || throw(ArgumentError("n_boarding_buckets must be positive"))
        lp_time_limit_sec > 0 || throw(ArgumentError("lp_time_limit_sec must be positive"))
        max_arcs > 0 || throw(ArgumentError("max_arcs must be positive"))
        new(enabled, time_step, n_boarding_buckets, lp_time_limit_sec, max_arcs, use_as_certificate)
    end
end

"""
The time-expanded network. Times are held as **integer grid indices** (`t =
index * time_step`) so that reachability and deadline comparisons are exact
integer arithmetic rather than repeated float rounding.

`arc_tail`/`arc_head` use `0` for the source and the sink respectively, so an
arc with `arc_tail[a] == 0` leaves `sigma` and one with `arc_head[a] == 0`
enters the sink.
"""
struct PassengerMCFNetwork
    nodes::Vector{Int}
    node_index::Dict{Int, Int}
    time_step::Float64
    n_layers::Int
    state_node::Vector{Int}
    state_time::Vector{Int}
    state_id::Dict{Tuple{Int, Int}, Int}
    arc_tail::Vector{Int}
    arc_head::Vector{Int}
    arc_cost::Vector{Float64}
    in_arcs::Vector{Vector{Int}}
    out_arcs::Vector{Vector{Int}}
    source_arcs::Vector{Int}
    travel_arcs::Vector{Int}
    sink_arcs::Vector{Int}
end

"""
Stations the network needs: the origins and destinations of the surviving
opportunities. A station that is neither can never certify anything, so routing
through it only burns time and money -- which is also why the label search's
`_passenger_free_assignment_candidate_next_nodes` never proposes one. Using the
full endpoint set (rather than the label search's tighter, state-dependent
prune) keeps this a superset of the exact search's route universe, as a
relaxation requires.
"""
function _passenger_mcf_endpoint_nodes(pricing_data::PassengerFreeAssignmentPricingData)::Vector{Int}
    endpoints = Set{Int}()
    for opp in pricing_data.opportunities
        push!(endpoints, opp.origin)
        push!(endpoints, opp.destination)
    end
    return [node for node in pricing_data.nodes if node in endpoints]
end

function _build_passenger_mcf_network(
    pricing_data::PassengerFreeAssignmentPricingData,
    config::PassengerMCFRelaxationConfig,
)
    nodes = _passenger_mcf_endpoint_nodes(pricing_data)
    length(nodes) >= 2 || return nothing, :too_few_nodes
    node_index = Dict(node => i for (i, node) in enumerate(nodes))
    n = length(nodes)

    # Dense travel matrix over the endpoint nodes. A missing arc stays `Inf` and
    # is simply never expanded -- unlike `_passenger_free_assignment_travel`,
    # which throws, because here an unreachable pair is a legitimately absent arc.
    travel = fill(Inf, n, n)
    min_positive = Inf
    for (i, u) in enumerate(nodes), (j, v) in enumerate(nodes)
        i == j && continue
        cost = get(pricing_data.travel_cost, (u, v), Inf)
        isfinite(cost) || continue
        travel[i, j] = cost
        cost > 0 && (min_positive = min(min_positive, cost))
    end
    isfinite(min_positive) || return nothing, :no_positive_travel

    # `time_step` is an upper bound only: a step longer than the shortest arc
    # would floor that arc to zero, giving same-layer cycles that the suffix
    # flows could circulate on.
    step = config.time_step > 0 ? min(config.time_step, min_positive) : min_positive
    step > 0 || return nothing, :degenerate_time_step

    max_ride_limit = maximum((opp.ride_limit for opp in pricing_data.opportunities); init=0.0)
    horizon = pricing_data.max_wait_time + max_ride_limit
    n_layers = Int(floor(horizon / step)) + 1
    n_layers >= 1 || return nothing, :empty_horizon

    advance = zeros(Int, n, n)
    for i in 1:n, j in 1:n
        i == j && continue
        isfinite(travel[i, j]) || continue
        advance[i, j] = Int(floor(travel[i, j] / step))
        # Guaranteed by `step <= min_positive`, but assert rather than trust it:
        # a zero-advance arc silently invalidates every bound this file returns.
        advance[i, j] >= 1 || return nothing, :zero_advance_arc
    end

    max_arcs_on_route = pricing_data.bounded_max_stops ? pricing_data.max_stops - 1 : typemax(Int)
    max_arcs_on_route >= 0 || return nothing, :max_stops_too_small

    # Forward reachability, tracking the *fewest* arcs needed to reach a state so
    # that a `max_stops` cap prunes correctly.
    state_id = Dict{Tuple{Int, Int}, Int}()
    state_node = Int[]
    state_time = Int[]
    state_depth = Int[]
    queue = Int[]

    function push_state!(node_idx::Int, time_idx::Int, depth::Int)
        key = (node_idx, time_idx)
        existing = get(state_id, key, 0)
        if existing == 0
            push!(state_node, node_idx)
            push!(state_time, time_idx)
            push!(state_depth, depth)
            id = length(state_node)
            state_id[key] = id
            push!(queue, id)
            return id
        end
        if depth < state_depth[existing]
            state_depth[existing] = depth
            push!(queue, existing)
        end
        return existing
    end

    # Routes start at any opportunity endpoint at t = 0, matching
    # `initial_passenger_free_assignment_pricing_labels`.
    for i in 1:n
        push_state!(i, 0, 0)
    end

    head = 1
    while head <= length(queue)
        id = queue[head]
        head += 1
        length(state_node) > config.max_arcs && return nothing, :too_large
        depth = state_depth[id]
        depth >= max_arcs_on_route && continue
        i = state_node[id]
        t = state_time[id]
        for j in 1:n
            i == j && continue
            advance[i, j] == 0 && continue
            t2 = t + advance[i, j]
            t2 <= n_layers - 1 || continue
            push_state!(j, t2, depth + 1)
        end
    end

    n_states = length(state_node)
    arc_tail = Int[]
    arc_head = Int[]
    arc_cost = Float64[]
    in_arcs = [Int[] for _ in 1:n_states]
    out_arcs = [Int[] for _ in 1:n_states]
    source_arcs = Int[]
    travel_arcs = Int[]
    sink_arcs = Int[]

    repositioning_cost = pricing_data.route_regularization_weight * pricing_data.repositioning_time

    function add_arc!(tail::Int, head_state::Int, cost::Float64)
        push!(arc_tail, tail)
        push!(arc_head, head_state)
        push!(arc_cost, cost)
        a = length(arc_tail)
        tail != 0 && push!(out_arcs[tail], a)
        head_state != 0 && push!(in_arcs[head_state], a)
        return a
    end

    for id in 1:n_states
        state_time[id] == 0 || continue
        push!(source_arcs, add_arc!(0, id, repositioning_cost))
    end
    for id in 1:n_states
        i = state_node[id]
        t = state_time[id]
        depth = state_depth[id]
        push!(sink_arcs, add_arc!(id, 0, 0.0))
        depth >= max_arcs_on_route && continue
        for j in 1:n
            i == j && continue
            advance[i, j] == 0 && continue
            t2 = t + advance[i, j]
            t2 <= n_layers - 1 || continue
            target = get(state_id, (j, t2), 0)
            target == 0 && continue
            push!(travel_arcs, add_arc!(
                id, target, pricing_data.route_regularization_weight * travel[i, j],
            ))
        end
        length(arc_tail) > config.max_arcs && return nothing, :too_large
    end

    net = PassengerMCFNetwork(
        nodes, node_index, step, n_layers, state_node, state_time, state_id,
        arc_tail, arc_head, arc_cost, in_arcs, out_arcs,
        source_arcs, travel_arcs, sink_arcs,
    )
    return net, :ok
end

"""
Boarding-time buckets as grid-index ranges over `[0, floor(W / delta)]`, split
as evenly as `n_boarding_buckets` allows. Returns `(lo, hi)` index pairs; a
commodity's ride-limit deadlines are measured from `hi`, which is what makes a
coarse bucketing a relaxation rather than a restriction.
"""
function _passenger_mcf_boarding_buckets(
    net::PassengerMCFNetwork, max_wait_time::Float64, n_buckets::Int,
)::Vector{Tuple{Int, Int}}
    last_index = min(net.n_layers - 1, Int(floor(max_wait_time / net.time_step)))
    last_index >= 0 || return Tuple{Int, Int}[]
    total = last_index + 1
    k = min(n_buckets, total)
    buckets = Tuple{Int, Int}[]
    lo = 0
    for b in 1:k
        width = div(total - lo, k - b + 1)
        push!(buckets, (lo, lo + width - 1))
        lo += width
    end
    return buckets
end

"""
    passenger_free_assignment_mcf_lower_bound(pricing_data, optimizer_env; config, silent)
        -> (lower_bound, certified, stats)

`lower_bound <= min_r rc_r` over every route the exact pricer could return;
`certified` is `lower_bound >= -reduced_cost_tol`, i.e. "no improving column
exists in this scenario".

Every path that fails to produce a proven bound -- a degenerate network, a size
guard trip, a non-optimal LP -- returns `(-Inf, false, ...)` with the reason in
`stats.reason`, so a caller can never mistake a missing bound for a certificate.
"""
function passenger_free_assignment_mcf_lower_bound(
    pricing_data::PassengerFreeAssignmentPricingData,
    optimizer_env;
    config::PassengerMCFRelaxationConfig=PassengerMCFRelaxationConfig(),
    reduced_cost_tol::Float64=1e-6,
    silent::Bool=true,
)
    failure(reason; build_sec=0.0, solve_sec=0.0) = (-Inf, false, (
        reason=reason, effective_time_step=NaN, n_time_layers=0, n_states=0,
        n_arcs=0, n_commodities=0, n_opportunities=length(pricing_data.opportunities),
        build_sec=build_sec, solve_sec=solve_sec, termination_status=nothing,
    ))

    config.enabled || return failure(:disabled)

    if isempty(pricing_data.opportunities)
        # No positive-reward assignment exists, so every route has
        # `rc = beta*(tau + repo) >= 0`. Certified without building anything.
        return 0.0, true, (
            reason=:no_opportunities, effective_time_step=NaN, n_time_layers=0,
            n_states=0, n_arcs=0, n_commodities=0, n_opportunities=0,
            build_sec=0.0, solve_sec=0.0, termination_status=nothing,
        )
    end

    t_build = time()
    net, reason = _build_passenger_mcf_network(pricing_data, config)
    isnothing(net) && return failure(reason; build_sec=time() - t_build)

    buckets = _passenger_mcf_boarding_buckets(
        net, pricing_data.max_wait_time, config.n_boarding_buckets,
    )
    isempty(buckets) && return failure(:no_boarding_window; build_sec=time() - t_build)

    n_states = length(net.state_node)
    n_arcs = length(net.arc_tail)

    # One suffix-flow commodity per (origin station, boarding bucket) that the
    # vehicle can actually be at during that bucket.
    origins = sort!(unique(opp.origin for opp in pricing_data.opportunities))
    max_ride_by_origin = Dict{Int, Float64}()
    for opp in pricing_data.opportunities
        max_ride_by_origin[opp.origin] = max(get(max_ride_by_origin, opp.origin, 0.0), opp.ride_limit)
    end

    commodity_origin = Int[]
    commodity_lo = Int[]
    commodity_hi = Int[]
    commodity_deadline = Int[]
    commodity_key = Dict{Tuple{Int, Int}, Int}()
    for origin in origins
        origin_idx = get(net.node_index, origin, 0)
        origin_idx == 0 && continue
        for (b, (lo, hi)) in enumerate(buckets)
            any(haskey(net.state_id, (origin_idx, t)) for t in lo:hi) || continue
            # Nothing past the latest deadline this origin can serve is useful:
            # the suffix flow exits via a sink arc instead.
            deadline = min(
                net.n_layers - 1,
                hi + Int(floor(get(max_ride_by_origin, origin, 0.0) / net.time_step)),
            )
            push!(commodity_origin, origin_idx)
            push!(commodity_lo, lo)
            push!(commodity_hi, hi)
            push!(commodity_deadline, deadline)
            commodity_key[(origin, b)] = length(commodity_origin)
        end
    end
    n_commodities = length(commodity_origin)
    n_commodities > 0 || return failure(:no_commodities; build_sec=time() - t_build)

    if n_arcs * (n_commodities + 1) > config.max_arcs
        return failure(:too_large; build_sec=time() - t_build)
    end

    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    silent && set_silent(m)
    set_optimizer_attribute(m, "TimeLimit", config.lp_time_limit_sec)

    f = @variable(m, [1:n_arcs], lower_bound = 0.0, upper_bound = 1.0)

    # (1) one route.
    @constraint(m, sum(f[a] for a in net.source_arcs) == 1.0)
    # (2) vehicle-flow conservation; every state has a sink arc, so a route may end anywhere.
    for id in 1:n_states
        @constraint(m, sum(f[a] for a in net.in_arcs[id]) == sum(f[a] for a in net.out_arcs[id]))
    end
    # (3) a route visits at most `max_stops` stations, i.e. traverses at most one fewer arcs.
    if pricing_data.bounded_max_stops
        @constraint(m, sum(f[a] for a in net.travel_arcs) <= pricing_data.max_stops - 1)
    end
    # (4) per-station visit cap, summed over the whole time axis.
    if pricing_data.max_visits_per_node < typemax(Int)
        states_by_node = [Int[] for _ in 1:length(net.nodes)]
        for id in 1:n_states
            push!(states_by_node[net.state_node[id]], id)
        end
        for ids in states_by_node
            isempty(ids) && continue
            @constraint(m, sum(f[a] for id in ids for a in net.in_arcs[id]) <=
                pricing_data.max_visits_per_node)
        end
    end

    # Suffix flows. Each commodity lives only inside its own time window: arcs
    # outside it can carry no reward for it, and dropping them is safe because
    # the flow can always leave through a sink arc instead.
    inflow_expr = Vector{Dict{Int, AffExpr}}(undef, n_commodities)
    injection_expr = Vector{AffExpr}(undef, n_commodities)
    for c in 1:n_commodities
        lo = commodity_lo[c]
        hi = commodity_hi[c]
        deadline = commodity_deadline[c]
        origin_idx = commodity_origin[c]

        # The commodity lives only inside `[lo, deadline]`. Arcs outside it can
        # carry no reward for this commodity, and dropping them is safe because
        # the flow can always leave through a sink arc instead of continuing.
        arcs = Int[]
        for a in net.travel_arcs
            net.state_time[net.arc_tail[a]] >= lo || continue
            net.state_time[net.arc_head[a]] <= deadline || continue
            push!(arcs, a)
        end
        for a in net.sink_arcs
            lo <= net.state_time[net.arc_tail[a]] <= deadline || continue
            push!(arcs, a)
        end

        in_c = Dict{Int, AffExpr}()
        out_c = Dict{Int, AffExpr}()
        for a in arcs
            v = @variable(m, lower_bound = 0.0)
            # (5) the suffix rides on the vehicle's own flow.
            @constraint(m, v <= f[a])
            tail = net.arc_tail[a]
            head_state = net.arc_head[a]
            tail != 0 && add_to_expression!(get!(() -> AffExpr(0.0), out_c, tail), 1.0, v)
            head_state != 0 && add_to_expression!(get!(() -> AffExpr(0.0), in_c, head_state), 1.0, v)
        end
        inflow_expr[c] = in_c

        injections = AffExpr(0.0)
        for id in union(keys(in_c), keys(out_c))
            inflow = get(in_c, id, AffExpr(0.0))
            outflow = get(out_c, id, AffExpr(0.0))
            if net.state_node[id] == origin_idx && lo <= net.state_time[id] <= hi
                # (7) a boarding node: the vehicle may pick the passenger up here,
                # injecting the suffix, but only while it is actually present.
                b_var = @variable(m, lower_bound = 0.0)
                @constraint(m, b_var <= sum(f[a] for a in net.in_arcs[id]))
                @constraint(m, outflow - inflow == b_var)
                add_to_expression!(injections, 1.0, b_var)
            else
                # (6) conservation everywhere else, which is what keeps the suffix
                # on the vehicle's own path rather than teleporting.
                @constraint(m, outflow == inflow)
            end
        end
        # One boarding per commodity: a second would need more than the single
        # unit of vehicle flow that `h <= f` makes available.
        @constraint(m, injections <= 1.0)
        injection_expr[c] = injections
    end

    # Splitting the boarding window into more buckets tightens each commodity's
    # deadline but also hands the LP one *independent* injection per bucket,
    # which pulls the other way -- measured, that made an 8-bucket bound looser
    # than a 1-bucket one. Tying the total number of injections at a station to
    # the number of times the vehicle actually stops there removes most of that
    # slack: a real route injects at most once per bucket containing a visit,
    # and the number of such buckets is at most the number of visits.
    if n_commodities > length(origins)
        commodities_by_origin = Dict{Int, Vector{Int}}()
        for c in 1:n_commodities
            push!(get!(() -> Int[], commodities_by_origin, commodity_origin[c]), c)
        end
        visits = [AffExpr(0.0) for _ in 1:length(net.nodes)]
        for id in 1:n_states
            for a in net.in_arcs[id]
                add_to_expression!(visits[net.state_node[id]], 1.0, f[a])
            end
        end
        for (origin_idx, cs) in commodities_by_origin
            length(cs) > 1 || continue
            @constraint(m, sum(injection_expr[c] for c in cs) <= visits[origin_idx])
        end
    end

    # (8)/(9) reward coupling and the per-passenger maximum.
    n_opps = length(pricing_data.opportunities)
    u = @variable(m, [1:n_opps], lower_bound = 0.0, upper_bound = 1.0)
    for (i, opp) in enumerate(pricing_data.opportunities)
        dest_idx = get(net.node_index, opp.destination, 0)
        if dest_idx == 0
            @constraint(m, u[i] == 0.0)
            continue
        end
        arrival = AffExpr(0.0)
        for (b, (lo, hi)) in enumerate(buckets)
            c = get(commodity_key, (opp.origin, b), 0)
            c == 0 && continue
            deadline_idx = min(
                net.n_layers - 1, hi + Int(floor(opp.ride_limit / net.time_step)),
            )
            for (id, expr) in inflow_expr[c]
                net.state_node[id] == dest_idx || continue
                net.state_time[id] <= deadline_idx || continue
                add_to_expression!(arrival, 1.0, expr)
            end
        end
        @constraint(m, u[i] <= arrival)
    end
    opps_by_passenger = Dict{Int, Vector{Int}}()
    for (i, opp) in enumerate(pricing_data.opportunities)
        push!(get!(() -> Int[], opps_by_passenger, opp.passenger), i)
    end
    for (_p, idxs) in opps_by_passenger
        @constraint(m, sum(u[i] for i in idxs) <= 1.0)
    end

    @objective(m, Min,
        sum(net.arc_cost[a] * f[a] for a in 1:n_arcs) -
        sum(pricing_data.opportunities[i].reward * u[i] for i in 1:n_opps),
    )

    build_sec = time() - t_build
    t_solve = time()
    optimize!(m)
    solve_sec = time() - t_solve
    status = termination_status(m)

    stats = (
        reason=:ok, effective_time_step=net.time_step, n_time_layers=net.n_layers,
        n_states=n_states, n_arcs=n_arcs, n_commodities=n_commodities,
        n_opportunities=n_opps, build_sec=build_sec, solve_sec=solve_sec,
        termination_status=status,
    )
    if status != MOI.OPTIMAL
        return -Inf, false, merge(stats, (reason=:lp_not_optimal,))
    end
    bound = objective_value(m)
    return bound, bound >= -reduced_cost_tol, stats
end
