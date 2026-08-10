"""
Certified station-subset branch-and-bound for passenger free-assignment pricing.

All bounds and incumbents in this file use *reduced profit*, i.e.
`profit = assignment reward - routing cost = -reduced_cost`.  In particular an
improving column has positive `value`.  LP rounding is used only to select calls
to the exact oracle; only an exhausted label search can update the incumbent.
"""

struct ExactPricingResult
    value::Float64
    reduced_cost::Float64
    route::Vector{Int}
    assignments::Vector{Tuple{Int,Int,Int}}
    station_set::BitSet
    certified::Bool
    labels_generated::Int
    labels_dominated::Int
    runtime_sec::Float64
end

struct PassengerRewardBoundData
    stations::Vector{Int}
    station_index::Dict{Int,Int}
    passengers::Vector{Int}
    assignments::Vector{PassengerAssignmentCandidate}
    by_passenger::Dict{Int,Vector{PassengerAssignmentCandidate}}
    aggregate_reward_impact::Vector{Float64}
end

struct PairwiseAssignmentRoutingBounds
    assignment_indices::Vector{Int}
    conflicts::Vector{Tuple{Int,Int}}
    joint_cost_lower_bounds::Vector{Tuple{Int,Int,Float64}}
end

"""
Compute certified two-assignment routing information. Every feasible route
induces one of the enumerated pickup/dropoff event orders. Replacing the route
segments between consecutive events by shortest-path costs can only reduce
travel and elapsed time, so the minimum enumerated cost is a lower bound. If no
order survives, the assignments are incompatible.

Only the highest-reward `alternatives_per_passenger` assignments are paired.
Omitted cuts weaken the relaxation but never affect validity.
"""
function build_pairwise_assignment_routing_bounds(
    pricing::PassengerFreeAssignmentPricingData,
    reward::PassengerRewardBoundData;
    alternatives_per_passenger::Int=10,
)::PairwiseAssignmentRoutingBounds
    alternatives_per_passenger >= 0 || throw(ArgumentError("alternatives_per_passenger must be nonnegative"))
    selected = Int[]
    for p in reward.passengers
        ids = findall(a -> a.passenger == p, reward.assignments)
        sort!(ids; by=i -> (-reward.assignments[i].reward,
                            reward.assignments[i].origin,
                            reward.assignments[i].destination))
        append!(selected, ids[1:min(alternatives_per_passenger,length(ids))])
    end
    conflicts = Tuple{Int,Int}[]
    costs = Tuple{Int,Int,Float64}[]
    beta = pricing.route_regularization_weight
    for ii in 1:max(0,length(selected)-1), jj in (ii+1):length(selected)
        i,j = selected[ii],selected[jj]
        a,b = reward.assignments[i],reward.assignments[j]
        a.passenger == b.passenger && continue
        events = ((a.origin,1,true),(a.destination,1,false),
                  (b.origin,2,true),(b.destination,2,false))
        best = Inf
        for order in permutations(1:4)
            findfirst(==(1),order) < findfirst(==(2),order) || continue
            findfirst(==(3),order) < findfirst(==(4),order) || continue
            elapsed=0.0; travel=0.0; pickup=fill(NaN,2); feasible=true
            previous=events[order[1]][1]
            for pos in 1:4
                station,which,is_pickup = events[order[pos]]
                if pos > 1 && station != previous
                    leg=get(pricing.travel_cost,(previous,station),Inf)
                    if !isfinite(leg) || leg < 0
                        feasible=false; break
                    end
                    elapsed += leg; travel += leg
                end
                if is_pickup
                    if elapsed > pricing.max_wait_time + 1e-9
                        feasible=false; break
                    end
                    pickup[which]=elapsed
                else
                    limit = which == 1 ? a.ride_limit : b.ride_limit
                    if isnan(pickup[which]) || elapsed-pickup[which] > limit + 1e-9
                        feasible=false; break
                    end
                end
                previous=station
            end
            feasible && (best=min(best,beta*travel))
        end
        isfinite(best) ? push!(costs,(i,j,best)) : push!(conflicts,(i,j))
    end
    PairwiseAssignmentRoutingBounds(selected,conflicts,costs)
end

struct TripleAssignmentRoutingBounds
    conflicts::Vector{Tuple{Int,Int,Int}}
    joint_cost_lower_bounds::Vector{Tuple{Int,Int,Int,Float64}}
end

"""
Minimum certified routing cost (`beta*travel`) over feasible pickup/dropoff event
orderings for a set of assignments, or `Inf` if no order survives (jointly
infeasible). Shortest-path leg costs make this a valid lower bound on any real
route serving them, for ANY `max_stops` -- the bound constrains the route's
reward-collection structure, not its length, so it lifts to every larger cap.
"""
function _min_joint_route_cost(pricing::PassengerFreeAssignmentPricingData,
                               assignments::AbstractVector{PassengerAssignmentCandidate})::Float64
    m = length(assignments)
    beta = pricing.route_regularization_weight
    events = Tuple{Int,Int,Bool}[]   # (station, which, is_pickup)
    for (w, a) in enumerate(assignments)
        push!(events, (a.origin, w, true)); push!(events, (a.destination, w, false))
    end
    best = Inf
    for order in permutations(1:2m)
        precedence_ok = true
        for w in 1:m
            pi = findfirst(t -> events[t][2] == w && events[t][3], order)
            di = findfirst(t -> events[t][2] == w && !events[t][3], order)
            if pi > di
                precedence_ok = false; break
            end
        end
        precedence_ok || continue
        elapsed = 0.0; travel = 0.0; pickup = fill(NaN, m); feasible = true
        previous = events[order[1]][1]
        for pos in 1:2m
            station, which, is_pickup = events[order[pos]]
            if pos > 1 && station != previous
                leg = get(pricing.travel_cost, (previous, station), Inf)
                if !isfinite(leg) || leg < 0
                    feasible = false; break
                end
                elapsed += leg; travel += leg
            end
            if is_pickup
                if elapsed > pricing.max_wait_time + 1e-9
                    feasible = false; break
                end
                pickup[which] = elapsed
            else
                if isnan(pickup[which]) || elapsed - pickup[which] > assignments[which].ride_limit + 1e-9
                    feasible = false; break
                end
            end
            previous = station
        end
        feasible && (best = min(best, beta * travel))
    end
    return best
end

"""
Certified three-assignment routing cuts, the k=3 generalization of
`build_pairwise_assignment_routing_bounds`. Only triples of assignments from
three DISTINCT passengers are considered (same-passenger pairs are already
covered by the per-passenger `sum(x) <= 1` constraint). Valid for every
`max_stops`. Omitted triples weaken the relaxation but never affect validity.
"""
function build_triple_assignment_routing_bounds(
    pricing::PassengerFreeAssignmentPricingData,
    reward::PassengerRewardBoundData;
    alternatives_per_passenger::Int=3,
)::TripleAssignmentRoutingBounds
    alternatives_per_passenger >= 0 || throw(ArgumentError("alternatives_per_passenger must be nonnegative"))
    selected = Int[]
    for p in reward.passengers
        ids = findall(a -> a.passenger == p, reward.assignments)
        sort!(ids; by=i -> (-reward.assignments[i].reward,
                            reward.assignments[i].origin,
                            reward.assignments[i].destination))
        append!(selected, ids[1:min(alternatives_per_passenger, length(ids))])
    end
    conflicts = Tuple{Int,Int,Int}[]
    costs = Tuple{Int,Int,Int,Float64}[]
    for ii in 1:max(0, length(selected) - 2), jj in (ii+1):(length(selected)-1), kk in (jj+1):length(selected)
        i, j, k = selected[ii], selected[jj], selected[kk]
        a, b, c = reward.assignments[i], reward.assignments[j], reward.assignments[k]
        (a.passenger == b.passenger || a.passenger == c.passenger || b.passenger == c.passenger) && continue
        best = _min_joint_route_cost(pricing, [a, b, c])
        isfinite(best) ? push!(costs, (i, j, k, best)) : push!(conflicts, (i, j, k))
    end
    TripleAssignmentRoutingBounds(conflicts, costs)
end

function PassengerRewardBoundData(data::PassengerFreeAssignmentPricingData)
    # opportunities contain exactly the safely removable positive-reward inputs.
    assignments = [PassengerAssignmentCandidate(o.passenger, o.origin, o.destination,
                                                  o.ride_limit, o.reward)
                   for o in data.opportunities]
    byp = Dict{Int,Vector{PassengerAssignmentCandidate}}()
    index = Dict(s => i for (i, s) in enumerate(data.nodes))
    impact = zeros(length(data.nodes))
    for a in assignments
        push!(get!(() -> PassengerAssignmentCandidate[], byp, a.passenger), a)
        impact[index[a.origin]] += a.reward
        a.destination == a.origin || (impact[index[a.destination]] += a.reward)
    end
    PassengerRewardBoundData(copy(data.nodes), index, sort!(collect(keys(byp))),
                             assignments, byp, impact)
end

function reward_upper_bound_fixed(data::PassengerRewardBoundData, stations::BitSet)::Float64
    sum(data.passengers; init=0.0) do p
        maximum((a.reward for a in data.by_passenger[p]
                 if a.origin in stations && a.destination in stations); init=0.0)
    end
end

reward_upper_bound_all_available(data::PassengerRewardBoundData, stations::BitSet) =
    reward_upper_bound_fixed(data, stations)

"""
Run the route-feasibility label search to exhaustion on one station set.

`use_reduced_cost_pruning` enables the admissible completion-reward bound to prune
partial labels that cannot complete into an improving column (reduced cost
< -tol). It is the same bound production CG pricing trusts by default; pruning only
discards labels that provably cannot beat 0, so the certified optimum is unchanged
while long non-improving routes are cut early. `use_post_w_completion_bound` layers
on the exact post-wait-window completion (stronger, but a solve per past-cutoff
label).
"""
function price_exact_on_stations(
    data::PassengerFreeAssignmentPricingData,
    allowed_stations::BitSet;
    warm_start=nothing,
    time_limit::Float64=Inf,
    reduced_cost_tol::Float64=1e-9,
    use_reduced_cost_pruning::Bool=true,
    use_post_w_completion_bound::Bool=false,
    settings=nothing,
)::ExactPricingResult
    t0 = time()
    allowed_nodes = [station for station in data.nodes if station in allowed_stations]
    length(allowed_nodes) == length(allowed_stations) || throw(ArgumentError(
        "allowed_stations contains an ID that is not in pricing_data.nodes"))
    allowed = Set(allowed_nodes)
    candidates = PassengerAssignmentCandidate[
        PassengerAssignmentCandidate(o.passenger, o.origin, o.destination, o.ride_limit, o.reward)
        for o in data.opportunities if o.origin in allowed && o.destination in allowed
    ]
    restricted = create_passenger_free_assignment_pricing_data(
        data.scenario, allowed_nodes, data.travel_cost, candidates;
        route_regularization_weight=data.route_regularization_weight,
        max_wait_time=data.max_wait_time,
        repositioning_time=data.repositioning_time,
        max_stops=data.bounded_max_stops ? data.max_stops : typemax(Int),
        max_visits_per_node=data.max_visits_per_node,
        max_distinct_stations=typemax(Int),
        compensated_dominance=data.compensated_dominance,
    )
    labels, exhausted, stats = _enumerate_passenger_free_assignment_pricing_labels(
        restricted; time_limit=time_limit, reduced_cost_tol=reduced_cost_tol,
        max_visits_per_node=restricted.max_visits_per_node,
        use_reduced_cost_pruning=use_reduced_cost_pruning,
        use_post_w_completion_bound=use_post_w_completion_bound,
        stop_if=_ -> false,
    )
    best_rc = 0.0
    best_route = Int[]
    best_assignments = Tuple{Int,Int,Int}[]
    for label in labels
        assignments, _tau, rc = _passenger_free_assignment_column_from_route(
            label.route, restricted; label_reduced_cost=label.reduced_cost)
        if rc < best_rc - reduced_cost_tol
            best_rc, best_route, best_assignments = rc, copy(label.route), assignments
        end
    end
    ExactPricingResult(-best_rc, best_rc, best_route, best_assignments, copy(allowed_stations),
        exhausted, stats.labels_generated,
        stats.labels_rejected_by_dominance + stats.labels_removed_by_dominance, time()-t0)
end

Base.@kwdef struct StationSubsetPricingSettings
    budget_mode::Symbol = :exact       # :exact or :at_most
    bound_tolerance::Float64 = 1e-7
    reduced_cost_tolerance::Float64 = 1e-9
    time_limit::Float64 = Inf
    node_limit::Int = typemax(Int)
    exact_oracle_time_limit::Float64 = Inf
    node_order::Symbol = :best_bound   # :best_bound or :depth_first
    branching_rule::Symbol = :most_fractional
    use_reward_lp::Bool = true
    # Search service-station envelopes rather than L-station availability sets.
    # If route feasibility proves every route visits at most K<L distinct
    # stations, monotonicity makes exact-K search globally equivalent.
    use_route_station_cap::Bool = true
    # Strengthen the routing-free reward model by making station selections
    # integral. It remains a certified upper bound, but is a MILP rather than LP.
    integral_reward_stations::Bool = false
    use_routing_reward_bound::Bool = true
    use_pairwise_routing_bounds::Bool = true
    pairwise_alternatives_per_passenger::Int = 10
    # k=3 generalization of the pairwise routing cuts (valid for any max_stops).
    use_triple_routing_bounds::Bool = false
    triple_alternatives_per_passenger::Int = 3
    use_single_assignment_cost_bound::Bool = false
    # Column-generation mode: return as soon as an exact fixed-subset solve
    # proves an improving column exists.  This is a feasibility early exit, not
    # a global pricing certificate; the returned global upper bound remains valid.
    stop_on_first_improving_column::Bool = false
    improving_profit_tolerance::Float64 = 1e-6
    output_csv::Union{Nothing,String} = nothing
    verbose::Bool = false
end

struct RewardBoundResult
    upper_bound::Float64
    y_values::Vector{Float64}
    passenger_values::Dict{Int,Float64}
    status
    runtime_sec::Float64
    valid::Bool
end

mutable struct RewardBoundModel
    model::JuMP.Model
    data::PassengerRewardBoundData
    budget::Int
    budget_mode::Symbol
    y::Vector{JuMP.VariableRef}
    x::Vector{JuMP.VariableRef}
    q::Union{Nothing,JuMP.VariableRef}
    z::Union{Nothing,JuMP.VariableRef}
    use_cost_bound::Bool
    integral_stations::Bool
end

function RewardBoundModel(data::PassengerRewardBoundData, budget::Int;
    budget_mode::Symbol=:exact, optimizer=Gurobi.Optimizer,
    integral_stations::Bool=false,
    use_single_assignment_cost_bound::Bool=false,
    assignment_cost_lower_bounds::Union{Nothing,AbstractVector{<:Real}}=nothing,
    repositioning_cost::Float64=0.0,
    pairwise_bounds::Union{Nothing,PairwiseAssignmentRoutingBounds}=nothing,
    triple_bounds::Union{Nothing,TripleAssignmentRoutingBounds}=nothing)
    budget_mode in (:exact, :at_most) || throw(ArgumentError("budget_mode must be :exact or :at_most"))
    model = Model(optimizer)
    set_silent(model)
    if integral_stations
        @variable(model, y[1:length(data.stations)], Bin)
    else
        @variable(model, 0 <= y[1:length(data.stations)] <= 1)
    end
    @variable(model, 0 <= x[1:length(data.assignments)] <= 1)
    budget_mode == :exact ? @constraint(model, sum(y) == budget) : @constraint(model, sum(y) <= budget)
    for (i, a) in enumerate(data.assignments)
        @constraint(model, x[i] <= y[data.station_index[a.origin]])
        @constraint(model, x[i] <= y[data.station_index[a.destination]])
    end
    for p in data.passengers
        ids = findall(a -> a.passenger == p, data.assignments)
        @constraint(model, sum(x[i] for i in ids) <= 1)
    end
    z = nothing
    if repositioning_cost > 0
        @variable(model, 0 <= zvar <= 1)
        z = zvar
        for xi in x
            @constraint(model, xi <= zvar)
        end
    end
    q = nothing
    if use_single_assignment_cost_bound || !isnothing(pairwise_bounds) || !isnothing(triple_bounds)
        isnothing(assignment_cost_lower_bounds) && throw(ArgumentError("cost lower bounds are required"))
        length(assignment_cost_lower_bounds) == length(x) || throw(DimensionMismatch("one cost bound per assignment is required"))
        all(v -> v >= 0, assignment_cost_lower_bounds) || throw(ArgumentError("cost lower bounds must be nonnegative"))
        @variable(model, qvar >= 0)
        q = qvar
        for i in eachindex(x)
            @constraint(model, qvar >= Float64(assignment_cost_lower_bounds[i]) * x[i])
        end
        if !isnothing(pairwise_bounds)
            for (i,j) in pairwise_bounds.conflicts
                @constraint(model, x[i]+x[j] <= 1)
            end
            for (i,j,cost) in pairwise_bounds.joint_cost_lower_bounds
                @constraint(model, qvar >= cost*(x[i]+x[j]-1))
            end
        end
        if !isnothing(triple_bounds)
            for (i,j,k) in triple_bounds.conflicts
                @constraint(model, x[i]+x[j]+x[k] <= 2)
            end
            for (i,j,k,cost) in triple_bounds.joint_cost_lower_bounds
                @constraint(model, qvar >= cost*(x[i]+x[j]+x[k]-2))
            end
        end
    end
    @objective(model, Max, sum(data.assignments[i].reward*x[i] for i in eachindex(x)) -
                           (isnothing(q) ? 0.0 : q) -
                           (isnothing(z) ? 0.0 : repositioning_cost*z))
    RewardBoundModel(model, data, budget, budget_mode, y, x, q, z,
                     use_single_assignment_cost_bound, integral_stations)
end

function solve_reward_bound_lp!(bm::RewardBoundModel, included::BitSet, excluded::BitSet;
                                warm_start=nothing)::RewardBoundResult
    t0 = time()
    for i in eachindex(bm.y)
        if i in included
            fix(bm.y[i], 1.0; force=true)
        elseif i in excluded
            fix(bm.y[i], 0.0; force=true)
        else
            is_fixed(bm.y[i]) && unfix(bm.y[i])
        end
        if !isnothing(warm_start) && i <= length(warm_start)
            set_start_value(bm.y[i], warm_start[i])
        end
    end
    optimize!(bm.model)
    status = termination_status(bm.model)
    valid = status == MOI.OPTIMAL && primal_status(bm.model) == MOI.FEASIBLE_POINT
    if !valid
        return RewardBoundResult(Inf, fill(NaN, length(bm.y)), Dict{Int,Float64}(), status, time()-t0, false)
    end
    xv, yv = value.(bm.x), value.(bm.y)
    pv = Dict(p => sum(bm.data.assignments[i].reward*xv[i] for i in eachindex(xv)
                       if bm.data.assignments[i].passenger == p) for p in bm.data.passengers)
    # The solver's objective bound is the pruning certificate.  At OPTIMAL it
    # normally equals objective_value, but using the bound avoids accidentally
    # turning a merely feasible primal value into an upper bound.
    RewardBoundResult(objective_bound(bm.model), yv, pv, status, time()-t0, true)
end

struct StationSearchNode
    included::BitSet
    excluded::BitSet
    upper_bound::Float64
    cheap_upper_bound::Float64
    depth::Int
    lp_y_values::Union{Nothing,Vector{Float64}}
    node_id::Int
    parent_id::Union{Nothing,Int}
end

struct StationSubsetPricingCertificate
    optimal_value::Float64
    best_exact_result::ExactPricingResult
    globally_certified::Bool
    final_global_upper_bound::Float64
    absolute_gap::Float64
    relative_gap::Float64
    nodes_created::Int
    nodes_processed::Int
    nodes_pruned_cheap::Int
    nodes_pruned_lp::Int
    nodes_pruned_cardinality::Int
    fixed_subsets_priced::Int
    heuristic_subsets_priced::Int
    unique_subsets_priced::Int
    reward_lp_solves::Int
    total_exact_pricing_time::Float64
    total_bound_time::Float64
    total_runtime_sec::Float64
end

_station_key(s::BitSet) = Tuple(s)

function _fixed_station_set(included::BitSet, excluded::BitSet, n::Int, L::Int)
    undecided = setdiff(BitSet(1:n), union(included, excluded))
    length(included) > L && return (:infeasible, BitSet())
    length(included) + length(undecided) < L && return (:infeasible, BitSet())
    length(included) == L && return (:fixed, copy(included))
    length(included) + length(undecided) == L && return (:fixed, union(included, undecided))
    return (:open, undecided)
end

function _choose_branch(undecided::BitSet, y, rewards, rule)
    rule in (:most_fractional, :highest_reward_impact, :strong_branching) ||
        throw(ArgumentError("unsupported branching rule $rule"))
    rule == :highest_reward_impact && return argmax(i -> rewards[i], undecided)
    return argmin(i -> (abs(y[i]-0.5), -rewards[i], i), undecided)
end

"""
    price_by_station_subset_branch_and_bound(data, L; settings, optimizer)

Globally maximize exact fixed-station reduced profit. Availability is monotone
because the oracle permits routes to use any subset of its allowed stations.
When feasibility proves that a route can visit at most `K < L` distinct
stations, exact-K service-envelope search is therefore equivalent to exact-L
availability search, without repricing the same route through many supersets.
"""
function price_by_station_subset_branch_and_bound(
    data::PassengerFreeAssignmentPricingData, L::Int;
    settings::StationSubsetPricingSettings=StationSubsetPricingSettings(),
    optimizer=Gurobi.Optimizer,
)
    t0 = time(); n = length(data.nodes)
    0 <= L <= n || throw(ArgumentError("station budget must lie in 0:$n"))
    settings.budget_mode in (:exact, :at_most) || throw(ArgumentError("invalid budget mode"))
    data.route_regularization_weight >= 0 || throw(ArgumentError(
        "reward-only bounds require nonnegative routing cost"))
    data.repositioning_time >= 0 || throw(ArgumentError(
        "reward-only bounds require nonnegative repositioning cost"))
    all(cost -> cost >= 0, values(data.travel_cost)) || throw(ArgumentError(
        "reward-only bounds require nonnegative travel arcs; supply a proven route-cost lower bound instead"))
    search_budget = L
    if settings.use_route_station_cap
        data.bounded_max_stops && (search_budget = min(search_budget, data.max_stops))
        data.bounded_distinct_stations &&
            (search_budget = min(search_budget, data.max_distinct_stations))
    end
    reward_data = PassengerRewardBoundData(data)
    routing_bound_enabled = settings.use_routing_reward_bound
    pairwise = routing_bound_enabled && settings.use_pairwise_routing_bounds ?
        build_pairwise_assignment_routing_bounds(data,reward_data;
            alternatives_per_passenger=settings.pairwise_alternatives_per_passenger) : nothing
    triple = routing_bound_enabled && settings.use_triple_routing_bounds ?
        build_triple_assignment_routing_bounds(data,reward_data;
            alternatives_per_passenger=settings.triple_alternatives_per_passenger) : nothing
    direct_cost_bounds = (routing_bound_enabled || settings.use_single_assignment_cost_bound) ?
        [data.route_regularization_weight * get(data.travel_cost,(a.origin,a.destination),Inf)
         for a in reward_data.assignments] : nothing
    bm = settings.use_reward_lp ? RewardBoundModel(reward_data, search_budget;
        budget_mode=:exact, optimizer=optimizer,
        integral_stations=settings.integral_reward_stations,
        use_single_assignment_cost_bound=routing_bound_enabled || settings.use_single_assignment_cost_bound,
        assignment_cost_lower_bounds=(routing_bound_enabled || settings.use_single_assignment_cost_bound) ? direct_cost_bounds : nothing,
        repositioning_cost=routing_bound_enabled ?
            data.route_regularization_weight*data.repositioning_time : 0.0,
        pairwise_bounds=pairwise, triple_bounds=triple) : nothing
    cache = Dict{Tuple,ExactPricingResult}()
    empty_result = ExactPricingResult(0.0, -0.0, Int[], Tuple{Int,Int,Int}[], BitSet(), true, 0, 0, 0.0)
    incumbent = empty_result
    nodes = StationSearchNode[StationSearchNode(BitSet(), BitSet(), Inf, Inf, 0, nothing, 1, nothing)]
    created=1; processed=cheap_pruned=lp_pruned=card_pruned=fixed_priced=heur_priced=lp_solves=0
    exact_time=bound_time=0.0; stopped=false
    csv_rows = String["node,depth,included_count,excluded_count,open_nodes,cheap_bound,lp_bound,incumbent,branch_station,prune_reason,exact_subsets_priced"]
    function exact!(T::BitSet, heuristic::Bool)
        key = _station_key(T)
        haskey(cache,key) && return cache[key]
        station_ids = BitSet(data.nodes[i] for i in T)
        r = price_exact_on_stations(data,station_ids; time_limit=settings.exact_oracle_time_limit,
                                    reduced_cost_tol=settings.reduced_cost_tolerance)
        cache[key]=r; exact_time += r.runtime_sec
        heuristic ? (heur_priced += 1) : (fixed_priced += 1)
        if r.certified && r.value > incumbent.value + settings.bound_tolerance
            incumbent = r
        end
        return r
    end
    while !isempty(nodes)
        if processed >= settings.node_limit || time()-t0 >= settings.time_limit
            stopped=true; break
        end
        idx = settings.node_order == :depth_first ? length(nodes) :
              argmax(i -> nodes[i].upper_bound, eachindex(nodes))
        node = splice!(nodes,idx); processed += 1
        if node.upper_bound <= incumbent.value + settings.bound_tolerance
            cheap_pruned += 1; continue
        end
        state, payload = _fixed_station_set(node.included,node.excluded,n,search_budget)
        if state == :infeasible
            card_pruned += 1; continue
        elseif state == :fixed
            r=exact!(payload,false); r.certified || (stopped=true; push!(nodes,node); break)
            if settings.stop_on_first_improving_column &&
                    r.value > settings.improving_profit_tolerance
                stopped=true; break
            end
            continue
        end
        undecided=payload; available=setdiff(BitSet(1:n),node.excluded)
        available_ids=BitSet(data.nodes[i] for i in available)
        tb=time(); cheap=reward_upper_bound_all_available(reward_data,available_ids); bound_time+=time()-tb
        if cheap <= incumbent.value + settings.bound_tolerance
            cheap_pruned += 1; continue
        end
        lpbound=cheap; y=fill(0.5,n)
        if settings.use_reward_lp
            lp=solve_reward_bound_lp!(bm,node.included,node.excluded;warm_start=node.lp_y_values)
            lp_solves += 1; bound_time += lp.runtime_sec
            if lp.valid
                lpbound=min(cheap,lp.upper_bound); y=lp.y_values
                if lpbound <= incumbent.value + settings.bound_tolerance
                    lp_pruned += 1; continue
                end
                T=copy(node.included)
                need=search_budget-length(T)
                ranked=sort!(collect(undecided),by=i->(-y[i],-reward_data.aggregate_reward_impact[i],i))
                union!(T,BitSet(ranked[1:need]))
                r=exact!(T,true); r.certified || (stopped=true; push!(nodes,node); break)
                if settings.stop_on_first_improving_column &&
                        r.value > settings.improving_profit_tolerance
                    # The rounded subset is resolved, but the current family is
                    # not. Keep its certified LP bound in the open-node bound.
                    push!(nodes,StationSearchNode(copy(node.included),copy(node.excluded),
                        lpbound,cheap,node.depth,copy(y),node.node_id,node.parent_id))
                    stopped=true; break
                end
                lpbound <= incumbent.value + settings.bound_tolerance && (lp_pruned += 1; continue)
            end
        end
        stopped && break
        branch=_choose_branch(undecided,y,reward_data.aggregate_reward_impact,settings.branching_rule)
        for include in (true,false)
            I=copy(node.included); E=copy(node.excluded)
            include ? push!(I,branch) : push!(E,branch)
            created += 1
            push!(nodes,StationSearchNode(I,E,lpbound,cheap,node.depth+1,copy(y),created,node.node_id))
        end
        settings.verbose && @info "station subset pricing" node=node.node_id depth=node.depth cheap_bound=cheap lp_bound=lpbound incumbent=incumbent.value branch_station=data.nodes[branch] open_nodes=length(nodes)
    end
    open_upper = isempty(nodes) ? -Inf : maximum(x.upper_bound for x in nodes)
    global_upper=max(incumbent.value,open_upper)
    certified=!stopped && isempty(nodes) && all(r.certified for r in values(cache))
    gap=max(0.0,global_upper-incumbent.value)
    relgap=gap/max(1.0,abs(incumbent.value))
    if !isnothing(settings.output_csv)
        open(settings.output_csv,"w") do io
            foreach(row->println(io,row),csv_rows)
        end
    end
    cert=StationSubsetPricingCertificate(incumbent.value,incumbent,certified,global_upper,gap,relgap,
        created,processed,cheap_pruned,lp_pruned,card_pruned,fixed_priced,heur_priced,length(cache),
        lp_solves,exact_time,bound_time,time()-t0)
    certified && @assert global_upper <= incumbent.value + settings.bound_tolerance
    return cert
end
