"""
Exhaustive column enumeration for `JointRoutingAssignmentRouteColumn`/`θ`, used by
`AggregateODRouteJointRoutingAssignmentFormulation` builds that need the whole column
universe up front (`DirectMIPSolver`) rather than iteratively priced columns (`CGSolver`).

# Reusing Base's physical-route DFS

The physical feasibility rule this formulation's routes obey -- wait time at pickup,
`detour_factor * routing_cost(j, k)` ride limit at dropoff -- is identical to
`AggregateODRouteBaseFormulation`'s, and it is evaluated per `(j, k)` pair, not per
passenger (`ride_limit = detour_factor * get_routing_cost(data, j, k)` in
`joint_routing_assignment/pricing_round.jl`, the exact formula
`route_covering/data.jl`'s `_direct_ride_limit` uses). So the set of physical
routes a from-scratch DFS against `JointRoutingAssignmentPricingLabel`'s own transitions
would visit is provably identical to what `route_covering/exact/enumeration.jl`'s DFS
already visits, given the same `max_stops`/`max_wait_time`/`detour_factor` and every
`(j, k)` pair made visible (both DFSs use the "uniform positive reward, nothing pruned"
trick to neutralize their own dual-dependent candidate-node filters). This module reuses
that DFS verbatim via `_enumerate_aggregate_od_route_raw_columns` -- the *raw*, pre-dedup
form, since collapsing to one column per served-pairs signature (what
`AggregateODRouteBaseFormulation`'s own public enumerator does next) would silently
discard physically distinct routes this formulation needs (different node order, different
`tau`, potentially different passenger certifications) that happen to share a served-pairs
signature.

# What's genuinely new here: the combinatorial assignment layer

Base's `x` is decoupled from `θ`, so a route's `served_pairs` is a plain *set* -- the
master can freely pick any subset via `x`. Joint has no such decoupling: a column's
`assignments` are baked in at creation. Real pricing (`exact/accept.jl`'s
`_replay_joint_routing_assignment_route`) resolves a fixed route to exactly *one*
assignment per passenger -- whichever `(j, k)` has the highest reward under the current
duals -- which is the right thing for CG (only the single most-improving column matters at
any one dual vector), but is not a complete column universe on its own: coverage
(`constraints/aggregate_od_route/joint_routing_assignment/coverage.jl`) is `>= 1`, a genuine
set-*covering* constraint, not a partition, so a column should always claim *every*
passenger a route certifies -- dropping one is never beneficial, only ever a missed free
credit. What real pricing's arg-max throws away isn't *which* passengers to claim (always
all of them), but *which* of several certified `(j, k)` pairs a multi-certified passenger
should be credited through, since a column is one physical vehicle trip and a passenger can
only actually be dropped off once: crediting the same passenger through two different pairs
in one column would mean one `θ` representing two mutually exclusive outcomes for them
(elementarity on `p`, the same idea as an elementary route not revisiting a node, applied
here to a passenger's assignment within one column rather than to the route itself).

So the combinatorial dimension is exactly: for each of Base's shared physical routes, take
every certified passenger (never omit one -- see above), and for any passenger certified
through more than one `(j, k)` pair on that route, branch over which single one they take.
`_joint_routing_assignment_route_combinations` below is the direct cartesian product of
each certified passenger's own option list (size 1 -- no branching -- for the large
majority of passengers; only genuinely multi-certified ones contribute a real factor).
There is no separate "which stations to require built" axis on top of this: a route
achieving the same certifications with fewer stations is already its own, separately
`max_stops`-bounded entry in the shared physical-route pool (the DFS seeds a label at
*every* relevant node, not just one start, and visits every prefix length), so trying to
carve smaller station requirements out of a *longer* route here would only duplicate routes
the DFS already explores on its own.
"""

export enumerate_joint_routing_assignment_columns

"""
    _replay_joint_routing_assignment_route_all_certifications(route, pricing_data)
        -> Dict{Int, Vector{Tuple{Int, Int}}}

Same replay walk as `_replay_joint_routing_assignment_route` (`exact/accept.jl`) -- same
station-age/ride-limit physical rules, same `pricing_data.assignments_by_destination`
lookups -- but keeps *every* distinct `(origin, destination)` certified for each
passenger `p`, not only the highest-reward one. Reward plays no role here (this
enumerator feeds `pricing_data` uniform positive rewards, see
`enumerate_joint_routing_assignment_columns`), so there is nothing to arg-max over.
"""
function _replay_joint_routing_assignment_route_all_certifications(
    route::Vector{Int},
    pricing_data::JointRoutingAssignmentPricingData,
)::Dict{Int, Vector{Tuple{Int, Int}}}
    certified = Dict{Int, Vector{Tuple{Int, Int}}}()
    isempty(route) && return certified

    pickup_time = Dict{Int, Float64}()
    current = route[1]
    elapsed_time = 0.0
    pickup_time[current] = 0.0

    for idx in 2:length(route)
        next_node = route[idx]
        travel_time = _joint_routing_assignment_travel(pricing_data, current, next_node)
        elapsed_time += travel_time

        for opp in get(pricing_data.assignments_by_destination, next_node, PassengerAssignmentOpportunity[])
            origin_age = elapsed_time - get(pickup_time, opp.origin, -Inf)
            origin_age <= opp.ride_limit + 1e-9 || continue
            options = get!(() -> Tuple{Int, Int}[], certified, opp.p)
            pair = (opp.origin, opp.destination)
            pair in options || push!(options, pair)
        end

        if elapsed_time <= pricing_data.max_wait_time + 1e-9
            pickup_time[next_node] = elapsed_time
        end
        current = next_node
    end

    return certified
end

"""
    _joint_routing_assignment_route_combinations(certified) -> Vector{Vector{Tuple{Int,Int,Int}}}

One combination per way to pick, for *every* passenger this route certifies, exactly one of
their own certified `(j, k)` pairs -- the direct cartesian product of each certified
passenger's own option list. Always maximal (every certified passenger appears in every
combination -- see this file's module docstring for why omitting one is never beneficial
under `>= 1` coverage) and elementary on `p` (never more than one pair per passenger per
combination). Most passengers contribute a size-1 option list (no branching); only
passengers this specific route certifies through more than one `(j, k)` pair contribute a
real factor, so this stays small in practice even though it is technically exponential in
how many *multi-certified* passengers one route reaches.
"""
function _joint_routing_assignment_route_combinations(
    certified::Dict{Int, Vector{Tuple{Int, Int}}},
)::Vector{Vector{Tuple{Int, Int, Int}}}
    passengers = sort!(collect(keys(certified)))
    isempty(passengers) && return Vector{Tuple{Int, Int, Int}}[]

    choice_sets = [[(p, o, d) for (o, d) in certified[p]] for p in passengers]
    return vec([collect(combo) for combo in Iterators.product(choice_sets...)])
end

"""
    _deduplicate_joint_routing_assignment_columns(columns) -> Vector{JointRoutingAssignmentRouteColumn}

Keep the cheapest (`tau`) column per distinct `(scenario, assignments signature)`,
renumbered `1:length(...)`. Scoped by scenario (unlike
`_deduplicate_aggregate_od_route_columns`'s single `od_pairs` key) because `p` is a
scenario-local demand-group index -- the same `(p, j, k)` triple means different real
demand in different scenarios.
"""
function _deduplicate_joint_routing_assignment_columns(
    columns::Vector{JointRoutingAssignmentRouteColumn},
)::Vector{JointRoutingAssignmentRouteColumn}
    best = Dict{Tuple{Int, Any}, JointRoutingAssignmentRouteColumn}()
    for column in columns
        key = (Int(column.metadata["scenario"]), _joint_routing_assignment_column_signature(column))
        incumbent = get(best, key, nothing)
        if isnothing(incumbent) || column.tau < incumbent.tau - 1e-9
            best[key] = column
        end
    end
    out = JointRoutingAssignmentRouteColumn[]
    next_id = 1
    for (key, column) in sort!(collect(best); by=kv -> (kv[1][1], length(kv[1][2]), kv[2].tau))
        push!(out, JointRoutingAssignmentRouteColumn(
            next_id, column.route, column.assignments, column.tau; metadata=copy(column.metadata),
        ))
        next_id += 1
    end
    return out
end

"""
    enumerate_joint_routing_assignment_columns(problem, formulation, data;
        max_routes=10_000, time_limit_sec=30.0) -> Vector{JointRoutingAssignmentRouteColumn}

Exhaustive `θ` pool for `AggregateODRouteJointRoutingAssignmentFormulation`'s own
`DirectMIPSolver` build (`optimize/aggregate_od_route/direct/build_joint_routing_assignment.jl`)
-- the formulation's counterpart to `enumerate_aggregate_od_route_columns`. See this
file's module docstring for the design: reuse Base's physical-route DFS, then for each
route take the maximal, elementarity-preserving cartesian product over every certified
passenger's own certified `(j, k)` options.

**Small instances only.** The combinatorial expansion is exponential in how many passengers
a single route certifies through more than one `(j, k)` pair, on top of the physical-route
DFS's own combinatorial growth in `max_stops`. Both throw (never silently truncate) once
`max_routes`/`time_limit_sec` is exceeded, exactly like `enumerate_aggregate_od_route_columns`.
This makes the function suitable as a small-`max_stops` ground-truth/certification tool
(see `benchmarks/study1_formulation_lp_ip_gap`), not as part of any scalability benchmark.

Feeds the underlying `JointRoutingAssignmentPricingData` uniform positive rewards (mirroring
`enumerate_aggregate_od_route_columns`'s own uniform duals trick) so every physically valid
`(p, j, k)` opportunity survives `create_joint_routing_assignment_pricing_data`'s
`reward > 0` filter -- real reward values play no further role, since the replay this
enumerator uses collects every certification rather than arg-maxing over reward.
"""
function enumerate_joint_routing_assignment_columns(
    problem::StationSelectionProblem,
    formulation::AggregateODRouteJointRoutingAssignmentFormulation,
    data::StationSelectionData;
    max_routes::Int=10_000,
    time_limit_sec::Float64=30.0,
)::Vector{JointRoutingAssignmentRouteColumn}
    max_routes > 0 || throw(ArgumentError("max_routes must be positive"))
    time_limit_sec > 0 || throw(ArgumentError("time_limit_sec must be positive"))
    t_start = time()

    mapping = create_aggregate_od_route_map(problem, formulation, data)
    raw_routes = _enumerate_aggregate_od_route_raw_columns(
        mapping, data,
        formulation.route_regularization_weight, formulation.repositioning_time,
        formulation.max_wait_time, formulation.detour_factor, formulation.max_stops,
        max_routes, time_limit_sec,
    )
    # `_enumerate_aggregate_od_route_raw_columns` also appends `mapping.columns`, its
    # walking-feasibility-guaranteeing singleton defaults (`_singleton_aggregate_od_route_columns`,
    # `data/maps/aggregate_od_route_map.jl`) -- those carry no `"route"` metadata and are
    # redundant here: any direct `(j, k)` route they represent is always within the same
    # DFS's own depth-2 reach (`detour_factor >= 1` means the direct hop is always inside
    # its own ride limit), so only the genuinely DFS-visited entries are kept.
    physical_routes = unique(
        c -> c.metadata["route"],
        filter(c -> haskey(c.metadata, "route"), raw_routes),
    )

    n = data.n_stations
    nodes = collect(1:n)
    travel_cost = Dict{Tuple{Int, Int}, Float64}()
    for i in 1:n, j in 1:n
        i == j && continue
        cost = get_routing_cost(data, i, j)
        isfinite(cost) && (travel_cost[(i, j)] = cost)
    end

    columns = JointRoutingAssignmentRouteColumn[]
    next_id = 1
    for s in 1:n_scenarios(data)
        candidates = PassengerAssignmentCandidate[]
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            mapping.Q_s[s][p] > 0 || continue
            for pair in get_valid_jk_pairs(mapping, o, d)
                is_walk_only_pair(pair) && continue
                j, k = pair
                ride_limit = formulation.detour_factor * get_routing_cost(data, j, k)
                push!(candidates, PassengerAssignmentCandidate(p, j, k, ride_limit, 1.0))
            end
        end
        isempty(candidates) && continue

        pricing_data = create_joint_routing_assignment_pricing_data(
            s, nodes, travel_cost, candidates;
            route_regularization_weight=formulation.route_regularization_weight,
            max_wait_time=formulation.max_wait_time,
            repositioning_time=formulation.repositioning_time,
            max_stops=formulation.max_stops,
            compensated_dominance=formulation.compensated_dominance,
        )

        for raw in physical_routes
            time() - t_start <= time_limit_sec || throw(ArgumentError(
                "joint route enumeration did not complete within time_limit_sec=$(time_limit_sec)"
            ))
            route = collect(Int, raw.metadata["route"])
            certified = _replay_joint_routing_assignment_route_all_certifications(route, pricing_data)
            for assignments in _joint_routing_assignment_route_combinations(certified)
                push!(columns, JointRoutingAssignmentRouteColumn(
                    next_id, route, assignments, raw.tau;
                    metadata=Dict{String, Any}(
                        "scenario" => s, "route" => Tuple(route), "initialization" => "enumeration",
                    ),
                ))
                next_id += 1
                length(columns) <= max_routes ||
                    throw(ArgumentError("joint route enumeration exceeded max_routes=$(max_routes)"))
            end
        end
    end

    return _deduplicate_joint_routing_assignment_columns(columns)
end
