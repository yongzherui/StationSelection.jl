"""
    scripts/diag_passenger_free_assignment_components.jl

Decides whether compatibility-component decomposition is worth implementing for
the passenger free-assignment pricer, before any of it is built.

The idea under test: if the positive-reward assignment graph splits into pieces
that share no station, then no route can serve two pieces profitably in a way
that couples them, and one large pricing problem becomes several small ones --
a big win, because label-search cost is strongly superlinear in the number of
live labels. If it does not split, the whole approach is worthless here.

Two graphs are reported, because they answer different questions:

  - **station graph** -- one node per station, an edge `j -- k` for every
    positive-reward opportunity `(p, j, k)`. Components here are what the label
    search could actually be partitioned over: a route is a walk in this graph,
    so a route collecting reward in two different components would have to
    travel between them while certifying nothing, which is never profitable.

  - **passenger reachability** -- how many distinct stations each passenger can
    use at all. If most passengers can reach most stations, the station graph is
    necessarily one blob and decomposition is dead on arrival.

Rule of thumb from the design discussion: if the largest component holds more
than ~80% of positive assignments, decomposition will not pay.

Usage:
    julia --project=. scripts/diag_passenger_free_assignment_components.jl [n_stations ...]
"""

using Printf, StationSelection

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = 16
const SEED = 42
const N_SCENARIOS = 3
const MAX_WALK = 600.0
const ROUTE_REG_WEIGHT = 10.0
const MAX_WAIT_TIME = 900.0
const REPOSITIONING_TIME = 20.0
const DETOUR_FACTOR = 2.0
const WALK_COST_WEIGHT = 0.1
const BASE_VALUE = 5000.0

function build_scenario_candidates(data::StationSelectionData, n_stations::Int, s::Int)
    candidates = PassengerAssignmentCandidate[]
    requests = data.scenarios[s].requests
    for row in eachrow(requests)
        o, d = row.origin_idx, row.dest_idx
        for j in 1:n_stations
            walk_o = get_walking_cost(data, o, j)
            walk_o <= MAX_WALK || continue
            for k in 1:n_stations
                k == j && continue
                walk_d = get_walking_cost(data, d, k)
                walk_d <= MAX_WALK || continue
                reward = BASE_VALUE - WALK_COST_WEIGHT * (walk_o + walk_d)
                reward > 0 || continue
                ride_limit = DETOUR_FACTOR * get_routing_cost(data, j, k)
                push!(candidates, PassengerAssignmentCandidate(row.id, j, k, ride_limit, reward))
            end
        end
    end
    return candidates
end

"""Union-find over station ids; returns `root -> member stations`."""
function station_components(opportunities, n_stations::Int)
    parent = collect(1:n_stations)
    find(x) = parent[x] == x ? x : (parent[x] = find(parent[x]))
    function unite!(a, b)
        ra, rb = find(a), find(b)
        ra == rb || (parent[ra] = rb)
    end
    for opp in opportunities
        unite!(opp.origin, opp.destination)
    end
    groups = Dict{Int, Vector{Int}}()
    for s in 1:n_stations
        push!(get!(() -> Int[], groups, find(s)), s)
    end
    return groups
end

function run_case(n_stations::Int)
    data, _meta = generate_zhuzhou_data(
        DATA_DIR, n_stations, N_PAIRS; n_scenarios=N_SCENARIOS, seed=SEED,
    )
    nodes = collect(1:n_stations)
    travel_cost = Dict{Tuple{Int, Int}, Float64}()
    for i in nodes, j in nodes
        i == j && continue
        travel_cost[(i, j)] = get_routing_cost(data, i, j)
    end

    for s in 1:StationSelection.n_scenarios(data)
        candidates = build_scenario_candidates(data, n_stations, s)
        isempty(candidates) && continue
        pd = create_passenger_free_assignment_pricing_data(
            s, nodes, travel_cost, candidates;
            route_regularization_weight=ROUTE_REG_WEIGHT,
            max_wait_time=MAX_WAIT_TIME,
            repositioning_time=REPOSITIONING_TIME,
        )
        opps = pd.opportunities
        isempty(opps) && continue

        groups = station_components(opps, n_stations)
        # Only components that actually carry opportunities matter; an isolated
        # station with no positive assignment is not a "piece" of anything.
        sizes = Int[]
        for (_root, members) in groups
            member_set = Set(members)
            n_opp = count(o -> o.origin in member_set, opps)
            n_opp > 0 && push!(sizes, n_opp)
        end
        sort!(sizes; rev=true)
        largest_share = isempty(sizes) ? 0.0 : sizes[1] / length(opps)

        stations_per_passenger = Dict{Int, Set{Int}}()
        for o in opps
            st = get!(() -> Set{Int}(), stations_per_passenger, o.passenger)
            push!(st, o.origin); push!(st, o.destination)
        end
        reach = [length(v) for v in values(stations_per_passenger)]

        @printf(
            "COMPONENTS\tn=%d\ts=%d\topportunities=%d\tpassengers=%d\tcomponents=%d\tsizes=%s\tlargest_share=%.3f\tmean_stations_per_passenger=%.1f/%d\n",
            n_stations, s, length(opps), length(stations_per_passenger),
            length(sizes), string(sizes[1:min(5, length(sizes))]),
            largest_share, sum(reach) / length(reach), n_stations,
        )
        flush(stdout)
    end
end

function main()
    for n_stations in (isempty(ARGS) ? [10, 15, 20] : parse.(Int, ARGS))
        run_case(n_stations)
    end
end

main()
