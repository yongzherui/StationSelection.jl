"""
Restricted master problem for passenger free-assignment column generation.

# Why this needs its own master

The existing `AggregateODRouteModel` master covers *aggregate* station OD pairs:
its coverage rows are indexed by `(j, k, s)` and yield one dual `sigma_jks` per
station pair. The passenger pricer in `search.jl` prices against a reward

    rho_pjk = alpha_p - gamma^O_pj - gamma^D_pk - w_pjk

whose duals are **passenger-specific**. No dual of the aggregate master has that
index structure, so the two cannot share a master -- this file builds the
formulation whose LP duals are exactly `alpha_p`, `gamma^O_pj`, `gamma^D_pk`.

# Formulation

Decision variables:
  - `y[j] in {0,1}`     station j built (first stage, shared across scenarios)
  - `theta[r] >= 0`      route column r selected (r carries its own scenario)
  - `x_same[p,j] >= 0`   passenger p served with NO vehicle route, walking in and
                         out at station j (enumerable, not generated)
  - `v[p] >= 0`          passenger p left unserved (penalty slack)

    min  sum_r ( beta*(tau_r + repositioning) + walk_weight * sum_p q_p*w_{p,j,k} ) theta_r
         + sum_{p,j} walk_weight * q_p * w_{p,j,j} x_same[p,j]
         + sum_p unserved_penalty * v[p]

    s.t. (alpha_p)     sum_r a_rp theta_r + sum_j x_same[p,j] + v[p]  >= 1     for each passenger p
         (gamma^O_pj)  sum_r a^O_rpj theta_r + x_same[p,j]            <= y[j]  for each feasible (p, j)
         (gamma^D_pk)  sum_r a^D_rpk theta_r + x_same[p,k]            <= y[k]  for each feasible (p, k)
                       sum_j y[j] = l

`a_rp` is 1 when column r assigns passenger p at all; `a^O_rpj` is 1 when it
assigns p a *pickup at j* specifically. Reduced cost of `theta_r` is then

    rc_r = beta*(tau_r + repositioning) - sum_p rho_{p, j_p^r, k_p^r}

which is precisely what `_passenger_free_assignment_column_from_route` computes,
so the pricer is an exact oracle for this master with no translation layer. See
`_verify_passenger_master_reduced_cost` for the assertion that pins this.

The pickup/dropoff linking is **disaggregated** (one row per `(p, j)`, not a
single big-M row per `j`). That is what makes the duals passenger-specific, and
it is also the tighter LP relaxation -- but it means the row count grows as
`sum_p |feasible pickups of p|`, which is the main size driver of this master.

`v[p]` exists so the RMP is feasible from an empty column pool.
`unserved_penalty` must exceed any real service cost or it will distort the
optimum rather than merely guaranteeing feasibility; see
`_default_unserved_penalty`. Any solution with `v[p] > 0` makes the objective
incomparable to the LP bound (it contains a big-M term), so gap statistics must
be read as invalid whenever a passenger is on slack.

Route columns cover only `j != k`; `x_same` covers `j == k`. Excluding
same-station options entirely (an earlier version of this file) was measured to
force a passenger onto slack at n=10/l=5 -- not because any single passenger
lacked a `j != k` option, but combinatorially: no choice of 5 stations covered
all 16 passengers without same-station legs. Adding `x_same` took the provable
minimum-unserved from 1 to 0 on that instance.
"""

export PassengerFreeAssignmentPassenger
export PassengerFreeAssignmentMasterData
export PassengerFreeAssignmentMaster
export build_passenger_free_assignment_master
export passenger_free_assignment_two_stop_seed_columns
export extract_passenger_free_assignment_duals
export passenger_free_assignment_pricing_candidates

struct PassengerFreeAssignmentPassenger
    id::Int
    scenario::Int
    origin::Int
    destination::Int
    demand::Int
end

"""
Dual-independent preprocessing shared by the master and every pricing call:
who the passengers are, which `(j, k)` each may use, and the per-assignment
walking cost. Only `rho` changes between CG iterations, so this is built once.
"""
struct PassengerFreeAssignmentMasterData
    passengers::Vector{PassengerFreeAssignmentPassenger}
    passengers_by_scenario::Dict{Int, Vector{Int}}
    feasible_assignments::Dict{Int, Vector{Tuple{Int, Int}}}
    assignment_walk_cost::Dict{Tuple{Int, Int, Int}, Float64}
    feasible_pickups::Dict{Int, Vector{Int}}
    feasible_dropoffs::Dict{Int, Vector{Int}}
    ride_limit::Dict{Tuple{Int, Int, Int}, Float64}
    nodes::Vector{Int}
    travel_cost::Dict{Tuple{Int, Int}, Float64}
    route_regularization_weight::Float64
    walk_cost_weight::Float64
    repositioning_time::Float64
    max_wait_time::Float64
    max_stops::Int
    l::Int
    unserved_penalty::Float64
    # No-vehicle-route ("same-station") options: passenger p walks to j and from j
    # to its destination, so no route column is needed. Enumerable and small
    # (O(P*n)), so held directly in the master rather than generated. A route can
    # never certify (j, j) anyway -- `ride_limit = detour_factor * travel(j,j) = 0`,
    # so any revisit to j has age > 0 and fails the test -- which is why these must
    # be represented outside the column pool. Mirrors the aggregate model's
    # `requires_no_vehicle_route` / WALK_ONLY_PAIR handling.
    same_station_options::Dict{Int, Vector{Int}}
    same_station_walk_cost::Dict{Tuple{Int, Int}, Float64}
end

mutable struct PassengerFreeAssignmentMaster
    model::Model
    master_data::PassengerFreeAssignmentMasterData
    y::Vector{VariableRef}
    v::Dict{Int, VariableRef}
    x_same::Dict{Tuple{Int, Int}, VariableRef}
    theta::Dict{Int, VariableRef}
    coverage::Dict{Int, ConstraintRef}
    pickup_link::Dict{Tuple{Int, Int}, ConstraintRef}
    dropoff_link::Dict{Tuple{Int, Int}, ConstraintRef}
    columns::Dict{Int, PassengerFreeAssignmentRouteColumn}
    column_signatures::Dict{Any, Int}
end

"""
A penalty that is strictly worse than serving a passenger by the most expensive
possible single route, so `v[p] > 0` at the optimum means "genuinely
unservable", never "cheaper to abandon". Derived from the actual cost data
rather than a magic constant: the worst conceivable route cost under
`max_stops`, plus the worst walking cost.
"""
function _default_unserved_penalty(
    nodes::Vector{Int},
    travel_cost::Dict{Tuple{Int, Int}, Float64},
    assignment_walk_cost::Dict{Tuple{Int, Int, Int}, Float64},
    route_regularization_weight::Float64,
    walk_cost_weight::Float64,
    repositioning_time::Float64,
    max_stops::Int,
)::Float64
    finite_travel = [c for c in values(travel_cost) if isfinite(c)]
    max_travel = isempty(finite_travel) ? 0.0 : maximum(finite_travel)
    # Serving ONE passenger never requires more than a direct two-stop route `[j,k]`
    # (that is the cheapest route certifying `(j,k)`), so `beta*(max_travel + repo)`
    # plus the worst walking cost already exceeds any single passenger's true service
    # cost. 10x that is a safe big-M.
    #
    # Deliberately independent of `max_stops`. An earlier version scaled `hops` with
    # `max_stops`, which made the penalty ~2x larger for an uncapped run than a
    # capped one -- so whenever slack was active the two runs' objectives were not
    # comparable, and an uncapped run could look WORSE than a capped one purely from
    # the penalty (observed at n=8/p=16, where 2 passengers sat on slack).
    worst_route = route_regularization_weight * (max_travel + repositioning_time)
    worst_walk = isempty(assignment_walk_cost) ? 0.0 :
        walk_cost_weight * maximum(values(assignment_walk_cost))
    return 10.0 * (worst_route + worst_walk) + 1.0
end

"""
    create_passenger_free_assignment_master_data(model, data, mapping) -> PassengerFreeAssignmentMasterData

One passenger per `(scenario, origin, destination)` demand group, carrying the
group's multiplicity as `demand` (so a group of `q` identical requests is one
passenger whose walking cost is scaled by `q`, not `q` separate passengers --
they are interchangeable under free assignment, and collapsing them keeps the
disaggregated linking rows from multiplying needlessly).

Assignments with `j == k` are excluded: they need no vehicle route, so they are
outside what a route column can certify. A passenger left with no `j != k`
option stays coverable only through its slack `v[p]`.
"""
function create_passenger_free_assignment_master_data(
    model::AnyAggregateODRouteModel,
    data::StationSelectionData,
    mapping::AggregateODRouteMap;
    unserved_penalty::Union{Float64, Nothing}=nothing,
)::PassengerFreeAssignmentMasterData
    base_model = model isa RouteCoveringProblem ? model.base : model
    has_routing_costs(data) ||
        throw(ArgumentError("passenger free-assignment master requires routing_costs"))

    nodes = collect(1:data.n_stations)
    travel_cost = Dict{Tuple{Int, Int}, Float64}()
    for i in nodes, j in nodes
        i == j && continue
        cost = get_routing_cost(data, i, j)
        isfinite(cost) && (travel_cost[(i, j)] = cost)
    end

    passengers = PassengerFreeAssignmentPassenger[]
    passengers_by_scenario = Dict{Int, Vector{Int}}()
    next_id = 1
    for s in 1:n_scenarios(data)
        ids = Int[]
        for (o, d) in sort!(collect(keys(get(mapping.Q_s, s, Dict{Tuple{Int, Int}, Int}()))))
            demand = mapping.Q_s[s][(o, d)]
            demand > 0 || continue
            push!(passengers, PassengerFreeAssignmentPassenger(next_id, s, o, d, demand))
            push!(ids, next_id)
            next_id += 1
        end
        passengers_by_scenario[s] = ids
    end

    feasible_assignments = Dict{Int, Vector{Tuple{Int, Int}}}()
    assignment_walk_cost = Dict{Tuple{Int, Int, Int}, Float64}()
    ride_limit = Dict{Tuple{Int, Int, Int}, Float64}()
    feasible_pickups = Dict{Int, Vector{Int}}()
    feasible_dropoffs = Dict{Int, Vector{Int}}()
    max_walk = mapping.max_walking_distance

    same_station_options = Dict{Int, Vector{Int}}()
    same_station_walk_cost = Dict{Tuple{Int, Int}, Float64}()

    for p in passengers
        pairs = Tuple{Int, Int}[]
        pickups = Set{Int}()
        dropoffs = Set{Int}()
        same = Int[]
        for j in nodes
            walk_o = get_walking_cost(data, p.origin, j)
            walk_o <= max_walk || continue
            # no-vehicle-route option: walk in and out at the same station
            walk_d_same = get_walking_cost(data, p.destination, j)
            if walk_d_same <= max_walk
                push!(same, j)
                same_station_walk_cost[(p.id, j)] = p.demand * (walk_o + walk_d_same)
                # the linking rows must exist for j so this option can be tied to y[j]
                push!(pickups, j)
                push!(dropoffs, j)
            end
            for k in nodes
                k == j && continue
                haskey(travel_cost, (j, k)) || continue
                walk_d = get_walking_cost(data, p.destination, k)
                walk_d <= max_walk || continue
                push!(pairs, (j, k))
                push!(pickups, j)
                push!(dropoffs, k)
                assignment_walk_cost[(p.id, j, k)] = p.demand * (walk_o + walk_d)
                ride_limit[(p.id, j, k)] = base_model.detour_factor * travel_cost[(j, k)]
            end
        end
        feasible_assignments[p.id] = pairs
        feasible_pickups[p.id] = sort!(collect(pickups))
        feasible_dropoffs[p.id] = sort!(collect(dropoffs))
        same_station_options[p.id] = same
    end

    penalty = isnothing(unserved_penalty) ?
        _default_unserved_penalty(
            nodes, travel_cost, assignment_walk_cost,
            base_model.route_regularization_weight, base_model.walk_cost_weight,
            base_model.repositioning_time, base_model.max_stops,
        ) : unserved_penalty

    return PassengerFreeAssignmentMasterData(
        passengers,
        passengers_by_scenario,
        feasible_assignments,
        assignment_walk_cost,
        feasible_pickups,
        feasible_dropoffs,
        ride_limit,
        nodes,
        travel_cost,
        base_model.route_regularization_weight,
        base_model.walk_cost_weight,
        base_model.repositioning_time,
        base_model.max_wait_time,
        base_model.max_stops,
        base_model.l,
        penalty,
        same_station_options,
        same_station_walk_cost,
    )
end

"""
    build_passenger_free_assignment_master(master_data, optimizer_env; relax_integrality)

Build the RMP with no columns yet. Constraint rows depend only on passengers, so
they are created once here; `add_passenger_free_assignment_column!` later grows
the model by adding `theta` variables and setting their coefficients in these
existing rows.

Thin orchestration only -- every `@variable`/`@constraint`/`@objective` call lives in
`variables|constraints|objectives/aggregate_od_route/column_generation/master.jl`, matching the
convention `AggregateODRouteModel`'s own `build.jl` already follows (see those files' module
docstrings for why `master_data` is passed there untyped).
"""
function build_passenger_free_assignment_master(
    master_data::PassengerFreeAssignmentMasterData,
    optimizer_env;
    relax_integrality::Bool=true,
)::PassengerFreeAssignmentMaster
    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    y = add_passenger_free_assignment_station_variables!(m, master_data; relax_integrality=relax_integrality)
    v = add_passenger_slack_variables!(m, master_data)
    coverage = add_passenger_coverage_constraints!(m, master_data, v)
    pickup_link, dropoff_link = add_passenger_station_linking_constraints!(m, master_data, y)
    x_same = add_passenger_same_station_variables!(m, master_data, coverage, pickup_link, dropoff_link)
    add_passenger_station_budget_constraint!(m, master_data)
    set_passenger_free_assignment_objective!(m, master_data, v, x_same)

    theta = Dict{Int, VariableRef}()
    columns = Dict{Int, PassengerFreeAssignmentRouteColumn}()
    m[:theta] = theta
    m[:passenger_coverage] = coverage
    m[:passenger_pickup_link] = pickup_link
    m[:passenger_dropoff_link] = dropoff_link

    return PassengerFreeAssignmentMaster(
        m, master_data, y, v, x_same,
        theta,
        coverage, pickup_link, dropoff_link,
        columns,
        Dict{Any, Int}(),
    )
end

"""
    passenger_free_assignment_column_cost(column, master_data) -> Float64

The column's true objective coefficient: `beta*(tau + repositioning)` plus the
demand-weighted walking cost of the concrete assignments it carries.
"""
function passenger_free_assignment_column_cost(
    column::PassengerFreeAssignmentRouteColumn,
    master_data::PassengerFreeAssignmentMasterData,
)::Float64
    walk = 0.0
    for (p, j, k) in column.assignments
        walk += get(master_data.assignment_walk_cost, (p, j, k), 0.0)
    end
    return master_data.route_regularization_weight * (column.tau + master_data.repositioning_time) +
        master_data.walk_cost_weight * walk
end

"""
    passenger_free_assignment_two_stop_seed_columns(master_data; next_column_id=1)

Every two-stop route `[j, k]` that any passenger can use, one column per
`(scenario, j, k)`.

# Why this exists

The `v[p]` slack makes the RMP feasible from an *empty* pool, but an empty pool
is not a requirement -- it was just the starting point. Starting empty means the
first several CG iterations are not improving the routing cost at all, they are
hunting for enough columns to cover every passenger, and until they succeed the
LP objective is dominated by `unserved_penalty * sum_p v[p]`. Measured on the
2026-07-30 scaling grid: the iteration-1 LP is 39x-131x the final value, and
*all* of that is big-M draining out. The genuine CG improvement, measured from
the first iterate that covers everyone, is only 1.8%-50%.

Two-stop routes remove that phase entirely, because they are exactly the
columns the big-M was standing in for. `_default_unserved_penalty` already says
so in its own derivation: "serving ONE passenger never requires more than a
direct two-stop route `[j, k]`". Seed them and every feasible assignment is in
the pool from iteration 1, so `v` is never priced in unless the instance is
*genuinely* uncoverable by any `l`-subset (which `x_same` also exists to
mitigate) -- and the duals the pricer sees are real service costs rather than
big-M, from the very first search.

# Coverage claim

`ride_limit[(p,j,k)] = detour_factor * travel(j,k)`, and replaying `[j, k]`
gives the pickup at `j` an age of exactly `travel(j,k)` on arrival at `k`. So
with `detour_factor >= 1` *every* `(p,j,k)` in `feasible_assignments` is
certified by its own two-stop route. The age test is still applied explicitly
below rather than assumed, so a `detour_factor < 1` configuration silently
drops the uncertifiable assignments instead of building an invalid column.

One column per `(s, j, k)` rather than per `(p, j, k)`: a two-stop route carries
*every* passenger of that scenario whose `(j,k)` it certifies, and that is the
same column, so the seed count is bounded by `n_scenarios * n * (n-1)` and in
practice by the number of distinct feasible pairs.
"""
function passenger_free_assignment_two_stop_seed_columns(
    master_data::PassengerFreeAssignmentMasterData;
    next_column_id::Int=1,
)::Vector{PassengerFreeAssignmentRouteColumn}
    by_route = Dict{Tuple{Int, Int, Int}, Vector{Tuple{Int, Int, Int}}}()
    for p in master_data.passengers
        for (j, k) in master_data.feasible_assignments[p.id]
            j == k && continue
            tau = get(master_data.travel_cost, (j, k), Inf)
            isfinite(tau) || continue
            # Replay of `[j, k]`: the pickup at `j` is `tau` old on arrival at `k`.
            tau <= master_data.ride_limit[(p.id, j, k)] + 1e-9 || continue
            push!(get!(by_route, (p.scenario, j, k), Tuple{Int, Int, Int}[]), (p.id, j, k))
        end
    end

    columns = PassengerFreeAssignmentRouteColumn[]
    id = next_column_id
    for (s, j, k) in sort!(collect(keys(by_route)))
        push!(columns, PassengerFreeAssignmentRouteColumn(
            id, [j, k], sort!(by_route[(s, j, k)]), master_data.travel_cost[(j, k)];
            metadata=Dict{String, Any}("scenario" => s, "seed" => "two_stop"),
        ))
        id += 1
    end
    return columns
end

"""
    extract_passenger_free_assignment_duals(master) -> (alpha, gamma_o, gamma_d)

`alpha_p >= 0` from the `>=` coverage rows; `gamma^O/gamma^D >= 0` as the
*negated* duals of the `<=` linking rows, so that

    rc_theta = cost - sum_p alpha_p + sum gamma^O + sum gamma^D

matches the pricer's `beta*(tau+repo) - sum_p rho_p` sign convention directly.
"""
function extract_passenger_free_assignment_duals(master::PassengerFreeAssignmentMaster)
    alpha = Dict{Int, Float64}()
    for (p, con) in master.coverage
        alpha[p] = dual(con)
    end
    gamma_o = Dict{Tuple{Int, Int}, Float64}()
    for (key, con) in master.pickup_link
        gamma_o[key] = -dual(con)
    end
    gamma_d = Dict{Tuple{Int, Int}, Float64}()
    for (key, con) in master.dropoff_link
        gamma_d[key] = -dual(con)
    end
    return alpha, gamma_o, gamma_d
end

"""
    passenger_free_assignment_pricing_candidates(master_data, alpha, gamma_o, gamma_d, scenario)

Turn the current RMP duals into `PassengerAssignmentCandidate`s for one
scenario. Only `rho > 0` survives into pricing (the pricer's reward-layer
preprocessing drops the rest anyway); this filter is applied here too so the
candidate vector handed to `create_passenger_free_assignment_pricing_data`
stays as small as possible.
"""
function passenger_free_assignment_pricing_candidates(
    master_data::PassengerFreeAssignmentMasterData,
    alpha::Dict{Int, Float64},
    gamma_o::Dict{Tuple{Int, Int}, Float64},
    gamma_d::Dict{Tuple{Int, Int}, Float64},
    scenario::Int,
)::Vector{PassengerAssignmentCandidate}
    candidates = PassengerAssignmentCandidate[]
    for p_id in get(master_data.passengers_by_scenario, scenario, Int[])
        a = get(alpha, p_id, 0.0)
        a > 1e-9 || continue
        for (j, k) in master_data.feasible_assignments[p_id]
            rho = a - get(gamma_o, (p_id, j), 0.0) - get(gamma_d, (p_id, k), 0.0) -
                master_data.walk_cost_weight * master_data.assignment_walk_cost[(p_id, j, k)]
            rho > 1e-9 || continue
            push!(candidates, PassengerAssignmentCandidate(
                p_id, j, k, master_data.ride_limit[(p_id, j, k)], rho,
            ))
        end
    end
    return candidates
end

"""
Cross-check that the pricer's reported reduced cost equals the one implied by
the master's own duals and the column's true objective coefficient. Catches any
drift between the two formulations (a wrong dual sign, a missing linking row, a
walking-cost weight applied on one side only) at the moment it happens instead
of as a silently wrong LP bound.
"""
function _verify_passenger_master_reduced_cost(
    column::PassengerFreeAssignmentRouteColumn,
    master_data::PassengerFreeAssignmentMasterData,
    alpha::Dict{Int, Float64},
    gamma_o::Dict{Tuple{Int, Int}, Float64},
    gamma_d::Dict{Tuple{Int, Int}, Float64};
    atol::Float64=1e-5,
)
    pricer_rc = Float64(get(column.metadata, "reduced_cost", NaN))
    isnan(pricer_rc) && return true, pricer_rc, NaN
    master_rc = passenger_free_assignment_column_cost(column, master_data)
    for (p, j, k) in column.assignments
        master_rc -= get(alpha, p, 0.0)
        master_rc += get(gamma_o, (p, j), 0.0)
        master_rc += get(gamma_d, (p, k), 0.0)
    end
    return isapprox(pricer_rc, master_rc; atol=atol), pricer_rc, master_rc
end
