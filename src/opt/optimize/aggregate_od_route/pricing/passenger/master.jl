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
export passenger_free_assignment_station_reduced_cost_eliminations

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
    max_visits_per_node::Int
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
        base_model.max_visits_per_node,
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
"""
function build_passenger_free_assignment_master(
    master_data::PassengerFreeAssignmentMasterData,
    optimizer_env;
    relax_integrality::Bool=true,
)::PassengerFreeAssignmentMaster
    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    n = length(master_data.nodes)

    if relax_integrality
        y = @variable(m, [1:n], lower_bound = 0.0, upper_bound = 1.0, base_name = "y")
    else
        y = @variable(m, [1:n], Bin, base_name = "y")
    end
    m[:y] = y

    v = Dict{Int, VariableRef}()
    for p in master_data.passengers
        v[p.id] = @variable(m, lower_bound = 0.0, base_name = "v[$(p.id)]")
    end
    m[:v] = v

    coverage = Dict{Int, ConstraintRef}()
    for p in master_data.passengers
        coverage[p.id] = @constraint(m, v[p.id] >= 1)
    end
    pickup_link = Dict{Tuple{Int, Int}, ConstraintRef}()
    dropoff_link = Dict{Tuple{Int, Int}, ConstraintRef}()
    # Written as `-y <= 0` rather than `0 <= y` so the normalized form JuMP stores
    # is unambiguous: adding a column's `theta` coefficient of +1.0 via
    # `set_normalized_coefficient` then yields exactly `theta - y[j] <= 0`.
    for p in master_data.passengers
        for j in master_data.feasible_pickups[p.id]
            pickup_link[(p.id, j)] = @constraint(m, -y[j] <= 0.0)
        end
        for k in master_data.feasible_dropoffs[p.id]
            dropoff_link[(p.id, k)] = @constraint(m, -y[k] <= 0.0)
        end
    end

    # No-vehicle-route options, added into the SAME coverage and linking rows the
    # route columns use. Sharing the linking rows (rather than giving these their
    # own) keeps `gamma^O_pj`/`gamma^D_pj` meaning "the price of using station j
    # for passenger p", however p gets there, and makes the row
    # `sum_r a^O_rpj theta_r + x_same[p,j] <= y[j]` tighter than two separate rows:
    # it forbids paying for both a route pickup and a walk-only leg at the same
    # station for the same passenger, which is never useful under `>=` coverage.
    x_same = Dict{Tuple{Int, Int}, VariableRef}()
    for p in master_data.passengers
        for j in master_data.same_station_options[p.id]
            # No explicit upper bound: `x_same[p,j] <= y[j] <= 1` already implies it
            # via the pickup row. Stating it again would add a dual variable that the
            # dual selector (dual_selection.jl) would then have to carry for nothing.
            x = @variable(m, lower_bound = 0.0, base_name = "x_same[$(p.id),$j]")
            x_same[(p.id, j)] = x
            set_normalized_coefficient(coverage[p.id], x, 1.0)
            haskey(pickup_link, (p.id, j)) && set_normalized_coefficient(pickup_link[(p.id, j)], x, 1.0)
            haskey(dropoff_link, (p.id, j)) && set_normalized_coefficient(dropoff_link[(p.id, j)], x, 1.0)
        end
    end
    m[:x_same] = x_same

    station_budget = @constraint(m, sum(y) == master_data.l)
    m[:station_budget] = station_budget
    @objective(m, Min,
        sum(master_data.unserved_penalty * v[p.id] for p in master_data.passengers; init = 0.0) +
        sum(master_data.walk_cost_weight * master_data.same_station_walk_cost[key] * var
            for (key, var) in x_same; init = 0.0),
    )

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
    add_passenger_free_assignment_column!(master, column) -> (theta, action)

`action` is `:added`, or `:skipped` when an identical assignment signature is
already in the pool at no greater `tau` (the pool keeps the cheapest route per
signature, mirroring the pricer's own dedup rule).
"""
function add_passenger_free_assignment_column!(
    master::PassengerFreeAssignmentMaster,
    column::PassengerFreeAssignmentRouteColumn,
)
    signature = _passenger_free_assignment_column_signature(column)
    existing_id = get(master.column_signatures, signature, nothing)
    if !isnothing(existing_id)
        master.columns[existing_id].tau <= column.tau + 1e-9 &&
            return master.theta[existing_id], :skipped
    end

    m = master.model
    md = master.master_data
    theta = @variable(m, lower_bound = 0.0, base_name = "theta[$(column.id)]")
    master.theta[column.id] = theta
    master.columns[column.id] = column
    master.column_signatures[signature] = column.id

    set_objective_coefficient(m, theta, passenger_free_assignment_column_cost(column, md))
    for (p, j, k) in column.assignments
        haskey(master.coverage, p) || continue
        set_normalized_coefficient(master.coverage[p], theta, 1.0)
        haskey(master.pickup_link, (p, j)) &&
            set_normalized_coefficient(master.pickup_link[(p, j)], theta, 1.0)
        haskey(master.dropoff_link, (p, k)) &&
            set_normalized_coefficient(master.dropoff_link[(p, k)], theta, 1.0)
    end
    return theta, :added
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
    passenger_free_assignment_station_reduced_cost_eliminations(candidates, y_value, y_lower_rc; kwargs...)

Certified iteration-only station eliminations from closed-station reduced-cost
slack. For each closed station `j`, the test computes the minimum increase in
that station's linking duals needed to make every positive pricing opportunity
incident to `j` nonpositive:

    Q_j = sum_p (max_k [rho_pjk]_+ + max_i [rho_pij]_+)

If `Q_j <= rc(y_j) - tol`, the same RMP-optimal dual face contains a dual point
where station `j` has no positive-reward route-pricing endpoint, so all
opportunities incident to `j` may be dropped for this pricing iteration.

This passenger master has `x_same[p,j]` variables in the RMP, not `(p,j,j)`
route-pricing opportunities, so there is no same-station `C_pj` term here:
increasing the linking duals only relaxes those existing `x_same` constraints.
"""
function passenger_free_assignment_station_reduced_cost_eliminations(
    candidates::AbstractVector{PassengerAssignmentCandidate},
    y_value::AbstractVector{<:Real},
    y_lower_rc::AbstractVector{<:Real};
    closed_tol::Float64=1e-7,
    reward_tol::Float64=1e-9,
    slack_tol::Float64=1e-7,
)
    length(y_value) == length(y_lower_rc) ||
        throw(ArgumentError("y_value and y_lower_rc must have the same length"))
    n = length(y_value)
    origin_need = Dict{Tuple{Int, Int}, Float64}()
    destination_need = Dict{Tuple{Int, Int}, Float64}()
    for c in candidates
        c.reward > reward_tol || continue
        1 <= c.origin <= n || throw(ArgumentError("candidate origin $(c.origin) outside 1:$n"))
        1 <= c.destination <= n ||
            throw(ArgumentError("candidate destination $(c.destination) outside 1:$n"))
        key_o = (c.passenger, c.origin)
        origin_need[key_o] = max(get(origin_need, key_o, 0.0), c.reward)
        key_d = (c.passenger, c.destination)
        destination_need[key_d] = max(get(destination_need, key_d, 0.0), c.reward)
    end

    required = zeros(Float64, n)
    passenger_station = union(keys(origin_need), keys(destination_need))
    for key in passenger_station
        _p, j = key
        required[j] += get(origin_need, key, 0.0) + get(destination_need, key, 0.0)
    end

    eliminated = Set{Int}()
    for j in 1:n
        y_value[j] <= closed_tol || continue
        y_lower_rc[j] > slack_tol || continue
        required[j] <= y_lower_rc[j] - slack_tol && push!(eliminated, j)
    end
    return eliminated, required
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
