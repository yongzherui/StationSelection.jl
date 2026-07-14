"""
Constraints and dynamic column update helpers for AggregateODRouteModel.
"""

export add_assignment_to_selected_constraints!
export add_aggregate_od_route_coverage_constraints!
export add_aggregate_od_route_column!
export add_or_update_aggregate_od_route_column!
export aggregate_od_route_column_objective_coefficient
export add_nearest_open_assignment_constraints!
export validate_big_m_nearest_aggregate_od_route!
export add_fixed_open_station_constraints!

const _AggregateODRouteEndpointChainKey = Tuple{Symbol, Tuple{Int, Vararg{Int}}, Tuple{Float64, Vararg{Float64}}}

function add_assignment_to_selected_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
)::Int
    before = _total_num_constraints(m)
    y = m[:y]
    x = m[:x]
    for s in 1:n_scenarios(data)
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            for (pair_idx, (j, k)) in enumerate(get_valid_jk_pairs(mapping, o, d))
                @constraint(m, x_od[pair_idx] <= y[j])
                @constraint(m, x_od[pair_idx] <= y[k])
            end
        end
    end
    return _total_num_constraints(m) - before
end

function _aggregate_od_route_assignment_pair_cost(
    data::StationSelectionData,
    o::Int,
    d::Int,
    pair::Tuple{Int, Int},
)::Float64
    j, k = pair
    return get_walking_cost(data, o, j) + get_walking_cost(data, k, d)
end

function _rank_aggregate_od_route_pairs_by_assignment_cost(
    data::StationSelectionData,
    o::Int,
    d::Int,
    pairs::AbstractVector{Tuple{Int, Int}},
)::Vector{Int}
    idxs = collect(eachindex(pairs))
    sort!(idxs, by=i -> (_aggregate_od_route_assignment_pair_cost(data, o, d, pairs[i]), pairs[i][1], pairs[i][2]))
    return idxs
end

function add_nearest_open_assignment_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
)::Int
    before = _total_num_constraints(m)
    y = m[:y]
    x = m[:x]
    for s in 1:n_scenarios(data)
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            pairs = get_valid_jk_pairs(mapping, o, d)
            ranked_pair_idxs = _rank_aggregate_od_route_pairs_by_assignment_cost(data, o, d, pairs)
            for rank_pos in 2:length(ranked_pair_idxs)
                pair_idx = ranked_pair_idxs[rank_pos]
                for prior_rank_pos in 1:(rank_pos - 1)
                    prior_pair_idx = ranked_pair_idxs[prior_rank_pos]
                    prior_j, prior_k = pairs[prior_pair_idx]
                    @constraint(m, x_od[pair_idx] <= 2.0 - y[prior_j] - y[prior_k])
                end
            end
        end
    end
    return _total_num_constraints(m) - before
end

function _endpoint_chain_key(
    side::Symbol,
    endpoints::Vector{Int},
    costs::Vector{Float64},
)::_AggregateODRouteEndpointChainKey
    return (side, Tuple(endpoints), Tuple(round.(costs; digits=9)))
end

function _endpoint_chain_variable!(
    m::Model,
    y,
    side::Symbol,
    endpoints::Vector{Int},
    costs::Vector{Float64},
)
    cache = if haskey(m, :nearest_endpoint_chain_cache)
        m[:nearest_endpoint_chain_cache]
    else
        m[:nearest_endpoint_chain_cache] = Dict{_AggregateODRouteEndpointChainKey, Vector{VariableRef}}()
    end
    order = sortperm(collect(eachindex(endpoints)); by=i -> (costs[i], endpoints[i]))
    sorted_endpoints = endpoints[order]
    sorted_costs = costs[order]
    key = _endpoint_chain_key(side, sorted_endpoints, sorted_costs)
    return get!(cache, key) do
        z = @variable(m, [1:length(sorted_endpoints)], binary = true)
        @constraint(m, sum(z) == 1.0)
        for (rank, station) in enumerate(sorted_endpoints)
            @constraint(m, z[rank] <= y[station])
            for prior in 1:(rank - 1)
                @constraint(m, z[rank] <= 1.0 - y[sorted_endpoints[prior]])
            end
        end
        z
    end, sorted_endpoints
end

function _validate_endpoint_cartesian!(
    data::StationSelectionData,
    o::Int,
    d::Int,
    pairs::Vector{Tuple{Int, Int}},
)::Tuple{Vector{Int}, Vector{Int}}
    origins = sort!(unique(first.(pairs)))
    destinations = sort!(unique(last.(pairs)))
    pair_set = Set(pairs)
    length(pairs) == length(origins) * length(destinations) &&
        all((j, k) in pair_set for j in origins for k in destinations) ||
        throw(ArgumentError(
            ":big_m_nearest requires feasible pairs for OD $((o, d)) " *
            "to form the full pickup/dropoff Cartesian product"
        ))
    all(isfinite(get_walking_cost(data, o, j)) for j in origins) ||
        throw(ArgumentError(":big_m_nearest has non-finite pickup walking cost for OD $((o, d))"))
    all(isfinite(get_walking_cost(data, k, d)) for k in destinations) ||
        throw(ArgumentError(":big_m_nearest has non-finite dropoff walking cost for OD $((o, d))"))
    return origins, destinations
end

function add_nearest_open_endpoint_chain_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
)::Int
    before = _total_num_constraints(m)
    y = m[:y]
    x = m[:x]
    for s in 1:n_scenarios(data)
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            pairs = get_valid_jk_pairs(mapping, o, d)
            pickups, dropoffs = _validate_endpoint_cartesian!(data, o, d, pairs)
            pickup_costs = [get_walking_cost(data, o, j) for j in pickups]
            dropoff_costs = [get_walking_cost(data, k, d) for k in dropoffs]
            zp, sorted_pickups = _endpoint_chain_variable!(m, y, :pickup, pickups, pickup_costs)
            zd, sorted_dropoffs = _endpoint_chain_variable!(m, y, :dropoff, dropoffs, dropoff_costs)
            pickup_rank = Dict(station => idx for (idx, station) in enumerate(sorted_pickups))
            dropoff_rank = Dict(station => idx for (idx, station) in enumerate(sorted_dropoffs))
            for (pair_idx, (j, k)) in enumerate(pairs)
                @constraint(m, x_od[pair_idx] <= zp[pickup_rank[j]])
                @constraint(m, x_od[pair_idx] <= zd[dropoff_rank[k]])
            end
        end
    end
    return _total_num_constraints(m) - before
end

function validate_big_m_nearest_aggregate_od_route!(
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
)::Nothing
    for s in 1:n_scenarios(data)
        for (o, d) in mapping.Omega_s[s]
            pairs = get_valid_jk_pairs(mapping, o, d)
            isempty(pairs) && continue
            _validate_endpoint_cartesian!(data, o, d, pairs)
        end
    end
    return nothing
end

function add_fixed_open_station_constraints!(
    m::Model,
    data::StationSelectionData,
    model::RouteCoveringProblem,
)::Int
    before = _total_num_constraints(m)
    y = m[:y]
    open_set = Set(model.open_stations)
    for station in 1:data.n_stations
        if station in open_set
            @constraint(m, y[station] == 1.0)
        else
            @constraint(m, y[station] == 0.0)
        end
    end
    return _total_num_constraints(m) - before
end

function add_aggregate_od_route_coverage_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap;
    equality::Bool=false,
)::Int
    before = _total_num_constraints(m)
    x = m[:x]
    theta = m[:theta_compat]
    coverage = Dict{NTuple{5, Int}, ConstraintRef}()
    coverage_by_pair_s = Dict{NTuple{3, Int}, Vector{ConstraintRef}}()
    for s in 1:n_scenarios(data)
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            for (pair_idx, (j, k)) in enumerate(get_valid_jk_pairs(mapping, o, d))
                expr = AffExpr(0.0)
                for column_id in get(mapping.columns_by_pair, (j, k), Int[])
                    theta_var = get(theta, (column_id, s), nothing)
                    theta_var === nothing && continue
                    add_to_expression!(expr, 1.0, theta_var)
                end
                con = equality ? @constraint(m, expr - x_od[pair_idx] == 0.0) :
                                 @constraint(m, expr - x_od[pair_idx] >= 0.0)
                coverage[(j, k, s, od_idx, pair_idx)] = con
                push!(get!(coverage_by_pair_s, (j, k, s), ConstraintRef[]), con)
            end
        end
    end
    m[:aggregate_od_route_coverage_constraints] = coverage
    m[:aggregate_od_route_coverage_by_pair_s] = coverage_by_pair_s
    return _total_num_constraints(m) - before
end

aggregate_od_route_column_objective_coefficient(
    route_regularization_weight::Real,
    repositioning_time::Real,
    column::AggregateODRouteColumn,
) = Float64(route_regularization_weight) * (column.tau + Float64(repositioning_time))

function add_aggregate_od_route_column!(
    m::Model,
    mapping::AggregateODRouteMap,
    column::AggregateODRouteColumn,
)::AggregateODRouteColumn
    _register_aggregate_od_route_column_metadata!(mapping, column)

    S = length(mapping.scenarios)
    relax_integrality = Bool(m[:aggregate_od_route_relax_integrality])
    mu = Float64(m[:aggregate_od_route_route_regularization_weight])
    rho = Float64(m[:aggregate_od_route_repositioning_time])
    theta = m[:theta_compat]
    coverage_by_pair_s = m[:aggregate_od_route_coverage_by_pair_s]

    obj_coef = aggregate_od_route_column_objective_coefficient(mu, rho, column)
    for s in 1:S
        theta_var = relax_integrality ?
            @variable(m, lower_bound = 0.0, upper_bound = 1.0) :
            @variable(m, binary = true)
        theta[(column.id, s)] = theta_var
        set_objective_coefficient(m, theta_var, obj_coef)

        for (j, k) in column.od_pairs
            for con in get(coverage_by_pair_s, (j, k, s), ConstraintRef[])
                set_normalized_coefficient(con, theta_var, 1.0)
            end
        end
    end
    return column
end

function _aggregate_od_route_column_signature_from_pairs(pairs)
    return Tuple(sort!(collect(pairs)))
end

_aggregate_od_route_column_signature_for_update(column::AggregateODRouteColumn) =
    _aggregate_od_route_column_signature_from_pairs(column.od_pairs)

function _replace_aggregate_od_route_column_metadata!(
    mapping::AggregateODRouteMap,
    existing_idx::Int,
    column::AggregateODRouteColumn,
)::AggregateODRouteColumn
    existing = mapping.columns[existing_idx]
    replacement = AggregateODRouteColumn(
        existing.id,
        existing.od_pairs,
        column.tau;
        metadata=column.metadata,
    )
    mapping.columns[existing_idx] = replacement
    return replacement
end

function add_or_update_aggregate_od_route_column!(
    m::Model,
    mapping::AggregateODRouteMap,
    column::AggregateODRouteColumn,
)::Tuple{Union{VariableRef, Nothing}, Symbol}
    signature = _aggregate_od_route_column_signature_for_update(column)
    existing_idx = findfirst(
        existing -> _aggregate_od_route_column_signature_for_update(existing) == signature,
        mapping.columns,
    )

    if !isnothing(existing_idx)
        existing = mapping.columns[existing_idx]
        theta = m[:theta_compat]
        if column.tau < existing.tau - 1e-9
            replacement = _replace_aggregate_od_route_column_metadata!(mapping, existing_idx, column)
            mu = Float64(m[:aggregate_od_route_route_regularization_weight])
            rho = Float64(m[:aggregate_od_route_repositioning_time])
            obj_coef = aggregate_od_route_column_objective_coefficient(mu, rho, replacement)
            for s in 1:length(mapping.scenarios)
                theta_var = get(theta, (replacement.id, s), nothing)
                theta_var === nothing && continue
                set_objective_coefficient(m, theta_var, obj_coef)
            end
            return get(theta, (replacement.id, 1), nothing), :replaced
        end
        return get(theta, (existing.id, 1), nothing), :skipped
    end

    added = add_aggregate_od_route_column!(m, mapping, column)
    return get(m[:theta_compat], (added.id, 1), nothing), :added
end

function add_or_update_aggregate_od_route_column!(
    build_result::BuildResult,
    column::AggregateODRouteColumn,
)::Tuple{Union{VariableRef, Nothing}, Symbol}
    return add_or_update_aggregate_od_route_column!(build_result.model, build_result.mapping, column)
end

function add_aggregate_od_route_column!(
    build_result::BuildResult,
    column::AggregateODRouteColumn,
)::AggregateODRouteColumn
    return add_aggregate_od_route_column!(build_result.model, build_result.mapping, column)
end
