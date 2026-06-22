"""
Constraints and dynamic column update helpers for CompatibilitySetModel.
"""

export add_assignment_to_selected_constraints!
export add_assignment_to_od_activation_constraints!
export add_compatibility_coverage_constraints!
export add_compatibility_column!
export add_or_update_compatibility_column!
export compatibility_column_objective_coefficient

function add_assignment_to_selected_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::CompatibilitySetODMap,
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
                @constraint(m, x_od[pair_idx] <= demand * y[j])
                @constraint(m, x_od[pair_idx] <= demand * y[k])
            end
        end
    end
    return _total_num_constraints(m) - before
end

function add_assignment_to_od_activation_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::CompatibilitySetODMap,
)::Int
    before = _total_num_constraints(m)
    x = m[:x]
    u = m[:u]
    for s in 1:n_scenarios(data)
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            for (pair_idx, (j, k)) in enumerate(get_valid_jk_pairs(mapping, o, d))
                u_var = get(u, (j, k, s), nothing)
                u_var === nothing && continue
                @constraint(m, x_od[pair_idx] <= demand * u_var)
            end
        end
    end
    return _total_num_constraints(m) - before
end

function add_compatibility_coverage_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::CompatibilitySetODMap;
    equality::Bool=false,
)::Int
    before = _total_num_constraints(m)
    u = m[:u]
    theta = m[:theta_compat]
    coverage = Dict{NTuple{3, Int}, ConstraintRef}()
    for s in 1:n_scenarios(data)
        for (j, k) in get(mapping.active_jk_s, s, Tuple{Int, Int}[])
            expr = AffExpr(0.0)
            for column_id in get(mapping.columns_by_pair, (j, k), Int[])
                theta_var = get(theta, (column_id, s), nothing)
                theta_var === nothing && continue
                add_to_expression!(expr, 1.0, theta_var)
            end
            u_var = get(u, (j, k, s), nothing)
            u_var === nothing && continue
            con = equality ? @constraint(m, expr - u_var == 0.0) :
                             @constraint(m, expr - u_var >= 0.0)
            coverage[(j, k, s)] = con
        end
    end
    m[:compatibility_coverage_constraints] = coverage
    return _total_num_constraints(m) - before
end

compatibility_column_objective_coefficient(
    route_regularization_weight::Real,
    repositioning_time::Real,
    column::CompatibilityColumn,
) = Float64(route_regularization_weight) * (column.tau + Float64(repositioning_time))

function add_compatibility_column!(
    m::Model,
    mapping::CompatibilitySetODMap,
    column::CompatibilityColumn,
)::CompatibilityColumn
    _register_compatibility_column_metadata!(mapping, column)

    S = length(mapping.scenarios)
    relax_integrality = Bool(m[:compatibility_relax_integrality])
    mu = Float64(m[:compatibility_route_regularization_weight])
    rho = Float64(m[:compatibility_repositioning_time])
    theta = m[:theta_compat]
    coverage = m[:compatibility_coverage_constraints]

    obj_coef = compatibility_column_objective_coefficient(mu, rho, column)
    for s in 1:S
        theta_var = relax_integrality ?
            @variable(m, lower_bound = 0.0, upper_bound = 1.0) :
            @variable(m, binary = true)
        theta[(column.id, s)] = theta_var
        set_objective_coefficient(m, theta_var, obj_coef)

        for (j, k) in column.od_pairs
            con = get(coverage, (j, k, s), nothing)
            con === nothing && continue
            set_normalized_coefficient(con, theta_var, 1.0)
        end
    end
    return column
end

function _compatibility_column_signature_from_pairs(pairs)
    return Tuple(sort!(collect(pairs)))
end

_compatibility_column_signature_for_update(column::CompatibilityColumn) =
    _compatibility_column_signature_from_pairs(column.od_pairs)

function _replace_compatibility_column_metadata!(
    mapping::CompatibilitySetODMap,
    existing_idx::Int,
    column::CompatibilityColumn,
)::CompatibilityColumn
    existing = mapping.columns[existing_idx]
    replacement = CompatibilityColumn(
        existing.id,
        existing.od_pairs,
        column.tau;
        metadata=column.metadata,
    )
    mapping.columns[existing_idx] = replacement
    return replacement
end

function add_or_update_compatibility_column!(
    m::Model,
    mapping::CompatibilitySetODMap,
    column::CompatibilityColumn,
)::Tuple{Union{VariableRef, Nothing}, Symbol}
    signature = _compatibility_column_signature_for_update(column)
    existing_idx = findfirst(
        existing -> _compatibility_column_signature_for_update(existing) == signature,
        mapping.columns,
    )

    if !isnothing(existing_idx)
        existing = mapping.columns[existing_idx]
        theta = m[:theta_compat]
        if column.tau < existing.tau - 1e-9
            replacement = _replace_compatibility_column_metadata!(mapping, existing_idx, column)
            mu = Float64(m[:compatibility_route_regularization_weight])
            rho = Float64(m[:compatibility_repositioning_time])
            obj_coef = compatibility_column_objective_coefficient(mu, rho, replacement)
            for s in 1:length(mapping.scenarios)
                theta_var = get(theta, (replacement.id, s), nothing)
                theta_var === nothing && continue
                set_objective_coefficient(m, theta_var, obj_coef)
            end
            return get(theta, (replacement.id, 1), nothing), :replaced
        end
        return get(theta, (existing.id, 1), nothing), :skipped
    end

    added = add_compatibility_column!(m, mapping, column)
    return get(m[:theta_compat], (added.id, 1), nothing), :added
end

function add_or_update_compatibility_column!(
    build_result::BuildResult,
    column::CompatibilityColumn,
)::Tuple{Union{VariableRef, Nothing}, Symbol}
    return add_or_update_compatibility_column!(build_result.model, build_result.mapping, column)
end

function add_compatibility_column!(
    build_result::BuildResult,
    column::CompatibilityColumn,
)::CompatibilityColumn
    return add_compatibility_column!(build_result.model, build_result.mapping, column)
end
