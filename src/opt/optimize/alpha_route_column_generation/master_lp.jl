export build_alpha_route_restricted_master
export extract_alpha_route_cg_duals

function build_alpha_route_restricted_master(
    model::AlphaRouteModel,
    data::StationSelectionData,
    route_pool_state::AlphaRouteBucketPoolsState;
    optimizer_env=nothing
)::BuildResult
    build_result = build_model(
        model,
        data;
        optimizer_env=optimizer_env,
        route_pool_state=route_pool_state,
        relax_integrality=true,
        store_route_capacity_refs=true,
    )

    m = build_result.model
    set_route_od_objective!(
        m,
        data,
        build_result.mapping;
        route_regularization_weight=model.route_regularization_weight,
        repositioning_time=0.0,
    )
    set_optimizer_attribute(m, "Method", 1)
    set_optimizer_attribute(m, "Presolve", 0)

    return build_result
end

function extract_alpha_route_cg_duals(m::Model)::AlphaRouteCGDuals
    constraint_refs = get(m.obj_dict, :arm_capacity_constraints, Dict{NTuple{4, Int}, ConstraintRef}())
    route_capacity_duals = Dict{NTuple{4, Int}, Float64}()
    for (key, con_ref) in constraint_refs
        route_capacity_duals[key] = dual(con_ref)
    end
    return AlphaRouteCGDuals(route_capacity_duals)
end
