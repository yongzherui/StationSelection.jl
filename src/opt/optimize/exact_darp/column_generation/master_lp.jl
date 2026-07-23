export build_exact_darp_route_restricted_master
export extract_exact_darp_route_cg_duals

function build_exact_darp_route_restricted_master(
    model::ExactDARPRouteModel,
    data::StationSelectionData,
    route_pool_state::ExactDARPRouteBucketPoolsState;
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
        repositioning_time=model.repositioning_time,
    )
    set_optimizer_attribute(m, "Method", 1)
    set_optimizer_attribute(m, "Presolve", 0)

    return build_result
end

function extract_exact_darp_route_cg_duals(m::Model)::ExactDARPRouteCGDuals
    constraint_refs = get(m.obj_dict, :arm_capacity_constraints, Dict{NTuple{4, Int}, ConstraintRef}())
    route_capacity_duals = Dict{NTuple{4, Int}, Float64}()
    raw_route_capacity_duals = Dict{NTuple{4, Int}, Float64}()
    for (key, con_ref) in constraint_refs
        raw_dual = dual(con_ref)
        raw_route_capacity_duals[key] = raw_dual
        route_capacity_duals[key] = raw_dual < 0.0 ? -raw_dual : raw_dual
    end
    return ExactDARPRouteCGDuals(route_capacity_duals, raw_route_capacity_duals)
end
