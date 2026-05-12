"""
Fixed-station operational assignment helpers.

These utilities support transform-time operational re-assignment:
1. Load exported fixed station decisions (y* / z*) from a selection run
2. Solve a RouteVehicleCapacityModel operational subproblem with y/z fixed
3. Export route-model-shaped assignment artifacts for downstream transformation
"""

export FixedStationDecisions
export load_fixed_station_decisions
export solve_fixed_station_operational_assignment

struct FixedStationDecisions
    built_station_ids::Set{Int}
    active_station_ids_by_scenario::Dict{Int, Set{Int}}
    scenario_labels::Dict{Int, String}
    source_selection_run_dir::String
    z_available::Bool
    y_fallback_scenarios::Vector{Int}
end

function _load_fixed_station_selection(station_file::String)::Set{Int}
    isfile(station_file) || error("station_selection.csv not found: $station_file")
    df = CSV.read(station_file, DataFrame)
    cols = Set(names(df))
    if "selected" in cols
        return Set(Int(row.station_id) for row in eachrow(df) if row.selected >= 0.5)
    elseif "value" in cols
        return Set(Int(row.station_id) for row in eachrow(df) if row.value >= 0.5)
    end
    error("station_selection.csv must contain either 'selected' or 'value'")
end

function _load_scenario_labels(variable_exports_dir::String)
    scenario_file = joinpath(variable_exports_dir, "scenario_info.csv")
    labels = Dict{Int, String}()
    if isfile(scenario_file)
        scenario_df = CSV.read(scenario_file, DataFrame)
        for row in eachrow(scenario_df)
            labels[Int(row.scenario_idx)] = string(row.label)
        end
    end
    return labels
end

function load_fixed_station_decisions(
    selection_run_dir::String;
    fallback_to_y::Bool=true,
    scenario_indices::Union{Nothing, AbstractVector{Int}}=nothing
)::FixedStationDecisions
    variable_exports_dir = joinpath(selection_run_dir, "variable_exports")
    isdir(variable_exports_dir) || error("variable_exports directory not found: $variable_exports_dir")

    built_station_ids = _load_fixed_station_selection(joinpath(variable_exports_dir, "station_selection.csv"))
    scenario_labels = _load_scenario_labels(variable_exports_dir)

    activation_file = joinpath(variable_exports_dir, "scenario_activation.csv")
    active_station_ids_by_scenario = Dict{Int, Set{Int}}()
    z_available = isfile(activation_file)

    if z_available
        activation_df = CSV.read(activation_file, DataFrame)
        activation_cols = Set(names(activation_df))
        for row in eachrow(activation_df)
            row.value >= 0.5 || continue
            s = Int(row.scenario_idx)
            push!(get!(active_station_ids_by_scenario, s, Set{Int}()), Int(row.station_id))
            if "scenario_label" in activation_cols
                get!(scenario_labels, s, string(row.scenario_label))
            else
                get!(scenario_labels, s, "scenario_$s")
            end
        end
    end

    scenario_ids = if !isnothing(scenario_indices)
        sort!(collect(Int.(scenario_indices)))
    elseif !isempty(scenario_labels)
        sort!(collect(keys(scenario_labels)))
    elseif !isempty(active_station_ids_by_scenario)
        sort!(collect(keys(active_station_ids_by_scenario)))
    else
        [1]
    end

    y_fallback_scenarios = Int[]
    for s in scenario_ids
        if !haskey(active_station_ids_by_scenario, s) || isempty(active_station_ids_by_scenario[s])
            if fallback_to_y
                active_station_ids_by_scenario[s] = copy(built_station_ids)
                push!(y_fallback_scenarios, s)
            else
                active_station_ids_by_scenario[s] = Set{Int}()
            end
        end
        get!(scenario_labels, s, "scenario_$s")
    end

    for (s, active_ids) in active_station_ids_by_scenario
        issubset(active_ids, built_station_ids) || error(
            "Invalid fixed station decisions: scenario $s has active stations not present in built set"
        )
    end

    return FixedStationDecisions(
        built_station_ids,
        active_station_ids_by_scenario,
        scenario_labels,
        selection_run_dir,
        z_available,
        y_fallback_scenarios,
    )
end

function _fix_station_decisions!(
    m::JuMP.Model,
    mapping::AbstractStationSelectionMap,
    fixed::FixedStationDecisions
)
    y = m[:y]
    z = m[:z]
    id_to_idx = mapping.station_id_to_array_idx
    scenario_count = length(mapping.scenarios)

    unknown_y = sort!(collect(setdiff(fixed.built_station_ids, keys(id_to_idx))))
    isempty(unknown_y) || error("Fixed built station IDs are not in candidate station set: $unknown_y")

    for station_id in fixed.built_station_ids
        i = id_to_idx[station_id]
        JuMP.fix(y[i], 1.0; force=true)
    end
    for station_id in setdiff(Set(keys(id_to_idx)), fixed.built_station_ids)
        i = id_to_idx[station_id]
        JuMP.fix(y[i], 0.0; force=true)
    end

    for s in 1:scenario_count
        active_ids = get(fixed.active_station_ids_by_scenario, s, fixed.built_station_ids)
        unknown_z = sort!(collect(setdiff(active_ids, keys(id_to_idx))))
        isempty(unknown_z) || error("Fixed active station IDs are not in candidate station set for scenario $s: $unknown_z")
        for station_id in active_ids
            i = id_to_idx[station_id]
            JuMP.fix(z[i, s], 1.0; force=true)
        end
        for station_id in setdiff(Set(keys(id_to_idx)), active_ids)
            i = id_to_idx[station_id]
            JuMP.fix(z[i, s], 0.0; force=true)
        end
    end
end

function _build_fixed_station_operational_model(
    model::RouteVehicleCapacityModel,
    data::StationSelectionData,
    fixed::FixedStationDecisions;
    optimizer_env=nothing
)::BuildResult
    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    mapping = create_map(model, data)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    m[:vehicle_capacity] = model.vehicle_capacity

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts = Dict{String, Int}()

    n_routes = sum(
        sum(length(v) for v in values(mapping.routes_s[s]); init = 0)
        for s in 1:length(mapping.scenarios); init = 0
    )
    total_od_pairs = sum(length(mapping.Omega_s[s]) for s in 1:length(mapping.scenarios); init = 0)
    extra_counts["n_routes"] = n_routes
    extra_counts["total_od_pairs"] = total_od_pairs

    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    variable_counts["scenario_activation"] = add_scenario_activation_variables!(m, data)
    variable_counts["assignment"] = add_assignment_variables!(m, data, mapping)
    variable_counts["alpha_r_jkts"] = add_alpha_r_jkts_variables!(m, data, mapping; integer_alpha=false)
    variable_counts["theta_r_ts"] = add_theta_r_ts_variables!(m, data, mapping)

    set_route_od_objective!(
        m, data, mapping;
        route_regularization_weight=model.route_regularization_weight,
        repositioning_time=model.repositioning_time,
    )

    constraint_counts["activation_linking"] = add_activation_linking_constraints!(m, data)
    constraint_counts["assignment"] = add_assignment_constraints!(m, data, mapping)
    constraint_counts["assignment_to_active"] = add_assignment_to_active_constraints!(m, data, mapping)
    if model.use_lazy_constraints
        constraint_counts["route_capacity"] = add_route_capacity_lazy_constraints!(m, data, mapping)
    else
        constraint_counts["route_capacity"] = add_route_capacity_constraints!(m, data, mapping)
    end

    _fix_station_decisions!(m, mapping, fixed)

    metadata = Dict{String, Any}(
        "fixed_station_mode" => true,
        "fixed_y_count" => length(fixed.built_station_ids),
        "fixed_z_scenarios" => length(fixed.active_station_ids_by_scenario),
        "z_available" => fixed.z_available,
        "y_fallback_scenarios" => fixed.y_fallback_scenarios,
    )

    return BuildResult(
        m,
        mapping,
        nothing,
        ModelCounts(variable_counts, constraint_counts, extra_counts),
        metadata,
    )
end

function _operational_result_metadata(
    model::RouteVehicleCapacityModel,
    fixed::FixedStationDecisions,
    build_result::BuildResult,
    build_time_sec::Float64,
    solve_time_sec::Float64
)::Dict{String, Any}
    return Dict{String, Any}(
        "model_type" => "RouteVehicleCapacityModel",
        "source_selection_run_dir" => fixed.source_selection_run_dir,
        "fixed_station_mode" => true,
        "z_available" => fixed.z_available,
        "y_fallback_scenarios" => fixed.y_fallback_scenarios,
        "n_built_stations" => length(fixed.built_station_ids),
        "n_active_station_assignments" => sum(length(v) for v in values(fixed.active_station_ids_by_scenario)),
        "route_regularization_weight" => model.route_regularization_weight,
        "repositioning_time" => model.repositioning_time,
        "vehicle_capacity" => model.vehicle_capacity,
        "max_walking_distance" => model.max_walking_distance,
        "max_detour_time" => model.max_detour_time,
        "max_detour_ratio" => model.max_detour_ratio,
        "time_window_sec" => model.time_window_sec,
        "max_stations_visited" => model.max_stations_visited,
        "stop_dwell_time" => model.stop_dwell_time,
        "build_time_sec" => build_time_sec,
        "solve_time_sec" => solve_time_sec,
        "counts" => isnothing(build_result.counts) ? Dict{String, Any}() : Dict(
            "variables" => build_result.counts.variables,
            "constraints" => build_result.counts.constraints,
            "extras" => build_result.counts.extras,
        ),
    )
end

function solve_fixed_station_operational_assignment(
    model::RouteVehicleCapacityModel,
    data::StationSelectionData,
    fixed::FixedStationDecisions,
    output_dir::String;
    optimizer_env=nothing,
    silent::Bool=false,
    mip_gap::Union{Float64, Nothing}=nothing
)
    mkpath(output_dir)

    build_start = now()
    build_result = _build_fixed_station_operational_model(
        model, data, fixed; optimizer_env=optimizer_env
    )
    build_time_sec = Dates.value(now() - build_start) / 1000

    m = build_result.model
    if silent
        set_silent(m)
    end
    if !isnothing(mip_gap)
        set_optimizer_attribute(m, "MIPGap", mip_gap)
    end

    solve_start = now()
    optimize!(m)
    solve_time_sec = Dates.value(now() - solve_start) / 1000
    term_status = JuMP.termination_status(m)

    metadata = _operational_result_metadata(model, fixed, build_result, build_time_sec, solve_time_sec)
    metadata["termination_status"] = string(term_status)

    if term_status != MOI.OPTIMAL
        error("Fixed-station operational assignment solve failed with status $term_status")
    end

    objective_value = JuMP.objective_value(m)
    metadata["objective_value"] = objective_value

    result = OptResult(
        term_status,
        objective_value,
        nothing,
        build_time_sec + solve_time_sec,
        m,
        build_result.mapping,
        nothing,
        build_result.counts,
        nothing,
        metadata,
    )

    export_variables(result, output_dir)

    return (
        result = result,
        export_dir = joinpath(output_dir, "variable_exports"),
        fixed_decisions = fixed,
    )
end

function solve_fixed_station_operational_assignment(
    selection_config::Dict,
    selection_run_dir::String,
    project_root::String,
    output_dir::String;
    route_regularization_weight::Number = 0.0,
    repositioning_time::Number = 20.0,
    vehicle_capacity::Int = 18,
    max_route_travel_time::Union{Number, Nothing} = nothing,
    max_walking_distance::Number = 300.0,
    max_detour_time::Number = 1200.0,
    max_detour_ratio::Number = 2.0,
    time_window_sec::Int = 3600,
    use_lazy_constraints::Bool = false,
    max_stations_visited::Int = typemax(Int),
    stop_dwell_time::Number = 10.0,
    mip_gap::Union{Float64, Nothing} = nothing,
    silent::Bool = false,
    optimizer_env = nothing
)
    data = build_station_selection_data_from_config(selection_config, project_root)
    scenario_indices = collect(1:n_scenarios(data))
    fixed = load_fixed_station_decisions(
        selection_run_dir;
        fallback_to_y=true,
        scenario_indices=scenario_indices,
    )

    l = max(length(fixed.built_station_ids), 1)
    k = max(maximum(length(ids) for ids in values(fixed.active_station_ids_by_scenario)), 1)
    l = max(l, k)

    model = RouteVehicleCapacityModel(
        k,
        l;
        route_regularization_weight=route_regularization_weight,
        repositioning_time=repositioning_time,
        vehicle_capacity=vehicle_capacity,
        max_route_travel_time=max_route_travel_time,
        max_walking_distance=max_walking_distance,
        max_detour_time=max_detour_time,
        max_detour_ratio=max_detour_ratio,
        time_window_sec=time_window_sec,
        use_lazy_constraints=use_lazy_constraints,
        max_stations_visited=max_stations_visited,
        stop_dwell_time=stop_dwell_time,
    )

    return solve_fixed_station_operational_assignment(
        model,
        data,
        fixed,
        output_dir;
        optimizer_env=optimizer_env,
        silent=silent,
        mip_gap=mip_gap,
    )
end
