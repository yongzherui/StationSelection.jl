"""
Analysis helpers for exported optimization variables.

These functions compute model-implied walking distance and vehicle routing distance
from CSV exports created by `export_variables`.

Expected export files (under `export_dir`):
- assignment_variables.csv (required for walking distance)
- flow_variables.csv (optional; used for VRD with pooling when available)
"""

using CSV
using DataFrames
using Dates

import ..read_candidate_stations
import ..read_customer_requests
import ..compute_station_pairwise_costs
import ..read_routing_costs_from_segments
import ..generate_scenarios
import ..create_station_selection_data

export load_exported_assignment_variables
export load_exported_flow_variables
export load_exported_same_source_pooling
export load_exported_same_dest_pooling
export build_station_selection_data_from_config
export build_od_counts_from_data
export calculate_exported_walking_distance
export calculate_exported_vehicle_routing_distance

function load_exported_assignment_variables(export_dir::String)
    file = joinpath(export_dir, "assignment_variables.csv")
    isfile(file) || return DataFrame()
    filesize(file) == 0 && return DataFrame()
    df = CSV.read(file, DataFrame)
    ncol(df) == 0 && return df
    rename!(df, Dict(n => Symbol(n) for n in names(df)))
    return df
end

function load_exported_flow_variables(export_dir::String)
    file = joinpath(export_dir, "flow_variables.csv")
    isfile(file) || return DataFrame()
    filesize(file) == 0 && return DataFrame()
    df = CSV.read(file, DataFrame)
    ncol(df) == 0 && return df
    rename!(df, Dict(n => Symbol(n) for n in names(df)))
    return df
end

function load_exported_same_source_pooling(export_dir::String)
    file = joinpath(export_dir, "same_source_pooling.csv")
    isfile(file) || return DataFrame()
    filesize(file) == 0 && return DataFrame()
    df = CSV.read(file, DataFrame)
    ncol(df) == 0 && return df
    rename!(df, Dict(n => Symbol(n) for n in names(df)))
    return df
end

function load_exported_same_dest_pooling(export_dir::String)
    file = joinpath(export_dir, "same_dest_pooling.csv")
    isfile(file) || return DataFrame()
    filesize(file) == 0 && return DataFrame()
    df = CSV.read(file, DataFrame)
    ncol(df) == 0 && return df
    rename!(df, Dict(n => Symbol(n) for n in names(df)))
    return df
end

"""
    build_station_selection_data_from_config(config::Dict, project_root::String) -> StationSelectionData

Build StationSelectionData using the same inputs as the selection run.
"""
function build_station_selection_data_from_config(config::Dict, project_root::String)
    data_cfg = config["data"]
    params = config["parameters"]

    station_file = joinpath(project_root, data_cfg["station_file"])
    order_file = joinpath(project_root, data_cfg["order_file"])
    segment_file = joinpath(project_root, data_cfg["segment_file"])

    stations = read_candidate_stations(station_file)
    orders = read_customer_requests(
        order_file;
        start_time="$(params["start_date"]) 00:00:00",
        end_time="$(params["end_date"]) 23:59:59"
    )

    walking_costs = compute_station_pairwise_costs(stations)
    routing_costs = read_routing_costs_from_segments(segment_file, stations)

    scenarios = generate_scenarios(
        Date(params["start_date"], "yyyy-mm-dd"),
        Date(params["end_date"], "yyyy-mm-dd");
        segment_hours=get(params, "time_window_hours", 4),
        weekly_cycle=get(params, "weekly_cycle", false)
    )

    return create_station_selection_data(
        stations,
        orders,
        walking_costs;
        routing_costs=routing_costs,
        scenarios=scenarios
    )
end

"""
    build_od_counts_from_data(data::StationSelectionData) -> Vector{Dict}

Return OD counts per scenario (aligned with `data.scenarios` order).
"""
function build_od_counts_from_data(data::StationSelectionData)
    counts = Vector{Dict{Tuple{Int, Int}, Int}}()
    for scenario in data.scenarios
        d = Dict{Tuple{Int, Int}, Int}()
        for row in eachrow(scenario.requests)
            key = (row.start_station_id, row.end_station_id)
            d[key] = get(d, key, 0) + 1
        end
        push!(counts, d)
    end
    return counts
end

"""
    calculate_exported_walking_distance(export_dir::String, data::StationSelectionData) -> Float64

Compute total walking distance from exported assignment variables.
"""
function calculate_exported_walking_distance(
    export_dir::String,
    data::StationSelectionData;
    od_counts_by_scenario::Union{Nothing, Vector{Dict{Tuple{Int, Int}, Int}}}=nothing
)
    assign = load_exported_assignment_variables(export_dir)
    (isempty(assign) || ncol(assign) == 0) && return 0.0

    required = [:origin_id, :dest_id, :pickup_id, :dropoff_id, :value]
    cols_sym = Set(Symbol.(names(assign)))
    missing_cols = setdiff(required, cols_sym)
    if !isempty(missing_cols)
        file = joinpath(export_dir, "assignment_variables.csv")
        error("assignment_variables.csv missing columns: $(missing_cols). " *
              "Found: $(names(assign)). File: $file")
    end

    total = 0.0
    for row in eachrow(assign)
        val = row.value
        walking_pickup = get_walking_cost(data, row.origin_id, row.pickup_id)
        walking_dropoff = get_walking_cost(data, row.dropoff_id, row.dest_id)
        q = 1
        if !isnothing(od_counts_by_scenario) && :scenario in cols_sym
            s = row.scenario
            if s <= length(od_counts_by_scenario)
                q = get(od_counts_by_scenario[s], (row.origin_id, row.dest_id), 0)
            else
                q = 0
            end
        end
        total += (walking_pickup + walking_dropoff) * val * q
    end
    return total
end

"""
    calculate_exported_vehicle_routing_distance(
        export_dir::String,
        data::StationSelectionData;
        with_pooling::Bool=true
    ) -> Float64

Compute total vehicle routing distance from exported variables.
- If `with_pooling` is true and `flow_variables.csv` exists, use flow variables.
- Otherwise, fall back to assignment variables.
"""
function calculate_exported_vehicle_routing_distance(
    export_dir::String,
    data::StationSelectionData;
    with_pooling::Bool=true,
    od_counts_by_scenario::Union{Nothing, Vector{Dict{Tuple{Int, Int}, Int}}}=nothing
)
    if !has_routing_costs(data)
        @warn "Routing costs not available, returning 0"
        return 0.0
    end

    if with_pooling
        flows = load_exported_flow_variables(export_dir)
        if !(isempty(flows) || ncol(flows) == 0)
            required = [:j_id, :k_id, :value]
            cols_sym = Set(Symbol.(names(flows)))
            missing_cols = setdiff(required, cols_sym)
            isempty(missing_cols) || error("flow_variables.csv missing columns: $(missing_cols)")

            total = 0.0
            for row in eachrow(flows)
                total += get_routing_cost(data, row.j_id, row.k_id) * row.value
            end

            # Subtract pooling savings if u/v variables are available
            ss = load_exported_same_source_pooling(export_dir)
            if !(isempty(ss) || ncol(ss) == 0)
                required_ss = [:j_id, :k_id, :l_id, :value]
                cols_sym_ss = Set(Symbol.(names(ss)))
                missing_cols_ss = setdiff(required_ss, cols_sym_ss)
                isempty(missing_cols_ss) || error("same_source_pooling.csv missing columns: $(missing_cols_ss)")
                for row in eachrow(ss)
                    r = get_routing_cost(data, row.j_id, row.l_id) - get_routing_cost(data, row.k_id, row.l_id)
                    if r > 0
                        total -= r * row.value
                    end
                end
            end

            sd = load_exported_same_dest_pooling(export_dir)
            if !(isempty(sd) || ncol(sd) == 0)
                required_sd = [:j_id, :k_id, :l_id, :value]
                cols_sym_sd = Set(Symbol.(names(sd)))
                missing_cols_sd = setdiff(required_sd, cols_sym_sd)
                isempty(missing_cols_sd) || error("same_dest_pooling.csv missing columns: $(missing_cols_sd)")
                for row in eachrow(sd)
                    r = get_routing_cost(data, row.j_id, row.l_id) - get_routing_cost(data, row.j_id, row.k_id)
                    if r > 0
                        total -= r * row.value
                    end
                end
            end

            return total
        end
    end

    assign = load_exported_assignment_variables(export_dir)
    (isempty(assign) || ncol(assign) == 0) && return 0.0

    required = [:pickup_id, :dropoff_id, :value]
    cols_sym = Set(Symbol.(names(assign)))
    missing_cols = setdiff(required, cols_sym)
    isempty(missing_cols) || error("assignment_variables.csv missing columns: $(missing_cols)")

    total = 0.0
    for row in eachrow(assign)
        q = 1
        if !isnothing(od_counts_by_scenario) && :scenario in cols_sym
            s = row.scenario
            if s <= length(od_counts_by_scenario)
                q = get(od_counts_by_scenario[s], (row.origin_id, row.dest_id), 0)
            else
                q = 0
            end
        end
        total += get_routing_cost(data, row.pickup_id, row.dropoff_id) * row.value * q
    end
    return total
end
