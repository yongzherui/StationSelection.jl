"""
Station and vehicle transformation utilities for converting selection results to simulation inputs.

This module provides functionality to prepare station and vehicle CSVs for simulation
from station selection results.
"""

using CSV
using DataFrames

export prepare_station_data, prepare_vehicle_data

"""
    prepare_station_data(base_station_file::String,
                        selection_result_file::String) -> DataFrame

Prepare station data for simulation by filtering to candidate stations and merging base data.

Reads the base station CSV and the station selection results, filters to stations present
in the selection results, joins them, and creates a combined DataFrame with base station
columns plus 'selected' and 'is_station' columns.

# Arguments
- `base_station_file`: Path to base station CSV with all station attributes
- `selection_result_file`: Path to selection results CSV with columns (id, lon, lat, selected, ...)

# Returns
- DataFrame containing only candidate stations (from selection results), with columns
  from base_station plus:
  - `selected`: 1.0 if selected, 0.0 otherwise
  - `is_station`: Boolean, true if selected

# Example
```julia
station_df = prepare_station_data(
    "Data/station.csv",
    "results/run_123/result/stations.csv"
)

# Write to file
CSV.write("scenario_data/station.csv", station_df)
```
"""
function prepare_station_data(base_station_file::String,
                             selection_result_file::String)
    # Read base station data
    base_df = CSV.read(base_station_file, DataFrame)

    # Read selection results
    selection_df = CSV.read(selection_result_file, DataFrame)

    # Verify we have selected stations
    if !any(selection_df.selected .== 1.0)
        error("No selected stations found in $selection_result_file")
    end

    # Join base_df with selection_df based on station_id and id
    # Keep only stations present in selection results (candidate set)
    station_df = innerjoin(base_df, selection_df[:, [:id, :selected]],
                           on = :station_id => :id)

    # Create is_station column from selected
    station_df.is_station = station_df.selected .== 1.0

    return station_df
end

"""
    prepare_vehicle_data(base_vehicle_file::String,
                        selected_station_ids::Vector{Int}) -> DataFrame

Prepare vehicle data for simulation by ensuring all vehicles start at selected stations.

Reads the base vehicle CSV and updates starting_station_id to ensure every vehicle
starts at a selected station. If a vehicle's original starting station is not selected,
it's reassigned to the first selected station.

# Arguments
- `base_vehicle_file`: Path to base vehicle CSV with columns including starting_station_id
- `selected_station_ids`: Vector of selected station IDs

# Returns
- DataFrame with updated starting_station_id column

# Example
```julia
# Get selected station IDs from prepared station data
selected_ids = station_df[station_df.is_station, :station_id]

vehicle_df = prepare_vehicle_data(
    "Data/vehicle.csv",
    selected_ids
)

# Write to file
CSV.write("scenario_data/vehicle.csv", vehicle_df)
```
"""
function prepare_vehicle_data(base_vehicle_file::String,
                             selected_station_ids::Vector{Int})
    # Read base vehicle data
    vehicle_df = CSV.read(base_vehicle_file, DataFrame)

    if isempty(selected_station_ids)
        error("No selected station IDs provided")
    end

    # Function to find nearest selected station (simplified - just keeps if selected, else uses first)
    function assign_to_selected_station(original_station_id, selected_ids)
        # If the original station is selected, keep it
        if original_station_id in selected_ids
            return original_station_id
        end
        # Otherwise, use the first selected station (simple fallback)
        # In a more sophisticated version, this could use distance-based assignment
        return selected_ids[1]
    end

    # Update starting_station_id for each vehicle
    vehicle_df.starting_station_id = [
        assign_to_selected_station(sid, selected_station_ids)
        for sid in vehicle_df.starting_station_id
    ]

    return vehicle_df
end
