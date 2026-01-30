using CSV
using DataFrames
using JSON

"""
    export_results(result::OptResult, output_dir::String; station_df=nothing)

Export optimization results to the specified directory.
Creates:
- stations.csv (optional): DataFrame with station selection data
- metadata.json: Metadata dictionary with optimization information
"""
function export_results(
        result::OptResult,
        output_dir::String;
        station_df::Union{DataFrame, Nothing}=nothing
    )
    # Create output directory
    mkpath(output_dir)

    # Export dataframe to CSV
    if station_df !== nothing
        stations_csv_path = joinpath(output_dir, "stations.csv")
        CSV.write(stations_csv_path, station_df)
        println("  ✓ Exported stations data: $stations_csv_path")
    end

    # Export metadata to JSON
    metadata_json_path = joinpath(output_dir, "metadata.json")
    open(metadata_json_path, "w") do io
        JSON.print(io, result.metadata, 4)  # Pretty print with 4-space indentation
    end

    println("  ✓ Exported metadata: $metadata_json_path")
end

export export_results
