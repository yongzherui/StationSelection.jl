using CSV
using JSON


"""
    export_results(results::VbsLocationResult.Result, output_dir::String)

Depracated function. Please do not use. It is not good
Exports the result to the specified directory.
Creates two files:
- stations.csv: DataFrame with station selection data
- metadata.json: Metadata dictionary with optimization information
"""

# Base.@deprecate export_results(result::Result, output_dir::String) nothing
 
function export_results(result::Result, output_dir::String)
    # Create output directory
    mkpath(output_dir)

    # Export dataframe to CSV
    if result.station_df !== nothing
        stations_csv_path = joinpath(output_dir, "stations.csv")
        CSV.write(stations_csv_path, result.station_df)
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
