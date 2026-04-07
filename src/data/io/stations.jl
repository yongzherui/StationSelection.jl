using CSV
using DataFrames

"""
    read_candidate_stations(file_path::String) -> DataFrame

Reads candidate stations from a CSV file and returns a DataFrame.

Expected CSV schema:
| Field Name    | Description        |
|---------------|--------------------|
| station_id    | Station ID         |
| station_name  | Station name       |
| station_lon   | Station longitude  |
| station_lat   | Station latitude   |

Applies coordinate transformation from BD-09LL to WGS84 and renames columns to:
- id, lon, lat
"""
function read_candidate_stations(file_path::String)::DataFrame
    df = CSV.File(file_path) |> DataFrame

    # Convert BD-09 → WGS-84
    wgs_coords = [bd09_to_wgs84(df.station_lon[i], df.station_lat[i])
                  for i in 1:nrow(df)]
    df.station_lon = first.(wgs_coords)
    df.station_lat = last.(wgs_coords)

    # Rename columns
    rename!(df, Dict(:station_id => :id, :station_lon => :lon, :station_lat => :lat))
    return df
end
