using DataFrames
using Dates
using CSV

"""
    read_customer_requests(file_path::String; start_time=nothing, end_time=nothing) -> DataFrame

Read customer requests from a CSV file and return a DataFrame.

# Arguments
- `file_path::String`: Path to the CSV file
- `start_time::Union{DateTime, String, Nothing}`: Filter requests after this time
- `end_time::Union{DateTime, String, Nothing}`: Filter requests before this time

# Returns
DataFrame with columns: id, start_station_id, end_station_id, request_time

# Input CSV Schema
| Field Name                          | Description                                            |
|------------------------------------ |--------------------------------------------------------|
| order_id                            | Unique order ID                                        |
| order_time                          | Order creation time                                    |
| available_pickup_station_list       | List of available pickup stations (e.g., "[136]")      |
| available_dropoff_station_list      | List of available drop-off stations (e.g., "[22]")     |
"""
function read_customer_requests(
    file_path::String;
    start_time::Union{DateTime, String, Nothing}=nothing,
    end_time::Union{DateTime, String, Nothing}=nothing
)
    df = CSV.File(file_path) |> DataFrame

    # Parse station lists - they are in the form "[1]" or "[1, 2, 3]"
    # We take the first element of each list
    df.available_pickup_station_list = parse.(Int, replace.(string.(df.available_pickup_station_list), r"[\[\]]" => ""))
    df.available_dropoff_station_list = parse.(Int, replace.(string.(df.available_dropoff_station_list), r"[\[\]]" => ""))

    df.start_station_id = df.available_pickup_station_list
    df.end_station_id = df.available_dropoff_station_list

    # Select and rename columns
    df = select(df, [:order_id, :start_station_id, :end_station_id, :order_time])
    rename!(df, Dict(:order_id => :id, :order_time => :request_time))

    # Parse request times
    df.request_time = DateTime.(string.(df.request_time), "yyyy-mm-dd HH:MM:SS")

    # Filter by time range if specified
    if !isnothing(start_time)
        start_dt = start_time isa String ? DateTime(start_time, "yyyy-mm-dd HH:MM:SS") : start_time
        df = df[df.request_time .>= start_dt, :]
    end

    if !isnothing(end_time)
        end_dt = end_time isa String ? DateTime(end_time, "yyyy-mm-dd HH:MM:SS") : end_time
        df = df[df.request_time .<= end_dt, :]
    end

    return df
end
