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
DataFrame with columns: id, origin_station_id, destination_station_id, start_station_id,
end_station_id, request_time. The start/end columns are compatibility aliases for
the optimization data contract.

# Input CSV Schema
| Field Name                          | Description                                            |
|------------------------------------ |--------------------------------------------------------|
| order_id                            | Unique order ID                                        |
| order_time                          | Order creation time                                    |
| origin_station_id                   | Origin station ID                                      |
| destination_station_id              | Destination station ID                                 |

Legacy CSVs with `available_pickup_station_list` and
`available_dropoff_station_list` are still accepted as a fallback.
"""
function read_customer_requests(
    file_path::String;
    start_time::Union{DateTime, String, Nothing}=nothing,
    end_time::Union{DateTime, String, Nothing}=nothing
)
    df = CSV.File(file_path) |> DataFrame

    df.origin_station_id = _request_station_id.(eachrow(df), Ref(:origin))
    df.destination_station_id = _request_station_id.(eachrow(df), Ref(:destination))

    df.start_station_id = df.origin_station_id
    df.end_station_id = df.destination_station_id

    # Select and rename columns
    df = select(df, [:order_id, :origin_station_id, :destination_station_id, :start_station_id, :end_station_id, :order_time])
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

function _request_station_id(row, side::Symbol)::Int
    columns = propertynames(row)
    candidates = side == :origin ?
        (:origin_station_id, :start_station_id, :origin_id) :
        (:destination_station_id, :end_station_id, :target_id, :dest_station_id)

    for col in candidates
        if col in columns && !ismissing(row[col])
            station_id = Int(row[col])
            _warn_if_request_legacy_station_disagrees(row, side, station_id, col)
            return station_id
        end
    end

    legacy_col = side == :origin ? :available_pickup_station_list : :available_dropoff_station_list
    legacy_col in columns || error("request CSV missing $(side) station column")
    values = parse_station_list(string(row[legacy_col]))
    isempty(values) && error("request $(side) station column is empty")
    return first(values)
end

function _warn_if_request_legacy_station_disagrees(row, side::Symbol, station_id::Int, scalar_col::Symbol)
    legacy_col = side == :origin ? :available_pickup_station_list : :available_dropoff_station_list
    legacy_col in propertynames(row) || return
    ismissing(row[legacy_col]) && return

    values = parse_station_list(string(row[legacy_col]))
    isempty(values) && return
    legacy_id = first(values)
    if legacy_id != station_id
        @warn "Scalar station column disagrees with legacy station list; using scalar column" side scalar_col station_id legacy_col legacy_id
    end
end

function parse_station_list(list_str::AbstractString)::Vector{Int}
    cleaned = strip(replace(list_str, "[" => "", "]" => "", "," => " "))
    isempty(cleaned) && return Int[]
    return parse.(Int, split(cleaned))
end
