module ReadCustomerRequests

using DataFrames
using Dates
using CSV

export read_customer_requests

"""
Read customer requests from a file and return a DataFrame.

The schema for the input CSV file is as follows:
| Field Name                          | Description                                            |
|------------------------------------ |--------------------------------------------------------|
| order_id                            | Unique order ID                                        |
| region_id                           | Region ID                                              |
| pax_num                             | Number of passengers                                   |
| order_time                          | Order creation time                                    |
| available_pickup_station_list       | List of available pickup stations (1 station now)      |
| available_pickup_walkingtime_list   | Walking times to each pickup station (0 now)           |
| available_dropoff_station_list      | List of available drop-off stations (1 station now)    |
| available_dropoff_walkingtime_list  | Walking times to each drop-off station (0 now)         |
| status                              | Order status (0 new order; 1 accepted; 2 picked up; 3 finished; 4 canceled)                  |
| vehicle_id                          | Assigned vehicle ID                                    |
| pick_up_time                        | Actual pickup time                                     |
| drop_off_time                       | Actual drop-off time                                   |
| pick_up_early                       | Pickup earliness (1 if true)                           |
| drop_off_early                      | Drop-off earliness (1 if true)                         |

"""
function read_customer_requests(file_path::String)
    # Placeholder implementation
    df = CSV.File(file_path) |> DataFrame

    # we need to extract the available_pickup_station_list and available_dropoff_station_list
    # they are in the form of a string of a list, e.g. "[1, 2, 3]"
    # we will just take the first element of the list for now
    df.available_pickup_station_list = parse.(Int, replace.(df.available_pickup_station_list, r"[\[\]]" => ""))
    df.available_dropoff_station_list = parse.(Int, replace.(df.available_dropoff_station_list, r"[\[\]]" => ""))

    # We only take the first element for now
    df.start_station_id = first.(df.available_pickup_station_list)
    df.end_station_id = first.(df.available_dropoff_station_list)
    # We only need the following columns: id, start_station_id, end_station_id, request_time
    df = select(df, [:order_id, :start_station_id, :end_station_id, :order_time])
    # Rename columns to match expected output
    rename!(df, Dict(:order_id => :id, :order_time => :request_time))

    df.request_time = DateTime.(df.request_time, "yyyy-mm-dd HH:MM:SS")

    return df
end

end # module