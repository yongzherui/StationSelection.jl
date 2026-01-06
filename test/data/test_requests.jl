using StationSelection
# Tests the ReadCustomerRequests function

using DataFrames, Dates
@testset "ReadCustomerRequests" begin
    @testset "Full Schema" begin
        using .ReadCustomerRequests: read_customer_requests

        # Create a temporary CSV file for testing
        test_csv = tempname() * ".csv"
        open(test_csv, "w") do io
            write(io, "order_id,region_id,pax_num,order_time,available_pickup_station_list,available_pickup_walkingtime_list,available_dropoff_station_list,available_dropoff_walkingtime_list,status,vehicle_id,pick_up_time,drop_off_time,pick_up_early,drop_off_early\n")
            write(io, "23381,1,1,2025-01-15 10:42:33,[136],[0],[22],[0],4,,,,,\n")
            write(io, "23405,1,1,2025-02-06 10:17:40,[92],[0],[149],[0],4,,,,,\n")
            write(io, "23406,1,1,2025-02-06 10:40:45,[149],[0],[18],[0],4,,,,,\n")
        end

        # Read the customer requests from the CSV file
        df = read_customer_requests(test_csv)

        # Perform your tests on the DataFrame `df`
        @test size(df, 1) == 3
        @test df.id[1] == 23381
        @test df.start_station_id[1] == 136
        @test df.end_station_id[1] == 22
        @test df.request_time[1] == DateTime("2025-01-15 10:42:33", "yyyy-mm-dd HH:MM:SS")
    end
end