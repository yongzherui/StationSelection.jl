module ClusteringIdeal

using JuMP
using Dates
using DataFrames
using Gurobi

using ..ClusteringBase: clustering_base
using ..Results: Result

export clustering_ideal

function counting_scenario_request_counts(candidate_stations::DataFrame, scenario_requests::Dict{Int, DataFrame})
    # we combine the customer_requests into one big dataframe since it is a list of dataframe
    scenario_request_counts = Dict{Int, Dict{Int, Int}}()
    for (s, customer_requests) in scenario_requests
        requests = vcat(customer_requests.start_station_id, customer_requests.end_station_id)

        # we create a dictionary that is a counter of the number of times a station_id is requested for pick up or drop off
        request_counts = Dict{Int, Int}()
        for station_id in requests
            request_counts[station_id] = get(request_counts, station_id, 0) + 1
        end

        for station_id in candidate_stations.id
            if !haskey(request_counts, station_id)
                request_counts[station_id] = 0
            end
        end

        scenario_request_counts[s] = request_counts
    end

    return scenario_request_counts
end

function clustering_ideal(
    candidate_stations::DataFrame, 
    k::Int, 
    customer_requests::DataFrame, 
    costs::Dict{Tuple{Int, Int}, Float64}, 
    scenarios::Vector{Tuple{String, String}} = Vector{Tuple{String, String}}();
    strict_equality=true,
    optimizer_env=nothing)::Result

    if optimizer_env == nothing
        optimizer_env = Gurobi.Env()
    end

    # we stat an empty df
    # difference is there is no 'selected' column
    df = DataFrame(id=candidate_stations.id, lon=candidate_stations.lon, lat=candidate_stations.lat)
    status = true
    # we split the customer requests into scenarios if scenarios is not empty
    if length(scenarios) > 0
        scenario_requests = Dict{Int, DataFrame}()
        scenario_labels = Dict{Int, String}()
        idx = 1
        for (i, scenario) in enumerate(scenarios)
            start_date = DateTime(scenario[1], "yyyy-mm-dd HH:MM:SS")
            end_date = DateTime(scenario[2], "yyyy-mm-dd HH:MM:SS")
            i_requests = customer_requests[(customer_requests.request_time .>= start_date) .& (customer_requests.request_time .<= end_date), :]
            if size(i_requests, 1) == 0
                continue
            end
            scenario_requests[idx] = i_requests
            scenario_labels[idx] = "$(scenario[1])_$(scenario[2])"
            idx += 1
        end

    else
        # we just say that there is one index
        scenario_requests = Dict(1=> customer_requests)
        scenario_labels = Dict(1 => "one_scenario")
    end

    # we run ClusteringBase here for each one
    for (i, request) in scenario_requests

        clustering_base_result = clustering_base(candidate_stations, k, request, costs; strict_equality=strict_equality, optimizer_env=optimizer_env)
        # we only want to take the selected one and put that in as the solution
        # TODO the order is probably maintained so i think taking the value vector directly should be correct
        result_df = clustering_base_result.station_df
        # drop the lat lon columns
        result_df = select(result_df, Not(:lat, :lon))
        # rename the column appropriately
        rename!(result_df, :selected => scenario_labels[i])

        df = leftjoin(df, result_df, on=:id)

        status = status && clustering_base_result.status

        # if for some reason clustering fails
        if status == false
            return Result(
                "ClusteringIdeal",
                status,
                nothing,
                nothing,
                df,
                nothing
            )
        end
    end

    # we want to create the selected column so we take the row wise ANY operation over all the columns except the first 3
    df.selected = reduce((a, b) -> Int.(Bool.(a) .|| Bool.(b)), eachcol(df[:, Not([:id, :lon, :lat])]))

    # we want to rearrange it so that selected is the 4th column
    select!(df, :id, :lon, :lat, :selected, :)


    return Result(
        "ClusteringIdeal",
        status,
        df.selected,
        nothing,
        df,
        nothing
    )
end

end # module