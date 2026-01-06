# This is meant to implement k-medoids clustering in JuMP

module ClusteringBase

using JuMP
using DataFrames
using Dates
using Gurobi

using ..Results: Result

export clustering_base

function preparing_data_for_clustering_base(candidate_stations::DataFrame, customer_requests::DataFrame)
    # we combine the customer_requests into one big dataframe since it is a list of dataframe

    # we now consider the cost of walking from to a pick up station and from a drop-off station
    # we will have to make the variable x into double the length of combined_requests and only extract the start_station_id and end_station_id
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

    return request_counts
end

function clustering_base(
    candidate_stations::DataFrame, 
    k::Int, 
    customer_requests::DataFrame, 
    costs::Dict{Tuple{Int, Int}, Float64}; 
    filter::Vector{Tuple{String, String}}=Vector{Tuple{String,String}}(),
    strict_equality::Bool=true,
    optimizer_env=nothing)::Result

    if optimizer_env == nothing
        optimizer_env = Gurobi.Env()
    end

    if length(filter) > 0
        filtered_df = Vector{DataFrame}()
        for scenario in filter
            start_time = DateTime(scenario[1], "yyyy-mm-dd HH:MM:SS")
            end_time = DateTime(scenario[2], "yyyy-mm-dd HH:MM:SS")
            push!(filtered_df, customer_requests[(customer_requests.request_time .>= start_time) .& (customer_requests.request_time .<= end_time), :])
        end
        customer_requests = vcat(filtered_df...)
    end

    request_counts = preparing_data_for_clustering_base(candidate_stations, customer_requests)

    # create a mapping of station_id to index in candidate_stations
    id_to_idx = Dict{Int, Int}()
    idx_to_id = Dict{Int, Int}()
    for (idx, station_id) in enumerate(candidate_stations.id)
        id_to_idx[station_id] = idx
        idx_to_id[idx] = station_id
    end
    n = nrow(candidate_stations)

    # we now set up the optimization model
    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    set_silent(m)
    # x[i, j] is interpreted as request i is assigned to medoid j
    @variable(m, x[1:n, 1:n], Bin)
    # y[j] is interpreted as candidate station j is selected as a medoid
    @variable(m, y[1:n], Bin)

    # The objective is to minimize the total cost of serving all requests
    @objective(m, Min, sum(request_counts[idx_to_id[i]] * costs[(idx_to_id[i], idx_to_id[j])] * x[i,j] for i in 1:n, j in 1:n))

    # This constraint ensures that all requests in i are assigned to exactly one medoid in j
    @constraint(m, [i=1:n], sum(x[i,j] for j in 1:n) == 1)
    # This constraint ensures that a request can only be assigned to a medoid if that medoid is selected
    @constraint(m, [i=1:n, j=1:n], x[i,j] <= y[j])
    # This constraint ensures that we select at most k medoids
    @constraint(m, sum(y[i] for i in 1:n) == k)

    optimize!(m)

    # we want to return the selected stations
    # combine it with the fact that candidate_stations is a dataframe
    # we return a new dataframe with the value of y indicating if the station is selected or not
    # we map the value of y back to the station_id using idx_to_id
    df = DataFrame(id=candidate_stations.id, lon=candidate_stations.lon, lat=candidate_stations.lat, selected=value.(y))

    # we want to return a struct that is the result of the optimization
    # it will include the optimization status, value as well as the data frame
    return Result(
        "ClusteringBase",
        is_solved_and_feasible(m),
        Int.(value.(y)),
        Dict(idx_to_id[i] => (value.(y)[i] > 0.5) for i in 1:n),
        df,
        m
    )
end

end # module