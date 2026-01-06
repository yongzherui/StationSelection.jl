
module ClusteringTwoStageL

using JuMP
using Dates
using DataFrames
using Gurobi

using ..Results: Result

export clustering_two_stage_l

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

function clustering_two_stage_l(
    candidate_stations::DataFrame,
    k::Int,
    customer_requests::DataFrame,
    costs::Dict{Tuple{Int, Int}, Float64},
    scenarios::Vector{Tuple{String, String}} = Vector{Tuple{String, String}}();
    l::Int = nothing,
    optimizer_env=nothing)::Result

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    # if the value is not set
    # take a value between k and the number of candidate_stations
    if isnothing(l)
        l = div((k + nrow(candidate_stations)), 2)
    end

    if l < k
        error("l cannot be less than k")
    end

    if l > nrow(candidate_stations)
        error("l cannot be greater than the number of candidate stations")
    end

    # we split the customer requests into scenarios if scenarios is not empty
    if length(scenarios) > 0
        scenario_requests = Dict{Int, DataFrame}()
        scenario_labels = Dict{Int, String}()
        idx = 1
        for (i, scenario) in enumerate(scenarios)
            start_date = DateTime(scenario[1], "yyyy-mm-dd HH:MM:SS")
            end_date = DateTime(scenario[2], "yyyy-mm-dd HH:MM:SS")
            i_requests = customer_requests[(customer_requests.request_time .>= start_date) .& (customer_requests.request_time .<= end_date), :]
            # we need to check if there are any customer_requests in this time period, otherwise we skip it
            if size(i_requests, 1) > 0
                scenario_requests[idx] = i_requests
                scenario_labels[idx] = "$(scenario[1])_$(scenario[2])"
                idx += 1
            end
        end
    else
        scenario_requests = Dict(1=> customer_requests)
        scenario_labels = Dict(1 => "one_scenario")
    end

    scenario_request_counts = counting_scenario_request_counts(candidate_stations, scenario_requests)
    # we now have a dictionary of scenario index to request counts

    # create a mapping of station_id to index in candidate_stations
    id_to_idx = Dict{Int, Int}()
    idx_to_id = Dict{Int, Int}()
    for (idx, station_id) in enumerate(candidate_stations.id)
        id_to_idx[station_id] = idx
        idx_to_id[idx] = station_id
    end
    n = nrow(candidate_stations)
    S = length(scenario_requests)

    # we now set up the optimization model
    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    set_silent(m)
    # x[i, j, s] is interpreted as request i is assigned to medoid j in scenario s
    @variable(m, x[1:n, 1:n, 1:S], Bin)
    # z[i, s] is interpreted as station i is selected in scenario s
    @variable(m, z[1:n, 1:S], Bin)
    # y[j] is interpreted as candidate station j is selected as a medoid
    @variable(m, y[1:n], Bin)

    # The objective is to minimize the total cost of serving all requests
    @objective(m, Min, sum(scenario_request_counts[s][idx_to_id[i]] * costs[(idx_to_id[i], idx_to_id[j])] * x[i,j,s] for i in 1:n, j in 1:n, s in 1:S))

    # This constraint ensures that all requests in i are assigned to exactly one medoid in j
    @constraint(m, [i=1:n, s in 1:S], sum(x[i,j,s] for j in 1:n) == 1)
    # This constraint ensures that a request can only be assigned to a medoid if that medoid is selected
    @constraint(m, [i=1:n, j=1:n, s=1:S], x[i,j,s] <= z[j,s])
    @constraint(m, [j=1:n, s=1:S], z[j,s] <= y[j])
    # This constraint ensures that we select exactly l medoids
    @constraint(m, sum(y[i] for i in 1:n) == l)
    # This constraint ensures that we select exactly k stations in each scenario
    # this makes it comparable to the single stage k-medoids
    @constraint(m, [s=1:S], sum(z[j,s] for j in 1:n) == k)

    optimize!(m)

    # we want to return the selected stations
    # combine it with the fact that candidate_stations is a dataframe
    # we return a new dataframe with the value of y indicating if the station is selected or not
    # we map the value of y back to the station_id using idx_to_id
    df = DataFrame(id=candidate_stations.id, lon=candidate_stations.lon, lat=candidate_stations.lat, selected=value.(y))

    # we need to add the z_{js} variables for analysis
    for i in 1:length(scenario_requests)
        df[!, scenario_labels[i]] = value.(z)[:, i]
    end

    # we want to return a struct that is the result of the optimization
    # it will include the optimization status, value as well as the data frame
    return Result(
        "ClusteringTwoStageL",
        is_solved_and_feasible(m),
        Int.(value.(y)),
        Dict(idx_to_id[i] => (value.(y)[i] > 0.5) for i in 1:n),
        df,
        m
    )
end

end # module
