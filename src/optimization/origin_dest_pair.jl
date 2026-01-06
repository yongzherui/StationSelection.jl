module ClusteringTwoStageLOriginDestPair

using JuMP
using Dates
using DataFrames
using Gurobi

using ..Results: Result

export clustering_two_stage_l_od_pair

"""
    clustering_two_stage_l_od_pair(
        candidate_stations::DataFrame,
        k::Int,
        customer_requests::DataFrame,
        origin_costs::Dict{Tuple{Int, Int}, Float64},
        dest_costs::Dict{Tuple{Int, Int}, Float64},
        routing_costs::Dict{Tuple{Int, Int}, Float64},
        scenarios::Vector{Tuple{String, String}} = Vector{Tuple{String, String}}();
        l::Int = nothing,
        lambda::Float64 = 1.0,
        optimizer_env=nothing)::Result

Solves the two-stage location problem with origin-destination pairs.

# Arguments
- `candidate_stations::DataFrame`: DataFrame with columns id, lon, lat
- `k::Int`: Number of active stations per scenario
- `customer_requests::DataFrame`: DataFrame with columns start_station_id, end_station_id, request_time
- `origin_costs::Dict{Tuple{Int, Int}, Float64}`: Walking costs from origin o to station j
- `dest_costs::Dict{Tuple{Int, Int}, Float64}`: Walking costs from station k to destination d
- `routing_costs::Dict{Tuple{Int, Int}, Float64}`: Vehicle routing costs between stations j and k
- `scenarios::Vector{Tuple{String, String}}`: Time periods defining scenarios
- `l::Int`: Number of stations to build (first stage)
- `lambda::Float64`: Weight for routing costs
- `optimizer_env`: Gurobi environment (optional)

# Returns
- `Result`: Optimization result containing selected stations and scenario assignments
"""
function clustering_two_stage_l_od_pair(
    candidate_stations::DataFrame,
    k::Int,
    customer_requests::DataFrame,
    origin_costs::Dict{Tuple{Int, Int}, Float64},
    dest_costs::Dict{Tuple{Int, Int}, Float64},
    routing_costs::Dict{Tuple{Int, Int}, Float64},
    scenarios::Vector{Tuple{String, String}} = Vector{Tuple{String, String}}();
    l::Int = nothing,
    lambda::Float64 = 1.0,
    optimizer_env=nothing)::Result

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    # if the value is not set, take a value between k and the number of candidate_stations
    if isnothing(l)
        l = div((k + nrow(candidate_stations)), 2)
    end

    if l < k
        error("l cannot be less than k")
    end

    if l > nrow(candidate_stations)
        error("l cannot be greater than the number of candidate stations")
    end

    # Split customer requests into scenarios
    if length(scenarios) > 0
        scenario_requests = Dict{Int, DataFrame}()
        scenario_labels = Dict{Int, String}()
        idx = 1
        for scenario in scenarios
            start_date = DateTime(scenario[1], "yyyy-mm-dd HH:MM:SS")
            end_date = DateTime(scenario[2], "yyyy-mm-dd HH:MM:SS")
            i_requests = customer_requests[(customer_requests.request_time .>= start_date) .& (customer_requests.request_time .<= end_date), :]
            if size(i_requests, 1) > 0
                scenario_requests[idx] = i_requests
                scenario_labels[idx] = "$(scenario[1])_$(scenario[2])"
                idx += 1
            end
        end
    else
        scenario_requests = Dict(1 => customer_requests)
        scenario_labels = Dict(1 => "one_scenario")
    end

    # Build scenario-specific OD pair demand structure
    # Omega_s[s] is the set of OD pairs with positive demand in scenario s
    # q_ods[s][(o, d)] is the demand from origin o to destination d in scenario s
    Omega_s = Dict{Int, Vector{Tuple{Int, Int}}}()
    q_ods = Dict{Int, Dict{Tuple{Int, Int}, Int}}()

    for (s, requests) in scenario_requests
        od_demand = Dict{Tuple{Int, Int}, Int}()
        for row in eachrow(requests)
            o = row.start_station_id
            d = row.end_station_id
            od_pair = (o, d)
            od_demand[od_pair] = get(od_demand, od_pair, 0) + 1
        end
        # Store only OD pairs with positive demand for this scenario
        Omega_s[s] = collect(keys(od_demand))
        q_ods[s] = od_demand
    end

    # Create station ID mappings
    id_to_idx = Dict{Int, Int}()
    idx_to_id = Dict{Int, Int}()
    for (idx, station_id) in enumerate(candidate_stations.id)
        id_to_idx[station_id] = idx
        idx_to_id[idx] = station_id
    end
    n = nrow(candidate_stations)
    S = length(scenario_requests)

    # Set up optimization model
    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    set_silent(m)

    # Decision variables with scenario-specific OD pairs
    # x[s][od_idx] is a matrix of size n x n representing station pair assignment
    # x[s][od_idx][j, k]: OD pair od_idx in scenario s uses pick-up station j and drop-off station k
    x = [Dict{Int, Matrix{VariableRef}}() for _ in 1:S]

    for s in 1:S
        num_od_pairs = length(Omega_s[s])
        for od_idx in 1:num_od_pairs
            x[s][od_idx] = @variable(m, [1:n, 1:n], Bin)
        end
    end

    # z[j, s]: station j is active in scenario s
    @variable(m, z[1:n, 1:S], Bin)
    # y[j]: station j is built
    @variable(m, y[1:n], Bin)

    # make sure all the origin, dest and routing costs are valid and not Inf
    for j in 1:n, k in 1:n, s in 1:S, (od_idx, (o, d)) in enumerate(Omega_s[s])
        if get(origin_costs, (o, idx_to_id[j]), Inf) === Inf
            error("origin costs is infinity")
        end

        if get(dest_costs, (d, idx_to_id[k]), Inf) === Inf
            error("dest cost is infinity")
        end

        if get(routing_costs, (idx_to_id[j], idx_to_id[k]), Inf) === Inf
            error("routing costs is infinity")
        end
    end

    # Objective: minimize total walking + routing costs
    @objective(m, Min,
        sum(
            q_ods[s][(o, d)] * (
                get(origin_costs, (o, idx_to_id[j]), Inf) +
                get(dest_costs, (d, idx_to_id[k]), Inf) +
                lambda * get(routing_costs, (idx_to_id[j], idx_to_id[k]), Inf)
            ) * x[s][od_idx][j, k]
            for s in 1:S
            for (od_idx, (o, d)) in enumerate(Omega_s[s])
            for j in 1:n
            for k in 1:n
        )
    )

    # Constraints for OD pair assignments and station activation
    for s in 1:S
        for od_idx in 1:length(Omega_s[s])
            # Each OD pair must be assigned to exactly one station pair
            @constraint(m, sum(x[s][od_idx][j, k] for j in 1:n, k in 1:n) == 1)

            # Can only use active stations for pick-up and drop-off
            for j in 1:n, k in 1:n
                @constraint(m, 2 * x[s][od_idx][j, k] <= z[j, s] + z[k, s])  # Both Pick-up and Drop-off station must be active
            end
        end
    end

    # Constraint: active stations must be built
    @constraint(m, [j=1:n, s=1:S],
        z[j, s] <= y[j]
    )

    # Constraint: build exactly l stations
    @constraint(m, sum(y[j] for j in 1:n) == l)

    # Constraint: activate exactly k stations per scenario
    @constraint(m, [s=1:S],
        sum(z[j, s] for j in 1:n) == k
    )

    println("Total Number of Variables $(num_variables(m))")

    # Track optimization start time
    start_time = time()
    try
        optimize!(m)
    catch e
        if isa(e, LoadError) && occursin("OutOfMemoryError()", e.msg)
            error_type = "out_of_memory"
        else
            error_type = "unknown"
        end

        metadata = Dict{String, Any}(
            "error_message" => e.msg,
            "error_type" => erorr_type
        )
        return Result(
            "ClusteringTwoStageLOriginDestPair",
            0,
            zeros(n),
            Dict(idx_to_id[i] => -1 for i in 1:n),
            nothing,
            nothing,
            metadata
        )
    end
    end_time = time()

    if !is_solved_and_feasible(m)
        # we want to check what status the model is in
        metadata = Dict{String, Any}(
            "solver_status" => termination_status(m)
        )

        return Result(
            "ClusteringTwoStageLOriginDestPair",
            0,
            zeros(n),
            Dict(idx_to_id[i] => -1 for i in 1:n),
            nothing,
            nothing,
            metadata
        )
        
    end

    # Calculate optimization runtime
    runtime_seconds = end_time - start_time

    # Calculate total cardinality of Omega_s across all scenarios
    total_od_pairs = sum(length(Omega_s[s]) for s in 1:S)

    # Calculate total number of variables
    total_variables = num_variables(m)

    # Build result dataframe
    df = DataFrame(
        id=candidate_stations.id,
        lon=candidate_stations.lon,
        lat=candidate_stations.lat,
        selected=value.(y)
    )

    # Add scenario-specific activations
    for s in 1:S
        df[!, scenario_labels[s]] = value.(z)[:, s]
    end

    # Prepare metadata
    metadata = Dict{String, Any}(
        "optimization_runtime_seconds" => runtime_seconds,
        "total_od_pairs_across_scenarios" => total_od_pairs,
        "num_scenarios" => S,
        "total_variables" => total_variables,
        "num_candidate_stations" => n,
        "k_stations_per_scenario" => k,
        "l_total_stations" => l,
        "lambda_routing_weight" => lambda
    )

    return Result(
        "ClusteringTwoStageLOriginDestPair",
        is_solved_and_feasible(m),
        Int.(value.(y)),
        Dict(idx_to_id[i] => (value.(y)[i] > 0.5) for i in 1:n),
        df,
        m,
        metadata
    )
end

end # module
