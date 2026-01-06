module ClusteringTwoStageLRoutingTransportation

using JuMP
using Dates
using DataFrames
using Gurobi

using ..Results: Result

export clustering_two_stage_l_routing_transportation, validate_request_flow_mapping

"""
Count the number of pick-ups and drop-offs at each station for each scenario.
Returns two dictionaries: one for pick-ups and one for drop-offs.
"""
function counting_scenario_pickup_dropoff_counts(candidate_stations::DataFrame, scenario_requests::Dict{Int, DataFrame})
    scenario_pickup_counts = Dict{Int, Dict{Int, Int}}()
    scenario_dropoff_counts = Dict{Int, Dict{Int, Int}}()

    for (s, customer_requests) in scenario_requests
        # Count pick-ups (start_station_id)
        pickup_counts = Dict{Int, Int}()
        for station_id in customer_requests.start_station_id
            pickup_counts[station_id] = get(pickup_counts, station_id, 0) + 1
        end

        # Count drop-offs (end_station_id)
        dropoff_counts = Dict{Int, Int}()
        for station_id in customer_requests.end_station_id
            dropoff_counts[station_id] = get(dropoff_counts, station_id, 0) + 1
        end

        # Initialize all candidate stations to 0 if not present
        for station_id in candidate_stations.id
            if !haskey(pickup_counts, station_id)
                pickup_counts[station_id] = 0
            end
            if !haskey(dropoff_counts, station_id)
                dropoff_counts[station_id] = 0
            end
        end

        scenario_pickup_counts[s] = pickup_counts
        scenario_dropoff_counts[s] = dropoff_counts
    end

    return scenario_pickup_counts, scenario_dropoff_counts
end

"""
Clustering with two-stage optimization, incorporating routing costs via transportation problem.

This formulation:
- Separates pick-up and drop-off assignments
- Models routing cost using a transportation problem (supply/demand at stations)
- Avoids the cycle issue from min-cost network flow
- Uses pre-computed shortest paths for routing costs
- Differentiates between walking costs and routing costs

Parameters:
- candidate_stations: DataFrame with columns id, lon, lat
- k: Number of stations to activate per scenario
- customer_requests: DataFrame with columns start_station_id, end_station_id, request_time
- walking_costs: Dict{Tuple{Int,Int}, Float64} - walking distances between locations and stations
- routing_costs: Dict{Tuple{Int,Int}, Float64} - vehicle routing distances between stations
- scenarios: Vector of (start_time, end_time) tuples for splitting requests
- l: Number of permanent stations to build (default: midpoint between k and total stations)
- lambda: Weight for routing cost vs walking cost (default: 1.0)
- optimizer_env: Gurobi environment (optional)
"""
function clustering_two_stage_l_routing_transportation(
    candidate_stations::DataFrame,
    k::Int,
    customer_requests::DataFrame,
    walking_costs::Dict{Tuple{Int, Int}, Float64},
    routing_costs::Dict{Tuple{Int, Int}, Float64},
    scenarios::Vector{Tuple{String, String}} = Vector{Tuple{String, String}}();
    l::Int = nothing,
    lambda::Float64 = 1.0,
    optimizer_env=nothing)::Result

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    # Set default value for l
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
        for (i, scenario) in enumerate(scenarios)
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
        scenario_requests = Dict(1=> customer_requests)
        scenario_labels = Dict(1 => "one_scenario")
    end

    # Count pick-ups and drop-offs separately
    scenario_pickup_counts, scenario_dropoff_counts = counting_scenario_pickup_dropoff_counts(candidate_stations, scenario_requests)

    # Create mappings between station IDs and indices
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

    # Decision variables
    # x_pick[i, j, s]: location i assigned to pick-up station j in scenario s
    @variable(m, x_pick[1:n, 1:n, 1:S], Bin)
    # x_drop[i, j, s]: location i assigned to drop-off station j in scenario s
    @variable(m, x_drop[1:n, 1:n, 1:S], Bin)
    # f[j, k, s]: flow (number of passengers) from station j to station k in scenario s
    @variable(m, f[1:n, 1:n, 1:S] >= 0)
    # z[j, s]: station j is active in scenario s
    @variable(m, z[1:n, 1:S], Bin)
    # y[j]: station j is built (permanent)
    @variable(m, y[1:n], Bin)

    # Expressions for supply/demand at each station (computed from assignment variables)
    @expression(m, p[j=1:n, s=1:S], sum(scenario_pickup_counts[s][idx_to_id[i]] * x_pick[i,j,s] for i in 1:n))
    @expression(m, d[j=1:n, s=1:S], sum(scenario_dropoff_counts[s][idx_to_id[i]] * x_drop[i,j,s] for i in 1:n))

    # Objective: minimize walking distance (pick-up + drop-off) + lambda * routing cost
    @objective(m, Min,
        # Pick-up walking distance (customer origin to pick-up station)
        sum(scenario_pickup_counts[s][idx_to_id[i]] * walking_costs[(idx_to_id[i], idx_to_id[j])] * x_pick[i,j,s]
            for i in 1:n, j in 1:n, s in 1:S) +
        # Drop-off walking distance (drop-off station to customer destination)
        sum(scenario_dropoff_counts[s][idx_to_id[i]] * walking_costs[(idx_to_id[i], idx_to_id[j])] * x_drop[i,j,s]
            for i in 1:n, j in 1:n, s in 1:S) +
        # Routing cost (vehicle flow between stations)
        lambda * sum(routing_costs[(idx_to_id[j], idx_to_id[k])] * f[j,k,s]
            for j in 1:n, k in 1:n, s in 1:S)
    )

    # Constraint 1: Each location must be assigned to exactly one pick-up station
    @constraint(m, [i=1:n, s=1:S], sum(x_pick[i,j,s] for j in 1:n) == 1)

    # Constraint 2: Each location must be assigned to exactly one drop-off station
    @constraint(m, [i=1:n, s=1:S], sum(x_drop[i,j,s] for j in 1:n) == 1)

    # Constraint 3: Transportation constraint - outflow from j equals supply (pick-ups) at j
    @constraint(m, [j=1:n, s=1:S], sum(f[j,k,s] for k in 1:n) == p[j,s])

    # Constraint 4: Transportation constraint - inflow to k equals demand (drop-offs) at k
    @constraint(m, [k=1:n, s=1:S], sum(f[j,k,s] for j in 1:n) == d[k,s])

    # Constraint 5: Can only assign pick-ups to active stations
    @constraint(m, [i=1:n, j=1:n, s=1:S], x_pick[i,j,s] <= z[j,s])

    # Constraint 6: Can only assign drop-offs to active stations
    @constraint(m, [i=1:n, j=1:n, s=1:S], x_drop[i,j,s] <= z[j,s])

    # # Compute big-M for each scenario: total number of passengers in that scenario
    # M_s = Dict{Int, Float64}()
    # for s in 1:S
    #     total_passengers = sum(scenario_pickup_counts[s][idx_to_id[i]] for i in 1:n)
    #     M_s[s] = total_passengers
    # end

    # # Constraint 9: Flow can only originate from active stations
    # @constraint(m, [j=1:n, k=1:n, s=1:S], f[j,k,s] <= M_s[s] * z[j,s])

    # # Constraint 8: Flow can only go to active stations
    # @constraint(m, [j=1:n, k=1:n, s=1:S], f[j,k,s] <= M_s[s] * z[k,s])

    # Constraint 7: Active stations must be built
    @constraint(m, [j=1:n, s=1:S], z[j,s] <= y[j])

    # Constraint 8: Build exactly l permanent stations
    @constraint(m, sum(y[i] for i in 1:n) == l)

    # Constraint 9: Activate exactly k stations per scenario
    @constraint(m, [s=1:S], sum(z[j,s] for j in 1:n) == k)

    optimize!(m)

    # Prepare results DataFrame
    df = DataFrame(
        id=candidate_stations.id,
        lon=candidate_stations.lon,
        lat=candidate_stations.lat,
        selected=value.(y)
    )

    # Add z_{js} variables for analysis
    for i in 1:length(scenario_requests)
        df[!, scenario_labels[i]] = value.(z)[:, i]
    end

    return Result(
        "ClusteringTwoStageLRoutingTransportation",
        is_solved_and_feasible(m),
        Int.(value.(y)),
        Dict(idx_to_id[i] => (value.(y)[i] > 0.5) for i in 1:n),
        df,
        m
    )
end

"""
Validate that flow variables correctly account for all customer requests.

For each customer request (origin -> destination), this function:
1. Identifies the pickup station j where the customer is assigned
2. Identifies the dropoff station k where the customer is assigned
3. Verifies that the flow f[j,k,s] accounts for this passenger
4. Tracks flow usage to ensure no double-counting

Parameters:
- result: Result object from clustering_two_stage_l_routing_transportation
- scenario_requests: Dict mapping scenario index to DataFrame of customer requests
- candidate_stations: DataFrame with station information
- output_file: Optional CSV file to save detailed validation results

Returns:
- DataFrame with validation results for each request
"""
function validate_request_flow_mapping(
    result::Result,
    scenario_requests::Dict{Int, DataFrame},
    candidate_stations::DataFrame;
    output_file::Union{String, Nothing}=nothing,
    tolerance::Float64=1e-6)

    m = result.model

    # Create mappings
    id_to_idx = Dict{Int, Int}()
    idx_to_id = Dict{Int, Int}()
    for (idx, station_id) in enumerate(candidate_stations.id)
        id_to_idx[station_id] = idx
        idx_to_id[idx] = station_id
    end

    n = nrow(candidate_stations)
    S = length(scenario_requests)

    # Extract variable values
    x_pick_vals = value.(m[:x_pick])
    x_drop_vals = value.(m[:x_drop])
    f_vals = value.(m[:f])

    # Track flow usage: how much of each f[j,k,s] has been accounted for
    flow_remaining = copy(f_vals)

    println("="^70)
    println("REQUEST-BY-REQUEST FLOW VALIDATION")
    println("="^70)

    validation_data = []
    issues_found = false

    for (s, customer_requests) in sort(collect(scenario_requests))
        println("\n--- Scenario $s: $(nrow(customer_requests)) requests ---")

        for (req_idx, request) in enumerate(eachrow(customer_requests))
            origin_id = request.start_station_id
            dest_id = request.end_station_id

            origin_idx = id_to_idx[origin_id]
            dest_idx = id_to_idx[dest_id]

            # Find pickup station: which j has x_pick[origin_idx, j, s] = 1
            pickup_station_idx = findfirst(j -> x_pick_vals[origin_idx, j, s] > 0.5, 1:n)
            pickup_station_id = isnothing(pickup_station_idx) ? missing : idx_to_id[pickup_station_idx]

            # Find dropoff station: which k has x_drop[dest_idx, k, s] = 1
            dropoff_station_idx = findfirst(k -> x_drop_vals[dest_idx, k, s] > 0.5, 1:n)
            dropoff_station_id = isnothing(dropoff_station_idx) ? missing : idx_to_id[dropoff_station_idx]

            # Check if flow exists
            flow_value = missing
            flow_available = missing
            flow_issue = ""

            if !isnothing(pickup_station_idx) && !isnothing(dropoff_station_idx)
                flow_value = f_vals[pickup_station_idx, dropoff_station_idx, s]
                flow_available = flow_remaining[pickup_station_idx, dropoff_station_idx, s]

                # Check if there's enough flow
                if flow_available >= 1.0 - tolerance
                    # Account for this passenger in the flow
                    flow_remaining[pickup_station_idx, dropoff_station_idx, s] -= 1.0
                elseif flow_value >= 1.0 - tolerance
                    flow_issue = "Flow exists but already fully accounted for"
                    issues_found = true
                else
                    flow_issue = "Insufficient flow (need 1.0, have $(flow_value))"
                    issues_found = true
                end
            else
                flow_issue = "Missing assignment"
                if isnothing(pickup_station_idx)
                    flow_issue *= " (no pickup station)"
                end
                if isnothing(dropoff_station_idx)
                    flow_issue *= " (no dropoff station)"
                end
                issues_found = true
            end

            # Record validation data
            push!(validation_data, (
                scenario = s,
                request_idx = req_idx,
                origin_id = origin_id,
                destination_id = dest_id,
                pickup_station_id = pickup_station_id,
                dropoff_station_id = dropoff_station_id,
                flow_value = flow_value,
                flow_remaining_after = isnothing(pickup_station_idx) || isnothing(dropoff_station_idx) ? missing :
                                      flow_remaining[pickup_station_idx, dropoff_station_idx, s],
                issue = flow_issue,
                valid = flow_issue == ""
            ))

            # Print issues
            if flow_issue != ""
                println("  ❌ Request $req_idx: $(origin_id) → $(dest_id)")
                println("     Pickup: $(pickup_station_id), Dropoff: $(dropoff_station_id)")
                println("     Issue: $flow_issue")
            end
        end

        # Check for remaining unaccounted flows
        println("\n  Checking for unaccounted flows in scenario $s...")
        unaccounted_flows = 0
        for j in 1:n, k in 1:n
            remaining = flow_remaining[j, k, s]
            if remaining > tolerance
                println("    ⚠️  Unaccounted flow: f[$(idx_to_id[j]), $(idx_to_id[k]), $s] = $remaining")
                unaccounted_flows += 1
                issues_found = true
            end
        end

        if unaccounted_flows == 0
            println("  ✅ All flows accounted for in scenario $s")
        else
            println("  ❌ Found $unaccounted_flows unaccounted flows in scenario $s")
        end
    end

    println("\n" * "="^70)

    validation_df = DataFrame(validation_data)

    # Summary statistics
    total_requests = nrow(validation_df)
    valid_requests = sum(validation_df.valid)
    invalid_requests = total_requests - valid_requests

    println("\nVALIDATION SUMMARY:")
    println("  Total requests: $total_requests")
    println("  Valid: $valid_requests ($(round(100*valid_requests/total_requests, digits=2))%)")
    println("  Invalid: $invalid_requests ($(round(100*invalid_requests/total_requests, digits=2))%)")

    if !issues_found
        println("\n  ✅✅✅ ALL FLOW CONSTRAINTS VALIDATED SUCCESSFULLY! ✅✅✅")
    else
        println("\n  ❌ FLOW VALIDATION ISSUES DETECTED - SEE DETAILS ABOVE")
    end

    # Save to file if requested
    if !isnothing(output_file)
        CSV.write(output_file, validation_df)
        println("\nDetailed validation results saved to: $output_file")
    end

    return validation_df
end

end # module
