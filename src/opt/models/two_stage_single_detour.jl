struct TwoStageSingleDetourModel <: AbstractSingleDetourModel
    k::Int
    l::Int
    vehicle_routing_weight::Float64
    in_vehicle_time_weight::Float64

    # this is related to the formulation of the detour constraints
    time_window::Float64       # this decides how to divide and discretize the time of each order
    routing_delay::Float64     # this decides the maximum delay allowable by each single detour

    # walking distance constraint
    use_walking_distance_limit::Bool
    max_walking_distance::Union{Float64, Nothing}  # maximum walking distance from origin to pickup / dropoff to destination
    tight_constraints::Bool
    detour_use_flow_bounds::Bool

    function TwoStageSingleDetourModel(
            k::Int,
            l::Int,
            vehicle_routing_weight::Number,
            time_window::Number,
            routing_delay::Number;
            routing_weight::Union{Number, Nothing}=nothing,
            in_vehicle_time_weight::Union{Number, Nothing}=nothing,
            use_walking_distance_limit::Bool=false,
            max_walking_distance::Union{Number, Nothing}=nothing,
            tight_constraints::Bool=true,
            detour_use_flow_bounds::Bool=false
       )
        if !isnothing(routing_weight)
            vehicle_routing_weight = routing_weight
        end
        if isnothing(in_vehicle_time_weight)
            in_vehicle_time_weight = vehicle_routing_weight
        end

        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        vehicle_routing_weight >= 0 || throw(ArgumentError("vehicle_routing_weight must be non-negative"))
        in_vehicle_time_weight >= 0 || throw(ArgumentError("in_vehicle_time_weight must be non-negative"))
        time_window > 0 || throw(ArgumentError("time_window must be positive"))
        routing_delay >= 0 || throw(ArgumentError("routing_delay must be non-negative"))

        if !isnothing(max_walking_distance)
            use_walking_distance_limit = true
        end

        if use_walking_distance_limit
            isnothing(max_walking_distance) && throw(ArgumentError("max_walking_distance must be provided when walking distance limit is enabled"))
            max_walking_distance >= 0 || throw(ArgumentError("max_walking_distance must be non-negative"))

            new(k, l, Float64(vehicle_routing_weight), Float64(in_vehicle_time_weight),
                Float64(time_window), Float64(routing_delay),
                true, Float64(max_walking_distance), tight_constraints, detour_use_flow_bounds)
        else
            new(k, l, Float64(vehicle_routing_weight), Float64(in_vehicle_time_weight),
                Float64(time_window), Float64(routing_delay),
                false, nothing, tight_constraints, detour_use_flow_bounds)
        end
    end
end
