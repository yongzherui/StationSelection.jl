struct TwoStageSingleDetourModel <: AbstractTwoStageModel
    k::Int
    l::Int
    routing_weight::Float64

    # this is related to the formulation of the detour constraints
    time_window::Float64       # this decides how to divide and discretize the time of each order
    routing_delay::Float64     # this decides the maximum delay allowable by each single detour

    function TwoStageSingleDetourModel(
            k::Int,
            l::Int,
            routing_weight::Number,
            time_window::Number,
            routing_delay::Number
       )
        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        routing_weight >= 0 || throw(ArgumentError("routing_weight must be non-negative"))
        time_window > 0 || throw(ArgumentError("time_window must be positive"))
        routing_delay >= 0 || throw(ArgumentError("routing_delay must be non-negative"))

        new(k, l, Float64(routing_weight), Float64(time_window), Float64(routing_delay))
    end
end
