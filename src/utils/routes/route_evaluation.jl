export route_sequence_key
export route_has_repeated_stations
export compute_route_segment_costs
export compute_route_travel_time_from_segments
export compute_route_detour_feasible_legs
export evaluate_route_sequence

route_sequence_key(station_indices::AbstractVector{Int}) = Tuple(station_indices)

route_has_repeated_stations(station_indices::AbstractVector{Int}) =
    length(Set(station_indices)) != length(station_indices)

function compute_route_segment_costs(
    station_indices::AbstractVector{Int},
    data::StationSelectionData
)::Union{Vector{Float64}, Nothing}
    length(station_indices) >= 2 || return nothing
    seg = Vector{Float64}(undef, length(station_indices) - 1)
    for i in 1:length(seg)
        cost = get_routing_cost(data, station_indices[i], station_indices[i + 1])
        isfinite(cost) || return nothing
        seg[i] = cost
    end
    return seg
end

function compute_route_travel_time_from_segments(
    segment_costs::AbstractVector{<:Real};
    stop_dwell_time::Float64=10.0
)::Float64
    n_intermediate = max(length(segment_costs) - 1, 0)
    return sum(segment_costs; init=0.0) + n_intermediate * stop_dwell_time
end

function compute_route_detour_feasible_legs(
    station_indices::AbstractVector{Int},
    data::StationSelectionData;
    max_detour_time::Float64=Inf,
    max_detour_ratio::Float64=Inf,
    relevant_jk_pairs::Union{Nothing, Set{Tuple{Int, Int}}}=nothing,
    segment_costs::Union{Nothing, AbstractVector{<:Real}}=nothing,
)::Vector{Tuple{Int, Int}}
    seg = isnothing(segment_costs) ? compute_route_segment_costs(station_indices, data) : Float64.(segment_costs)
    isnothing(seg) && return Tuple{Int, Int}[]

    m = length(station_indices)
    feasible_legs = Tuple{Int, Int}[]
    for i in 1:m
        cum = 0.0
        for j in (i + 1):m
            cum += seg[j - 1]
            pair = (station_indices[i], station_indices[j])
            !isnothing(relevant_jk_pairs) && pair ∉ relevant_jk_pairs && continue
            direct = get_routing_cost(data, pair[1], pair[2])
            if (cum - direct <= max_detour_time) &&
               (direct == 0.0 || cum / direct <= 1.0 + max_detour_ratio)
                push!(feasible_legs, pair)
            end
        end
    end
    return feasible_legs
end

function evaluate_route_sequence(
    station_indices::AbstractVector{Int},
    data::StationSelectionData;
    route_id::Int=0,
    max_route_length::Int=typemax(Int),
    max_travel_time::Union{Nothing, Float64}=nothing,
    max_detour_time::Float64=Inf,
    max_detour_ratio::Float64=Inf,
    stop_dwell_time::Float64=10.0,
    relevant_jk_pairs::Union{Nothing, Set{Tuple{Int, Int}}}=nothing,
    min_relevant_feasible_legs::Int=1,
    allow_repeated_stations::Bool=false,
)::Union{RouteData, Nothing}
    length(station_indices) >= 2 || return nothing
    length(station_indices) <= max_route_length || return nothing
    !allow_repeated_stations && route_has_repeated_stations(station_indices) && return nothing

    seg = compute_route_segment_costs(station_indices, data)
    isnothing(seg) && return nothing

    travel_time = compute_route_travel_time_from_segments(seg; stop_dwell_time=stop_dwell_time)
    !isnothing(max_travel_time) && travel_time > max_travel_time && return nothing

    feasible_legs = compute_route_detour_feasible_legs(
        station_indices,
        data;
        max_detour_time=max_detour_time,
        max_detour_ratio=max_detour_ratio,
        relevant_jk_pairs=relevant_jk_pairs,
        segment_costs=seg,
    )
    length(feasible_legs) >= min_relevant_feasible_legs || return nothing

    return RouteData(route_id, collect(station_indices), travel_time, feasible_legs)
end
