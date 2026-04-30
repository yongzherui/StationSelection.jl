"""
Demand-bound calibration utilities for the robust total-demand-cap model.

Computes per-OD lower bounds, range widths, and per-scenario budget parameters
from a collection of historical ScenarioData instances grouped by time-of-day period.
"""

using Statistics
using Dates

export compute_demand_bounds
export group_scenarios_by_period
export calibrate_demand_bounds
export create_period_aggregated_data
export impute_od_bounds

# Period index by hour-of-day start (matches :four_period profile)
const _PERIOD_WINDOWS = [
    (6,  10),   # 1 – morning
    (10, 15),   # 2 – afternoon
    (15, 20),   # 3 – evening
    (20, 24),   # 4 – night
]

"""
    group_scenarios_by_period(scenarios::Vector{ScenarioData}) -> Dict{Int, Vector{ScenarioData}}

Group scenario instances by four-period label (1=morning, 2=afternoon, 3=evening, 4=night)
based on the hour of `start_time`.  Scenarios whose start hour does not fall within
any window (e.g. 00-06) are silently dropped.
"""
function group_scenarios_by_period(
        scenarios::Vector{ScenarioData}
    )::Dict{Int, Vector{ScenarioData}}

    groups = Dict{Int, Vector{ScenarioData}}(i => ScenarioData[] for i in 1:4)

    for sc in scenarios
        isnothing(sc.start_time) && continue
        h = hour(sc.start_time)
        period = _hour_to_period(h)
        isnothing(period) && continue
        push!(groups[period], sc)
    end

    return groups
end

function _hour_to_period(h::Int)::Union{Int, Nothing}
    for (idx, (lo, hi)) in enumerate(_PERIOD_WINDOWS)
        lo <= h < hi && return idx
    end
    return nothing
end


"""
    compute_demand_bounds(
        scenario_groups::Dict{Int, Vector{ScenarioData}};
        q_high_quantile::Float64 = 0.90,
        Q_cap_quantile::Float64  = 0.90,
    ) -> (q_low, q_hat, B, Q_cap)

Calibrate demand bounds from historical scenario observations grouped by period.

Lower bounds are fixed at zero for all OD pairs.  This is motivated by the
sparsity of the demand data: most OD pairs appear on fewer than 10% of days,
so a quantile-based lower bound would be zero anyway for the vast majority of
pairs.  More importantly, setting q̲ = 0 is the conservative choice: it lets
the adversary concentrate the full budget on the most costly corridors rather
than being forced to spread some demand onto cheap pairs.

# Arguments
- `scenario_groups`: output of `group_scenarios_by_period`
- `q_high_quantile`: upper quantile for per-OD demand (default 90th percentile)
- `Q_cap_quantile`: quantile for the total-demand cap Q̄_s (default 90th percentile)

# Returns
- `q_low ::Dict{Int, Dict{Tuple{Int,Int}, Float64}}` — q̲_ods = 0 for all (od, s)
- `q_hat ::Dict{Int, Dict{Tuple{Int,Int}, Float64}}` — q̂_ods = q̄_ods (since q̲ = 0)
- `B     ::Vector{Float64}` — B_s = Q̄_s (since Σ q̲ = 0)
- `Q_cap ::Vector{Float64}` — Q̄_s (for reference/serialisation)

OD pairs that never appear in period s are omitted from q_low[s] and q_hat[s].
"""
function compute_demand_bounds(
        scenario_groups::Dict{Int, Vector{ScenarioData}};
        q_high_quantile::Float64 = 0.90,
        Q_cap_quantile::Float64  = 0.90,
    )

    n_periods = length(scenario_groups)
    q_low  = Dict{Int, Dict{Tuple{Int,Int}, Float64}}()
    q_hat  = Dict{Int, Dict{Tuple{Int,Int}, Float64}}()
    Q_cap  = Vector{Float64}(undef, n_periods)
    B      = Vector{Float64}(undef, n_periods)

    for s in sort(collect(keys(scenario_groups)))
        instances = scenario_groups[s]

        # Collect per-OD demand counts across all historical instances
        od_demand_hist = Dict{Tuple{Int,Int}, Vector{Float64}}()
        total_demand_hist = Float64[]

        for sc in instances
            isempty(sc.requests) && continue
            _require_indexed_request_columns(sc.requests)
            od_count = Dict{Tuple{Int,Int}, Int}()
            for row in eachrow(sc.requests)
                key = (row.origin_idx, row.dest_idx)
                od_count[key] = get(od_count, key, 0) + 1
            end
            for (od, cnt) in od_count
                push!(get!(od_demand_hist, od, Float64[]), Float64(cnt))
            end
            push!(total_demand_hist, Float64(sum(values(od_count))))
        end

        # Per-OD quantiles (missing observations treated as 0)
        n_obs = length(instances)
        q_low_s = Dict{Tuple{Int,Int}, Float64}()
        q_hat_s = Dict{Tuple{Int,Int}, Float64}()

        for (od, hist) in od_demand_hist
            # Pad with zeros for days where this OD pair had no demand
            n_zeros = max(0, n_obs - length(hist))
            full_hist = vcat(fill(0.0, n_zeros), hist)
            hi = quantile(full_hist, q_high_quantile)
            q_low_s[od] = 0.0
            q_hat_s[od] = hi   # q̂ = q̄ − q̲ = q̄ since q̲ = 0
        end

        q_low[s] = q_low_s
        q_hat[s] = q_hat_s

        # Total-demand cap
        if isempty(total_demand_hist)
            Q_cap[s] = 0.0
        else
            Q_cap[s] = quantile(total_demand_hist, Q_cap_quantile)
        end

        # B_s = Q_cap since Σ q_low = 0
        B[s] = Q_cap[s]
    end

    return q_low, q_hat, B, Q_cap
end


"""
    impute_od_bounds(
        q_hat::Dict{Int, Dict{Tuple{Int,Int}, Float64}},
        all_od_pairs::Set{Tuple{Int,Int}};
        floor_value::Union{Float64, Nothing}=nothing,
    ) -> Dict{Int, Dict{Tuple{Int,Int}, Float64}}

Impute upper-bound estimates for OD pairs absent from calibration data.

For each period, applies rank-1 gravity factorization (Option B):

    q̂_gravity(o,d) = O(o) × D(d) / Q

where O(o) = Σ_d q̂(o,d), D(d) = Σ_o q̂(o,d), Q = Σ_{o,d} q̂(o,d) are
computed from the existing positive q̂ entries.

Falls back to `floor_value` when gravity is zero (cold origin or destination).
If `floor_value` is not supplied, uses the minimum positive q̂ entry across all
periods (the smallest data-anchored upper bound), or 1.0 if no positive entries
exist anywhere.

All imputed values are rounded up to the nearest positive integer (ceiling ≥ 1)
so that no OD pair is assigned a fractional or sub-unit upper bound.
Existing positive values are preserved unchanged.  Only (o,d) pairs whose
current q̂ is zero or absent are touched.
"""
function impute_od_bounds(
    q_hat::Dict{Int, Dict{Tuple{Int,Int}, Float64}},
    all_od_pairs::Set{Tuple{Int,Int}};
    floor_value::Union{Float64, Nothing}=nothing,
)::Dict{Int, Dict{Tuple{Int,Int}, Float64}}

    effective_floor = if isnothing(floor_value)
        min_positive = minimum(
            (v for (_, od_dict) in q_hat for (_, v) in od_dict if v > 0);
            init=Inf,
        )
        isinf(min_positive) ? 1.0 : min_positive
    else
        floor_value
    end

    _ceil_positive(v::Float64) = Float64(max(1, ceil(Int, v)))

    result = Dict{Int, Dict{Tuple{Int,Int}, Float64}}()

    for (p, od_dict) in q_hat
        origin_totals = Dict{Int, Float64}()
        dest_totals   = Dict{Int, Float64}()
        Q = 0.0

        for ((o, d), v) in od_dict
            v > 0 || continue
            origin_totals[o] = get(origin_totals, o, 0.0) + v
            dest_totals[d]   = get(dest_totals,   d, 0.0) + v
            Q += v
        end

        new_dict = copy(od_dict)

        for (o, d) in all_od_pairs
            get(new_dict, (o, d), 0.0) > 0 && continue
            O = get(origin_totals, o, 0.0)
            D = get(dest_totals,   d, 0.0)
            gravity = (Q > 0 && O > 0 && D > 0) ? O * D / Q : 0.0
            new_dict[(o, d)] = _ceil_positive(max(gravity, effective_floor))
        end

        result[p] = new_dict
    end

    return result
end


"""
    calibrate_demand_bounds(
        stations::DataFrame,
        requests::DataFrame,
        walking_costs::Dict{Tuple{Int,Int}, Float64},
        start_date::Date,
        end_date::Date;
        routing_costs=nothing,
        profile::Symbol=:four_period,
        q_high_quantile::Float64=0.90,
        Q_cap_quantile::Float64=0.90,
    ) -> (q_low, q_hat, B, Q_cap)

Calibrate demand bounds from historical data for use in RobustTotalDemandCapModel.

Generates one ScenarioData per (date × period) instance over [start_date, end_date]
using generate_scenarios_by_profile (so each ScenarioData contains exactly one
day's requests in one time-of-day window), groups by period, then computes
quantile-based per-OD bounds and total-demand caps via compute_demand_bounds.

This is the canonical entry point for producing the q_low/q_hat/B parameters
that RobustTotalDemandCapModel requires. The result is typically serialised to a
JSON file and loaded at optimization time.
"""
function calibrate_demand_bounds(
    stations::DataFrame,
    requests::DataFrame,
    walking_costs::Dict{Tuple{Int,Int}, Float64},
    start_date::Date,
    end_date::Date;
    routing_costs::Union{Dict{Tuple{Int,Int}, Float64}, Nothing}=nothing,
    profile::Symbol=:four_period,
    q_high_quantile::Float64=0.90,
    Q_cap_quantile::Float64=0.90,
)
    scenario_ranges = generate_scenarios_by_profile(start_date, end_date; profile=profile)
    data = create_station_selection_data(
        stations, requests, walking_costs;
        routing_costs=routing_costs, scenarios=scenario_ranges,
    )
    groups = group_scenarios_by_period(data.scenarios)
    q_low, q_hat, B, Q_cap = compute_demand_bounds(groups;
        q_high_quantile=q_high_quantile,
        Q_cap_quantile=Q_cap_quantile,
    )

    # Impute upper bounds for all feasible (o,d) pairs absent from calibration data,
    # so Omega_s in RobustTotalDemandCapMap covers every origin–destination pair.
    n = data.n_stations
    all_od_pairs = Set{Tuple{Int,Int}}((o, d) for o in 1:n for d in 1:n if o != d)
    q_hat = impute_od_bounds(q_hat, all_od_pairs)

    return q_low, q_hat, B, Q_cap
end


"""
    create_period_aggregated_data(
        stations::DataFrame,
        requests::DataFrame,
        walking_costs::Dict{Tuple{Int,Int}, Float64},
        start_date::Date,
        end_date::Date;
        routing_costs=nothing,
        profile::Symbol=:four_period,
    ) -> StationSelectionData

Build a StationSelectionData with one ScenarioData per time-of-day period,
where each ScenarioData aggregates all requests across every day in
[start_date, end_date] that fall within that period's time-of-day window.

Internally generates one ScenarioData per (date × period) via
generate_scenarios_by_profile (which correctly filters by time-of-day per day),
then groups by period and concatenates the requests. The resulting ScenarioData
has n_days = (end_date - start_date + 1), so NominalTwoStageODMap can divide
raw OD counts by n_days to obtain mean daily demand.

Use this in place of generate_scenarios_by_profile + create_station_selection_data
whenever you want 4 period-level scenarios for NominalTwoStageODModel or
RobustTotalDemandCapModel.
"""
function create_period_aggregated_data(
    stations::DataFrame,
    requests::DataFrame,
    walking_costs::Dict{Tuple{Int,Int}, Float64},
    start_date::Date,
    end_date::Date;
    routing_costs::Union{Dict{Tuple{Int,Int}, Float64}, Nothing}=nothing,
    profile::Symbol=:four_period,
)::StationSelectionData
    # Build daily per-period scenarios with correct time-of-day filtering
    daily_ranges = generate_scenarios_by_profile(start_date, end_date; profile=profile)
    daily_data = create_station_selection_data(
        stations, requests, walking_costs;
        routing_costs=routing_costs, scenarios=daily_ranges,
    )

    # Group daily scenarios by period
    groups = group_scenarios_by_period(daily_data.scenarios)

    # Aggregate each period's daily scenarios into one ScenarioData
    n_days = Dates.value(end_date - start_date) + 1
    period_scenarios = ScenarioData[]

    for p in sort(collect(keys(groups)))
        scs = groups[p]
        isempty(scs) && continue
        agg_requests = vcat([sc.requests for sc in scs]...)
        nrow(agg_requests) == 0 && continue
        # Preserve start_time so _period_from_scenario maps to the correct period
        push!(period_scenarios, ScenarioData(
            "period_$(p)",
            scs[1].start_time,
            scs[end].end_time,
            agg_requests,
            n_days,
        ))
    end

    return StationSelectionData(
        daily_data.stations,
        daily_data.n_stations,
        daily_data.station_id_to_array_idx,
        daily_data.array_idx_to_station_id,
        daily_data.walking_costs,
        daily_data.routing_costs,
        period_scenarios,
    )
end
