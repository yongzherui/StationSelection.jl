"""
Demand-bound calibration utilities for the robust total-demand-cap model.

Computes per-OD lower bounds, range widths, and per-scenario budget parameters
from a collection of historical ScenarioData instances grouped by time-of-day period.
"""

using Statistics
using Dates

export compute_demand_bounds
export group_scenarios_by_period

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
        q_low_quantile::Float64  = 0.10,
        q_high_quantile::Float64 = 0.90,
        Q_cap_quantile::Float64  = 0.90,
    ) -> (q_low, q_hat, B, Q_cap)

Calibrate demand bounds from historical scenario observations grouped by period.

# Arguments
- `scenario_groups`: output of `group_scenarios_by_period`
- `q_low_quantile`: lower quantile for per-OD demand (default 10th percentile)
- `q_high_quantile`: upper quantile for per-OD demand (default 90th percentile)
- `Q_cap_quantile`: quantile for the total-demand cap Q̄_s (default 90th percentile)

# Returns
- `q_low ::Dict{Int, Dict{Tuple{Int,Int}, Float64}}` — q̲_ods
- `q_hat ::Dict{Int, Dict{Tuple{Int,Int}, Float64}}` — q̂_ods = q̄_ods − q̲_ods
- `B     ::Vector{Float64}` — B_s = Q̄_s − Σ_{od} q̲_ods
- `Q_cap ::Vector{Float64}` — Q̄_s (for reference/serialisation)

OD pairs that never appear in period s are omitted from q_low[s] and q_hat[s].
B[s] is guaranteed ≥ 0 (enforced by clamping).
"""
function compute_demand_bounds(
        scenario_groups::Dict{Int, Vector{ScenarioData}};
        q_low_quantile::Float64  = 0.10,
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
            lo = quantile(full_hist, q_low_quantile)
            hi = quantile(full_hist, q_high_quantile)
            q_low_s[od]  = lo
            q_hat_s[od]  = max(0.0, hi - lo)
        end

        q_low[s] = q_low_s
        q_hat[s] = q_hat_s

        # Total-demand cap
        if isempty(total_demand_hist)
            Q_cap[s] = 0.0
        else
            Q_cap[s] = quantile(total_demand_hist, Q_cap_quantile)
        end

        # Budget B_s = Q_cap - Σ q_low; clamp to ≥ 0
        sum_q_low = sum(values(q_low_s); init=0.0)
        B[s] = max(0.0, Q_cap[s] - sum_q_low)
    end

    return q_low, q_hat, B, Q_cap
end
