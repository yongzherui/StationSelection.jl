"""
    scripts/lib/diagnostics_common.jl

Shared building blocks for the ad hoc PFA / AggregateODRouteModel diagnostics
under `scripts/modes/`. Centralises the boilerplate every one-off diagnostic
used to reimplement independently: instance loading, the standard Zhuzhou
model config, env-var parsing, a shared Gurobi environment, the passenger
candidate builder, and the "seed an RMP and pull duals" snapshot used to price
against a realistic dual vector without running full CG.

`using StationSelection` must happen before this file is included (see
scripts/diagnose.jl).
"""

using Gurobi, JuMP, Printf

include(joinpath(@__DIR__, "..", "generate_zhuzhou_instance.jl"))

const DIAG_DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "..", "Data", "base_data"))

# One Gurobi.Env for the whole process -- constructing several has previously
# caused a silent multi-minute stall on this cluster.
const _DIAG_GRB_ENV = Ref{Union{Nothing, Gurobi.Env}}(nothing)
function diag_grb_env()
    isnothing(_DIAG_GRB_ENV[]) && (_DIAG_GRB_ENV[] = Gurobi.Env())
    return _DIAG_GRB_ENV[]
end

# ── env-var parsing ────────────────────────────────────────────────────────────

env_int(name::AbstractString, default::Int)    = parse(Int, get(ENV, name, string(default)))
env_float(name::AbstractString, default::Real) = parse(Float64, get(ENV, name, string(default)))
env_bool(name::AbstractString, default::Bool)  =
    lowercase(strip(get(ENV, name, default ? "1" : "0"))) in ("1", "true", "yes")
env_ints(name::AbstractString, default::AbstractString) =
    parse.(Int, split(get(ENV, name, default), ","))
env_floats(name::AbstractString, default::AbstractString) =
    parse.(Float64, split(get(ENV, name, default), ","))

"`raw <= 0` means TRUE unbounded (`typemax(Int)`, the model's own no-limit
sentinel) -- not the same as a large finite cap, which sets `bounded_max_stops`
and makes label dominance additionally compare route length."
diag_unbounded(raw::Int) = raw <= 0 ? typemax(Int) : raw

# ── standard Zhuzhou instance + model ──────────────────────────────────────────

diag_l_for(n::Int; divisor::Real=2) = max(2, ceil(Int, n / divisor))

"""
    diag_zz_data(n_stations; n_pairs=16, n_scenarios=3, seed=42)

The standard Zhuzhou instance most diagnostics in this repo are built on:
top-`n_stations` stations by request volume, `n_pairs` OD pairs per scenario.
"""
function diag_zz_data(n_stations::Int; n_pairs::Int=16, n_scenarios::Int=3, seed::Int=42)
    return generate_zhuzhou_data(DIAG_DATA_DIR, n_stations, n_pairs; n_scenarios=n_scenarios, seed=seed)
end

"""
    diag_zz_model(n_stations; kwargs...)

The repo's standard AggregateODRouteModel config -- route_weight=10.0 /
walk_weight=0.1 (the "route100x" convention), `l = ceil(n/2)`. Every keyword
can be overridden.
"""
function diag_zz_model(n_stations::Int;
    l                            :: Int     = diag_l_for(n_stations),
    route_regularization_weight  :: Float64 = 10.0,
    walk_cost_weight             :: Float64 = 0.1,
    repositioning_time           :: Float64 = 20.0,
    max_walking_distance         :: Float64 = 600.0,
    max_wait_time                :: Float64 = 900.0,
    detour_factor                :: Float64 = 2.0,
    max_stops                    :: Int      = 4,
    max_visits_per_node          :: Int      = 3,
    kwargs...,
)
    return AggregateODRouteModel(
        l;
        route_regularization_weight=route_regularization_weight,
        walk_cost_weight=walk_cost_weight,
        repositioning_time=repositioning_time,
        max_walking_distance=max_walking_distance,
        max_wait_time=max_wait_time,
        detour_factor=detour_factor,
        max_stops=max_stops,
        max_visits_per_node=max_visits_per_node,
        kwargs...,
    )
end

# ── passenger-candidate construction ───────────────────────────────────────────

"""
    diag_scenario_candidates(data, n_stations, scenario; max_walk=600.0, base_value=5000.0,
                              walk_cost_weight=0.1, detour_factor=2.0)

Every (request, pickup station, dropoff station) triple within `max_walk` of
both endpoints, with `reward = base_value - walk_cost_weight*(walk_o+walk_d)`
and `ride_limit = detour_factor * direct_routing_time(j,k)`.
"""
function diag_scenario_candidates(
    data::StationSelectionData, n_stations::Int, scenario::Int;
    max_walk::Float64=600.0, base_value::Float64=5000.0,
    walk_cost_weight::Float64=0.1, detour_factor::Float64=2.0,
)
    candidates = PassengerAssignmentCandidate[]
    for row in eachrow(data.scenarios[scenario].requests)
        o, d = row.origin_idx, row.dest_idx
        for j in 1:n_stations
            walk_o = get_walking_cost(data, o, j)
            walk_o <= max_walk || continue
            for k in 1:n_stations
                k == j && continue
                walk_d = get_walking_cost(data, d, k)
                walk_d <= max_walk || continue
                reward = base_value - walk_cost_weight * (walk_o + walk_d)
                reward > 0 || continue
                ride_limit = detour_factor * get_routing_cost(data, j, k)
                push!(candidates, PassengerAssignmentCandidate(row.id, j, k, ride_limit, reward))
            end
        end
    end
    return candidates
end

"Dense pairwise routing-cost dict over `1:n_stations`."
function diag_travel_cost(data::StationSelectionData, n_stations::Int)
    travel = Dict{Tuple{Int, Int}, Float64}()
    for i in 1:n_stations, j in 1:n_stations
        i == j || (travel[(i, j)] = get_routing_cost(data, i, j))
    end
    return travel
end

# ── seeded-RMP dual snapshot ────────────────────────────────────────────────────

"""
    diag_dual_snapshot(model, data; grb_env=diag_grb_env())
        -> (master, mapping, master_data, alpha, gamma_o, gamma_d)

Build the passenger free-assignment master, seed it with the two-stop columns,
solve the LP relaxation once, and extract duals -- the "one realistic pricing
snapshot" station-subset / Lagrangian / relaxation-design diagnostics need
before they can price anything, without running a full CG loop.
"""
function diag_dual_snapshot(model, data::StationSelectionData; grb_env=diag_grb_env())
    mapping = create_map(model, data)
    md = StationSelection.create_passenger_free_assignment_master_data(model, data, mapping)
    master = build_passenger_free_assignment_master(md, grb_env; relax_integrality=true)
    set_silent(master.model)
    next_id = 1
    for column in passenger_free_assignment_two_stop_seed_columns(md; next_column_id=next_id)
        StationSelection.add_passenger_free_assignment_column!(master, column)
        next_id += 1
    end
    optimize!(master.model)
    termination_status(master.model) == JuMP.MOI.OPTIMAL || error("seed master did not solve")
    alpha, gamma_o, gamma_d = extract_passenger_free_assignment_duals(master)
    return master, mapping, md, alpha, gamma_o, gamma_d
end

"Write `df` to `path`, warning (not throwing) on failure -- a reporting bug
should never abort the diagnostic that produced the data."
function diag_safe_csv_write(path::AbstractString, df)
    try
        CSV.write(path, df)
    catch err
        @warn "failed to write CSV" path exception=(err, catch_backtrace())
    end
end

# ── reward-ladder relaxation (shared by reward_ladder_census / clock_quantization) ─

const DIAG_LADDER_TOL = 1e-9

"Passenger `p`'s distinct positive reward values, grouped by the SAME tolerance
rule `_build_passenger_reward_layers` uses, so `m_p` here is exactly the number
of layers that constructor would emit for `p`."
function diag_distinct_reward_values(rewards::Vector{Float64})
    values = Float64[]
    for v in sort(rewards)
        (!isempty(values) && v - values[end] <= DIAG_LADDER_TOL) && continue
        push!(values, v)
    end
    return values
end

function diag_reward_values_by_passenger(candidates)
    by_p = Dict{Int, Vector{Float64}}()
    for c in candidates
        c.reward > DIAG_LADDER_TOL || continue
        push!(get!(() -> Float64[], by_p, c.passenger), c.reward)
    end
    return Dict(p => diag_distinct_reward_values(vs) for (p, vs) in by_p)
end

"Retain `L` of a passenger's distinct values, evenly spaced **in value** and
always including the maximum, so the worst-case round-up is `(v_max - v_min) / L`."
function diag_retained_values(values::Vector{Float64}, L::Int)
    length(values) <= L && return values
    v_min, v_max = first(values), last(values)
    kept = Float64[]
    for i in 1:L
        target = v_min + (v_max - v_min) * i / L
        idx = findfirst(v -> v >= target - DIAG_LADDER_TOL, values)
        idx === nothing && continue
        v = values[idx]
        (isempty(kept) || v > kept[end] + DIAG_LADDER_TOL) && push!(kept, v)
    end
    (isempty(kept) || kept[end] < v_max - DIAG_LADDER_TOL) && push!(kept, v_max)
    return kept
end

"Round every candidate's reward UP to its passenger's next retained rung. `L<=0`
disables the transform (returns `candidates` unchanged)."
function diag_coarsen_candidates(candidates, values_by_p, L::Int)
    L <= 0 && return candidates
    kept_by_p = Dict(p => diag_retained_values(vs, L) for (p, vs) in values_by_p)
    out = PassengerAssignmentCandidate[]
    for c in candidates
        if c.reward <= DIAG_LADDER_TOL
            push!(out, c)
            continue
        end
        kept = kept_by_p[c.passenger]
        idx = findfirst(v -> v >= c.reward - DIAG_LADDER_TOL, kept)
        rounded = idx === nothing ? kept[end] : kept[idx]
        push!(out, PassengerAssignmentCandidate(
            c.passenger, c.origin, c.destination, c.ride_limit, rounded,
        ))
    end
    return out
end
