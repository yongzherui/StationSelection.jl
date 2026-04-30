"""
RobustTotalDemandCapModel — Two-stage robust station selection under demand uncertainty.

Uses the total-demand-cap uncertainty set:
    U_s = { q_s : 0 ≤ q_ods ≤ q̄_ods ∀(o,d),  Σ_{od} q_ods ≤ B_s }

The robust counterpart is derived via LP duality on the inner knapsack
maximisation; see report/main.tex §3–§5.
"""

export RobustTotalDemandCapModel

"""
    RobustTotalDemandCapModel <: AbstractODModel

Two-stage robust station selection with per-scenario total-demand cap.

# Fields
- `k`: active stations per scenario (second stage)
- `l`: stations to build (first stage)
- `in_vehicle_time_weight`: weight λ on in-vehicle routing cost c_{jk}
- `max_walking_distance`: walking-distance filter for valid (j,k) pairs (metres)
- `q_hat`: per-OD demand upper bounds q̄_ods; `q_hat[s][(o,d)]`
- `B`: per-scenario total-demand budgets B_s; `B[s]`
- `solve_mode`: `:cutting_plane` (default) or `:exact_dual`

Demand-bound data is pre-calibrated externally (see `compute_demand_bounds`).
The scenario indices in q_hat/B must align with the ScenarioData order
in the StationSelectionData passed to `run_opt`.

# Mathematical Formulation (robust counterpart)
    min  Σ_s B_s α_s + Σ_{s,od} q̄_ods β_ods
    s.t. α_s + β_ods ≥ t_ods                          ∀(o,d), s
         t_ods = Σ_{j,k} a_odjks x_odjks              ∀(o,d), s
         Σ_{j,k} x_odjks = 1                           ∀(o,d), s
         x_odjks ≤ z_{js},  x_odjks ≤ z_{ks}          ∀(o,d), j, k, s
         z_{js} ≤ y_j                                  ∀j, s
         Σ_j y_j = l,  Σ_j z_{js} = k                 ∀s
         x, α, β, t ≥ 0;  y, z ∈ {0,1}
"""
struct RobustTotalDemandCapModel <: AbstractODModel
    k::Int
    l::Int
    in_vehicle_time_weight::Float64
    max_walking_distance::Float64
    q_hat::Dict{Int, Dict{Tuple{Int,Int}, Float64}}
    B::Vector{Float64}
    solve_mode::Symbol

    function RobustTotalDemandCapModel(
            k::Int,
            l::Int;
            in_vehicle_time_weight::Number = 1.0,
            max_walking_distance::Number   = 300.0,
            q_hat::Dict{Int, Dict{Tuple{Int,Int}, Float64}},
            B::Vector{Float64},
            solve_mode::Symbol = :cutting_plane,
        )
        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        in_vehicle_time_weight >= 0 || throw(ArgumentError("in_vehicle_time_weight must be non-negative"))
        max_walking_distance >= 0   || throw(ArgumentError("max_walking_distance must be non-negative"))
        all(b >= 0 for b in B)      || throw(ArgumentError("all B[s] must be non-negative"))
        solve_mode in (:exact_dual, :cutting_plane) ||
            throw(ArgumentError("solve_mode must be :exact_dual or :cutting_plane"))

        new(k, l,
            Float64(in_vehicle_time_weight),
            Float64(max_walking_distance),
            q_hat, B, solve_mode)
    end
end
