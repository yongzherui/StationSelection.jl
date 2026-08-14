"""
Clustering formulations for station selection, paired with `StationSelectionProblem`:
`l` (stations built) and `max_walking_distance` come from `problem`, not from these
structs -- the redundant per-formulation `l`/`max_walking_distance` fields from the
initial split have been dropped so there is exactly one source of truth for each.

- `ClusteringBaseFormulation`: single-scenario k-medoids, station-to-station assignment.
  No build/activate split, so `problem.l` *is* the station count selected directly --
  this formulation carries no fields of its own, purely a dispatch marker.
- `ClusteringTwoStageFormulation`: two-stage (build/activate) station-to-station
  assignment. Carries `k` (activate per scenario); `l` (build) is `problem.l`.
- `ClusteringTwoStageODFormulation`: two-stage (build/activate) OD pickup/dropoff
  assignment. Carries `k`, `in_vehicle_time_weight`.
- `ClusteringTwoStageODFlowRegularizerFormulation`: `ClusteringTwoStageODFormulation` +
  a route-activation flow-regularization penalty. Carries `k`, `in_vehicle_time_weight`,
  `flow_regularization_weight`.

`k <= problem.l` can no longer be validated at formulation-construction time (no `l` in
scope there) -- it's checked in `_build_clustering!` instead, the first thing each
two-stage formulation's build path does once `problem` and `formulation` meet.
"""

export AbstractClusteringFormulation
export AbstractClusteringTwoStageODFormulation
export ClusteringBaseFormulation
export ClusteringTwoStageFormulation
export ClusteringTwoStageODFormulation
export ClusteringTwoStageODFlowRegularizerFormulation

"""
    AbstractClusteringFormulation <: AbstractFormulation

Shared supertype for all four clustering formulations, so `build_model`/`create_map`
can each keep a single method dispatched statically here instead of one method per
concrete type.
"""
abstract type AbstractClusteringFormulation <: AbstractFormulation end

"""
    ClusteringBaseFormulation <: AbstractClusteringFormulation

Basic k-medoids clustering formulation for station selection: selects `problem.l`
stations to minimize total walking distance from request origins and destinations to
their nearest selected station. No fields -- single-stage has no build/activate split,
so there is no formulation-specific knob left once `l`/`max_walking_distance` moved to
`StationSelectionProblem`.

# Decision Variables
- `y[j]`: Binary, 1 if station j is selected as a medoid
- `x[i,j]`: Binary, 1 if request point i is assigned to station j

# Objective
Minimize total weighted walking cost:
    min Σᵢ Σ_{j ∈ Aᵢ} qᵢ · d(i,j) · x[i,j]

where qᵢ is the request count at station location i. Here Aᵢ is the set of
admissible cluster centers for demand point i; with a walking limit it is
{j : d(i,j) ≤ mwd}, otherwise all candidate stations.

# Constraints
- Each station location assigned to exactly one medoid
- Can only assign to selected stations
- Select exactly `problem.l` stations
"""
struct ClusteringBaseFormulation <: AbstractClusteringFormulation end

"""
    ClusteringTwoStageFormulation <: AbstractClusteringFormulation

Two-stage stochastic station selection formulation with station-to-station clustering
assignments.

# Fields
- `k::Int`: Number of stations to activate per scenario. Stations built (`l`) comes
  from `StationSelectionProblem.l`.

# Mathematical Formulation
For each scenario s, let q_{is} be the number of request endpoints located at
candidate station i (counting both origins and destinations, aggregated across
time). Decision variables:

- y_j ∈ {0,1}: station j is built in the first stage
- z_{js} ∈ {0,1}: built station j is activated in scenario s
- x_{ijs} ∈ {0,1}: demand point i is assigned to active station j in scenario s

Objective:
    min  Σ_s Σ_i Σ_{j ∈ A_i} q_{is} d_{ij} x_{ijs}

Subject to:
    Σ_j y_j = l
    Σ_j z_{js} = k                     ∀s
    z_{js} ≤ y_j                       ∀j,s
    Σ_{j ∈ A_i} x_{ijs} = 1            ∀i,s with q_{is} > 0
    x_{ijs} ≤ z_{js}                   ∀i,j,s

Here A_i is the set of admissible cluster centers for demand point i; with a
walking limit it is {j : d(i,j) ≤ mwd}, otherwise all candidate stations.
"""
struct ClusteringTwoStageFormulation <: AbstractClusteringFormulation
    k::Int

    function ClusteringTwoStageFormulation(k::Int)
        k > 0 || throw(ArgumentError("k must be positive"))
        new(k)
    end
end

"""
    AbstractClusteringTwoStageODFormulation <: AbstractClusteringFormulation

Shared supertype for `ClusteringTwoStageODFormulation` and
`ClusteringTwoStageODFlowRegularizerFormulation`, both of which produce the identical
`ClusteringTwoStageODMap` (flow regularization changes the objective/variable/constraint
set `_build_clustering!` adds, not the OD mapping itself) -- lets `create_map` dispatch
on one type instead of duplicating a method per sibling.
"""
abstract type AbstractClusteringTwoStageODFormulation <: AbstractClusteringFormulation end

"""
    ClusteringTwoStageODFormulation <: AbstractClusteringTwoStageODFormulation

Two-stage stochastic station selection formulation with OD pair assignments (no flow
regularization -- see `ClusteringTwoStageODFlowRegularizerFormulation` for that).

# Fields
- `k::Int`: Number of stations to activate per scenario. Stations built (`l`) comes
  from `StationSelectionProblem.l`.
- `in_vehicle_time_weight::Float64`: Weight for in-vehicle travel time costs (c_{jk})

# Mathematical Formulation
First stage: Select l stations to build (y[j] ∈ {0,1})
Second stage: For each scenario s, activate k stations (z[j,s] ∈ {0,1})
              and assign OD demand counts to valid station pairs (x[s][p][idx] ∈ Z₊)

Objective:
    min Σ_s Σ_{p∈Ω_s} Σ_{(j,k)∈A_p} (d^origin_{oj} + d^dest_{dk} + w_ivt·c_{jk}) x_{p,jk,s}

Constraints:
- Σ_j y[j] = l                              (build exactly l stations)
- Σ_j z[j,s] = k  ∀s                        (activate k stations per scenario)
- z[j,s] ≤ y[j]  ∀j,s                       (can only activate built stations)
- Σ_{(j,k)∈A_p} x[s][p][j,k] = Q_s[s][p]  ∀s,p
- x[s][p][j,k] ≤ Q_s[s][p] * z[j,s], Q_s[s][p] * z[k,s]  ∀s,p,(j,k)∈A_p
"""
struct ClusteringTwoStageODFormulation <: AbstractClusteringTwoStageODFormulation
    k::Int
    in_vehicle_time_weight::Float64

    function ClusteringTwoStageODFormulation(
            k::Int;
            in_vehicle_time_weight::Number=1.0,
        )
        k > 0 || throw(ArgumentError("k must be positive"))
        in_vehicle_time_weight >= 0 || throw(ArgumentError("in_vehicle_time_weight must be non-negative"))
        new(k, Float64(in_vehicle_time_weight))
    end
end

"""
    ClusteringTwoStageODFlowRegularizerFormulation <: AbstractClusteringTwoStageODFormulation

`ClusteringTwoStageODFormulation` plus a route-activation flow-regularization penalty:
adds f_flow[s][(j,k)] variables and penalises distinct (j,k) segments weighted by
routing time.

Objective (additional term over `ClusteringTwoStageODFormulation`):
    + μ Σ_s Σ_{(j,k)} c_{jk} × f_flow[s][(j,k)]

# Fields
- `k`, `in_vehicle_time_weight`: see `ClusteringTwoStageODFormulation`
- `flow_regularization_weight::Float64`: weight μ for the route-activation penalty.
  Required (not optional) -- a formulation with no flow term at all is exactly what
  `ClusteringTwoStageODFormulation` already represents, so there is no meaningful
  "off" value here.
"""
struct ClusteringTwoStageODFlowRegularizerFormulation <: AbstractClusteringTwoStageODFormulation
    k::Int
    in_vehicle_time_weight::Float64
    flow_regularization_weight::Float64

    function ClusteringTwoStageODFlowRegularizerFormulation(
            k::Int;
            in_vehicle_time_weight::Number=1.0,
            flow_regularization_weight::Number,
        )
        k > 0 || throw(ArgumentError("k must be positive"))
        in_vehicle_time_weight >= 0 || throw(ArgumentError("in_vehicle_time_weight must be non-negative"))
        flow_regularization_weight >= 0 ||
            throw(ArgumentError("flow_regularization_weight must be non-negative"))
        new(k, Float64(in_vehicle_time_weight), Float64(flow_regularization_weight))
    end
end
