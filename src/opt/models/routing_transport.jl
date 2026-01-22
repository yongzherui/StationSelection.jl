"""
Two-stage station selection with routing via transportation problem.

Extends the two-stage L model to include vehicle routing costs modeled
as a transportation problem (supply at pickup stations, demand at dropoff stations).
"""
module RoutingTransportModelDef

using ..AbstractModels: AbstractRoutingModel

export RoutingTransportModel

"""
    RoutingTransportModel <: AbstractRoutingModel

Two-stage model with vehicle routing modeled as transportation problem.

This model separates pickup and dropoff assignments and models vehicle
routing as a transportation problem where:
- Supply at each station = number of pickups assigned to it
- Demand at each station = number of dropoffs assigned to it
- Flow variables represent vehicle movements between stations

# Fields
- `k::Int`: Number of stations to activate per scenario
- `l::Int`: Number of permanent stations to build (must satisfy l ≥ k)
- `lambda::Float64`: Weight for routing cost relative to walking cost

# Mathematical Formulation
```
min   Σᵢⱼₛ pᵢₛ × wᵢⱼ × xᵖᵢⱼₛ           (pickup walking)
    + Σᵢⱼₛ dᵢₛ × wᵢⱼ × xᵈᵢⱼₛ           (dropoff walking)
    + λ × Σⱼₖₛ rⱼₖ × fⱼₖₛ              (routing cost)

s.t.  Σⱼ xᵖᵢⱼₛ = 1           ∀i,s     (pickup assignment)
      Σⱼ xᵈᵢⱼₛ = 1           ∀i,s     (dropoff assignment)
      Σₖ fⱼₖₛ = pⱼₛ          ∀j,s     (outflow = supply)
      Σⱼ fⱼₖₛ = dₖₛ          ∀k,s     (inflow = demand)
      xᵖᵢⱼₛ ≤ zⱼₛ            ∀i,j,s   (pickup at active only)
      xᵈᵢⱼₛ ≤ zⱼₛ            ∀i,j,s   (dropoff at active only)
      zⱼₛ ≤ yⱼ               ∀j,s     (activate only built)
      Σⱼ yⱼ = l                        (build l stations)
      Σⱼ zⱼₛ = k             ∀s       (activate k per scenario)
      fⱼₖₛ ≥ 0, x,y,z ∈ {0,1}
```

Where:
- pᵢₛ = pickup count at location i in scenario s
- dᵢₛ = dropoff count at location i in scenario s
- wᵢⱼ = walking cost from i to j
- rⱼₖ = routing (vehicle) cost from j to k
"""
struct RoutingTransportModel <: AbstractRoutingModel
    k::Int
    l::Int
    lambda::Float64

    function RoutingTransportModel(k::Int, l::Int; lambda::Float64=1.0)
        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        lambda >= 0 || throw(ArgumentError("lambda must be non-negative"))
        new(k, l, lambda)
    end
end

end # module
