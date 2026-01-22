"""
Two-stage station selection with L permanent and k active stations.

First stage builds L permanent stations, second stage activates k of them
per scenario.
"""
module TwoStageLModelDef

using ..AbstractModels: AbstractTwoStageModel

export TwoStageLModel

"""
    TwoStageLModel <: AbstractTwoStageModel

Two-stage model with L permanent stations and k active per scenario.

# Fields
- `k::Int`: Number of stations to activate per scenario
- `l::Int`: Number of permanent stations to build (must satisfy l ≥ k)

# Mathematical Formulation
```
min   Σᵢⱼₛ rᵢₛ × cᵢⱼ × xᵢⱼₛ
s.t.  Σⱼ xᵢⱼₛ = 1        ∀i,s      (each location assigned per scenario)
      xᵢⱼₛ ≤ zⱼₛ         ∀i,j,s    (assign only to active stations)
      zⱼₛ ≤ yⱼ           ∀j,s      (activate only built stations)
      Σⱼ yⱼ = l                     (build exactly l stations)
      Σⱼ zⱼₛ = k         ∀s        (activate exactly k per scenario)
      x,y,z ∈ {0,1}
```

This model is useful when infrastructure cost limits permanent builds (l),
but operational constraints require fewer active stations (k) per scenario.
"""
struct TwoStageLModel <: AbstractTwoStageModel
    k::Int
    l::Int

    function TwoStageLModel(k::Int, l::Int)
        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        new(k, l)
    end
end

end # module
