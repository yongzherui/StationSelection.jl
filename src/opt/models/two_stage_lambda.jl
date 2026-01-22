"""
Two-stage station selection with λ penalty for scenario activations.

First stage selects permanent stations, second stage activates subsets
per scenario with a penalty for each activation.
"""
module TwoStageLambdaModelDef

using ..AbstractModels: AbstractTwoStageModel

export TwoStageLambdaModel

"""
    TwoStageLambdaModel <: AbstractTwoStageModel

Two-stage model with λ penalty for scenario-specific activations.

# Fields
- `k::Int`: Number of permanent stations to build
- `lambda::Float64`: Penalty weight for each scenario activation

# Mathematical Formulation
```
min   Σᵢⱼₛ rᵢₛ × cᵢⱼ × xᵢⱼₛ + λ × Σⱼₛ zⱼₛ
s.t.  Σⱼ xᵢⱼₛ = 1        ∀i,s      (each location assigned per scenario)
      xᵢⱼₛ ≤ zⱼₛ         ∀i,j,s    (assign only to active stations)
      zⱼₛ ≤ yⱼ           ∀j,s      (activate only built stations)
      Σⱼ yⱼ = k                     (build exactly k stations)
      x,y,z ∈ {0,1}
```

When λ = 0, each scenario can freely activate any built station.
Higher λ encourages using fewer total activations across scenarios.
"""
struct TwoStageLambdaModel <: AbstractTwoStageModel
    k::Int
    lambda::Float64

    function TwoStageLambdaModel(k::Int; lambda::Float64=0.0)
        k > 0 || throw(ArgumentError("k must be positive"))
        lambda >= 0 || throw(ArgumentError("lambda must be non-negative"))
        new(k, lambda)
    end
end

end # module
