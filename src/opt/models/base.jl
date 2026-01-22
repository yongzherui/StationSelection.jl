"""
Base station selection model (k-medoids style).

Single-scenario model that selects k stations to minimize total walking cost.
"""
module BaseModelDef

using ..AbstractModels: AbstractSingleScenarioModel

export BaseModel

"""
    BaseModel <: AbstractSingleScenarioModel

Basic k-medoids style station selection model.

Selects exactly (or at most) k stations to minimize the total walking
cost for all customer requests.

# Fields
- `k::Int`: Number of stations to select
- `strict_equality::Bool`: If true, select exactly k stations; if false, at most k

# Mathematical Formulation
```
min   Σᵢⱼ rᵢ × cᵢⱼ × xᵢⱼ           (minimize walking cost)
s.t.  Σⱼ xᵢⱼ = 1        ∀i        (each location assigned to one station)
      xᵢⱼ ≤ yⱼ          ∀i,j      (can only assign to selected stations)
      Σⱼ yⱼ = k (or ≤ k)          (select k stations)
      xᵢⱼ ∈ {0,1}, yⱼ ∈ {0,1}
```
"""
struct BaseModel <: AbstractSingleScenarioModel
    k::Int
    strict_equality::Bool

    function BaseModel(k::Int; strict_equality::Bool=true)
        k > 0 || throw(ArgumentError("k must be positive"))
        new(k, strict_equality)
    end
end

end # module
