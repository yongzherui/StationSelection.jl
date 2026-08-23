"""
Label/bitsets/dominance types specific to the revisit-tolerant pricer
(`exact.jl`). See `../types.jl` for the pricing graph/duals types this pricer
shares with `station_simple/`.
"""

export RouteCoveringPricingLabel

"""
A partial unlimited-capacity, synchronized-start route.

`time` is absolute time since the common `t=0` route/passenger start;
`station_age[j]` is ride time since the latest pickup-eligible visit to `j`;
and `served_pairs` records independently certified pairs. There is intentionally
no onboard passenger count or capacity resource. See `labels.jl` for the full
pricing contract.
"""
struct RouteCoveringPricingLabel
    current::Int
    route::Vector{Int}
    time::Float64
    station_age::Dict{Int, Float64}
    served_pairs::Set{Tuple{Int, Int}}
    tau::Float64
    reduced_cost::Float64
    route_length::Int
end

const RouteCoveringLabelId = Int
const RouteCoveringLabelOrderKey = Tuple{Float64, Float64, Int, Int}

"""
Hot-path mirror of a `RouteCoveringPricingLabel`, built once per label
(`_make_route_covering_label_bitsets`, `labels.jl`) and reused across every
dominance test it's involved in. `served_bits` is `served_pairs` reindexed to
dense pair indices (a `BitSet`, for fast subset/diff tests); `age_idx`/
`age_val` are `station_age` as parallel sorted arrays (station index ->
age), the same sparse representation `RouteCoveringDominanceFilters` and the
remaining-reward bound both walk; `age_mask` is a folded-bit summary of
`age_idx` used as a cheap prefilter before the real sorted-array walk.
"""
struct RouteCoveringLabelBitsets
    served_bits::BitSet
    age_idx::Vector{Int32}
    age_val::Vector{Float64}
    age_mask::UInt64
end

"""
Every piece of label state the dominance scan can reject a candidate with
using only scalar comparisons -- see `PricingLabelEntry`'s docstring
(`label_setting/types.jl`) for why this is stored inline rather than behind a
pointer to the label. `n_live_ages` is `length(age_idx)`, i.e. how many
stations currently have a live pickup clock.
"""
struct RouteCoveringDominanceFilters
    reduced_cost::Float64
    time::Float64
    age_mask::UInt64
    route_length::Int32
    n_live_ages::Int32
end

"""
    RouteCoveringDominanceRules{BoundedStops, Compensated}

The two dominance switches, carried in the *type* rather than as `Bool`
arguments, for the same reason as Joint's twin
(`JointRoutingAssignmentDominanceRules`, `joint_routing_assignment/exact/types.jl`):
they are constant for a whole pricing call, and encoding them as type
parameters lets the compiler delete a disabled `BoundedStops` branch outright
and specialize the reward-diff walk on `Compensated`.
"""
struct RouteCoveringDominanceRules{BoundedStops, Compensated} <: AbstractPricingDominanceRules end

_route_covering_dominance_rules(bounded_max_stops::Bool, compensated_dominance::Bool) =
    RouteCoveringDominanceRules{bounded_max_stops, compensated_dominance}()
