"""
Label/bitsets/dominance types specific to the revisit-tolerant pricer, plus
the operational contract those types encode. See `../types.jl` for the
pricing graph/duals types this pricer shares with `station_simple/`, and
`seed.jl` / `extend.jl` / `prune.jl` / `dominate.jl` / `context.jl` /
`hooks.jl` for the label-setting functionality built on top of the types
below -- `seed.jl` is the file to start from for "is the label search
correct".

# Operational contract of the column being priced

This is deliberately **not** a finite-capacity passenger-loading problem. A
column is one unlimited-capacity vehicle route under the following synchronized
service assumptions:

- the route clock and every passenger's wait clock start at `t = 0`;
- a visit to origin station `j` can pick up every relevant `(j,k)` passenger
  when the visit's arrival time is at most `max_wait_time`;
- after that pickup, `(j,k)` is certified by a later visit to `k` when elapsed
  onboard time is at most `detour_factor * routing_cost(j,k)`; and
- certifying one pair consumes no capacity and does not prevent the same route
  from certifying any other pair whose independent wait/detour tests pass.

Accordingly, `station_age[j]` is the elapsed time since the most recent eligible
pickup visit to `j`, not a vehicle-load state. `served_pairs` is the set of all
pairs independently certified by the stop sequence. It can be broad: for
example, `[1,2,3,4]` can certify all six forward pairs when their time tests
pass. This high overlap is intended model behavior and is also why the route
master has a potentially large set-covering LP/IP gap: pricing gives a column
the sum of the dual rewards of every pair it certifies, whereas the final MIP
must purchase the whole route.

Do not add load resources or capacity dominance rules here unless the model's
operational semantics are intentionally being changed. Conversely, any future
finite-capacity version must add passenger quantities and leg-by-leg onboard
load state; pair certification alone is not a capacity formulation.
"""

export RouteCoveringPricingLabel

"""
A partial unlimited-capacity, synchronized-start route.

`time` is absolute time since the common `t=0` route/passenger start;
`station_age[j]` is ride time since the latest pickup-eligible visit to `j`;
and `served_pairs` records independently certified pairs. There is intentionally
no onboard passenger count or capacity resource. See this file's own module
docstring above for the full pricing contract.
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
(`_make_route_covering_label_bitsets`, `dominate.jl`) and reused across every
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
