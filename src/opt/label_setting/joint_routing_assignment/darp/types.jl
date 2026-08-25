"""
Label/bitsets/dominance/pricing-data types for `darp/`: a first-commit
alternative to `exact/`'s running-max crediting, built as a controlled
comparison point. See `darp.jl`'s module docstring for the search-context
wiring; this file is the one to read for the reward model and dominance
soundness argument.

# Reward model, in one paragraph

Unlike `exact/` (`../types.jl`/`../data.jl`), which lets a route credit a
passenger's single *best* certified `(j, k)` regardless of visitation order
(via the reward-layer running-max trick), `darp/` credits whichever `(j, k)`
a not-yet-served passenger's assignment is *first* certified by, in
route-visitation order -- closer to how a real dial-a-ride request is
committed once a vehicle actually stops for it. This needs no reward-layer
preprocessing: each passenger contributes at most one reward, decided the
moment it happens, so the label tracks the concrete winning triple directly
(`served::Dict{Int, Tuple{Int,Int}}`) instead of a running-maximum proxy.
`exact/`'s replay-based reconstruction of concrete `(p,j,k)` assignments
(`exact/exact.jl`'s `_replay_joint_routing_assignment_route`) has no
counterpart here: since credit is assigned exactly once, in order, at
extension time, a finished label's own `served` field already *is* the final
answer -- see `darp.jl`'s `_pricing_candidate_from_label`, a trivial
projection like `route_covering/exact/exact.jl`'s, not a replay.

Still unlimited-capacity, synchronized-start, revisit-tolerant -- same
physical route contract as `RouteCoveringPricingLabel`/
`JointRoutingAssignmentPricingLabel` (see either's module docstring). Only
what a route gets *credit* for differs.

# Dominance: subset + compensated, not open-request equality

Textbook DARP/PDPTW branch-and-price dominance requires *equal* open-request
sets between two labels, because an open request there is an obligation --
the vehicle must still detour to it. Nothing here is obligatory: an
uncommitted passenger is simply left to `x_walk` (or another route) in the
master, so a label with *fewer* committed passengers than another is not
missing an obligation, just missing upside -- exactly the reward-collection
framing `route_covering/exact/`'s `served_pairs` dominance already relies on.
So dominance here reuses that same subset-with-compensated-budget test
(`_bitset_diff_weight`, `label_setting/utils.jl`), over a `served`-passenger
bitset instead of a served-pair bitset.

The one wrinkle: `route_covering`'s per-pair weight (`sigma[(j,k)]`) is a
single fixed dual value regardless of how a pair got served, so its weight
vector is exact. Here, a served passenger `p`'s *true* credited reward
depends on which of `p`'s candidate assignments happened to be first --
different labels can have credited `p` different amounts. Using the true
value per label would mean carrying it in the bitset mirror; instead
`passenger_weight[p]` below is deliberately an *upper bound* (`max` over
`p`'s positive-reward candidates), which only ever **overcharges** the
compensated test's budget check. An overcharge can only cause a sound label
to be wrongly rejected as non-dominant (lost pruning, never lost
correctness) -- see `_bitset_diff_weight`'s own docstring for why
`compensation <= budget` is what soundness requires, and note an upper bound
on `compensation` only ever makes that test harder to pass, not easier.
"""

export JointRoutingAssignmentDarpPricingData
export JointRoutingAssignmentDarpPricingLabel

struct JointRoutingAssignmentDarpPricingData
    scenario::Int
    nodes::Vector{Int}
    travel_cost::Dict{Tuple{Int, Int}, Float64}
    route_regularization_weight::Float64
    repositioning_time::Float64
    max_wait_time::Float64
    max_stops::Int
    bounded_max_stops::Bool
    compensated_dominance::Bool
    n_passengers::Int
    # weight[p] = max positive reward over p's candidates -- an admissible
    # upper bound used both by the remaining-reward priority bound and by the
    # compensated-dominance budget charge (see module docstring above).
    passenger_weight::Vector{Float64}
    candidates::Vector{PassengerAssignmentCandidate}
    candidates_by_origin::Dict{Int, Vector{Int}}
    candidates_by_destination::Dict{Int, Vector{Int}}
    candidates_by_passenger::Vector{Vector{Int}}
end

"""
A partial unlimited-capacity, synchronized-start route (`current`, `route`,
`time`, `station_age`, `tau`, `reduced_cost`, `route_length` all carry the
same meaning as `RouteCoveringPricingLabel`/`JointRoutingAssignmentPricingLabel`).
`served[p] = (origin, destination)` is the one assignment `p` was first
committed to -- unlike `exact/`'s reward-layer proxy, this already is the
final answer for a finished label; no replay step recovers it after the
fact. See this file's module docstring for the first-commit crediting rule.
"""
struct JointRoutingAssignmentDarpPricingLabel
    current::Int
    route::Vector{Int}
    time::Float64
    station_age::Dict{Int, Float64}
    served::Dict{Int, Tuple{Int, Int}}
    tau::Float64
    reduced_cost::Float64
    route_length::Int
end

const JointRoutingAssignmentDarpLabelId = Int
const JointRoutingAssignmentDarpLabelOrderKey = Tuple{Float64, Float64, Int, Int}

"""
Hot-path mirror of a `JointRoutingAssignmentDarpPricingLabel`, the same shape
as `RouteCoveringLabelBitsets` (`route_covering/exact/types.jl`): `served_bits`
is `keys(label.served)` as a `BitSet` -- passenger indices are already dense
(see `PassengerAssignmentCandidate`'s docstring, `../types.jl`), so this needs
no extra reindexing dict, unlike `route_covering`'s pair-index translation.
"""
struct JointRoutingAssignmentDarpLabelBitsets
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
struct JointRoutingAssignmentDarpDominanceFilters
    reduced_cost::Float64
    time::Float64
    age_mask::UInt64
    route_length::Int32
    n_live_ages::Int32
end

"""
    JointRoutingAssignmentDarpDominanceRules{BoundedStops, Compensated}

The two dominance switches, carried in the *type* rather than as `Bool`
arguments, for the same zero-cost-specialization reason as
`RouteCoveringDominanceRules`/`JointRoutingAssignmentDominanceRules` (see
either's own docstring).
"""
struct JointRoutingAssignmentDarpDominanceRules{BoundedStops, Compensated} <: AbstractPricingDominanceRules end

_joint_routing_assignment_darp_dominance_rules(bounded_max_stops::Bool, compensated_dominance::Bool) =
    JointRoutingAssignmentDarpDominanceRules{bounded_max_stops, compensated_dominance}()
