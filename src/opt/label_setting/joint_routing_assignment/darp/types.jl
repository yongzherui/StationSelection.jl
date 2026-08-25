"""
Label/bitsets/dominance/pricing-data types for `darp/`: a *provably
value-equivalent* alternative to `exact/`'s running-max crediting, built as a
controlled comparison point for how much `exact/`'s reward-layer trick is
worth *computationally* -- not a different, weaker reward model. See
`darp.jl`'s module docstring for the search-context wiring; this file is the
one to read for the reward model and dominance soundness argument.

# The equivalence this pricer is required to satisfy

**Run to exhaustion (unbounded `max_stops`, no early `n_candidates`/time-limit
cutoff), `darp/`'s optimal reduced cost for a scenario must equal `exact/`'s,
and the two `CGSolver` modes (`AggregateODRouteJointRoutingAssignmentFormulation.pricing_mode
= :exact` vs. `:darp`) must converge to the same master objective.** This is
not an empirical tendency, it's the whole point of the pricer: `exact/`'s
reward-layer running-max is a *computational shortcut* for "the search may
credit any passenger to whichever of their certifiable candidates is best,
independent of when each was reached" -- `darp/` computes that same optimum
the straightforward way, by literally giving the search that choice, rather
than encoding it away via a bitset trick. If the two ever disagree on a
finished search, that is a bug in this pricer (most likely in the branching
below), not an expected difference to shrug off.

# Reward model: commit-or-skip is a branch, not forced

A route's visit to a station where some not-yet-served passenger `p` has a
certifiable candidate does **not** automatically commit `p` -- committing `p`
there is one *option*; leaving `p` open (uncommitted, still eligible for a
possibly-better candidate reached later) is the other, and the search
branches into both. Forcing automatic commitment (this pricer's first
implementation) is unsound: it can make `exact/`'s winning assignment set
physically unrepresentable as *any* `darp/` route, whenever a passenger's
weaker candidate's station also lies on the cheapest path to their better
one -- concretely, `exact/` can pass through such a station "for free" (credit
nothing there, credit the better candidate later), while forced-commit
`darp/` cannot avoid taking the worse credit, or must pay extra travel to
route around the station entirely. Branching removes exactly this gap: for
any physical route and any target assignment achievable along it, the search
can choose "skip" at every station before the target candidate and "commit"
at the target itself, so `exact/`'s optimal assignment set is always reachable
as *one* of `darp/`'s branches. See
`_joint_routing_assignment_darp_eligible_at_node`/`_joint_routing_assignment_darp_commit_subsets`
(`data.jl`) and `_joint_routing_assignment_darp_candidate_next_nodes`
(`labels.jl`) for the mechanics -- `JointRoutingAssignmentDarpAction` below is
the `(next_node, commit_subset)` pair one branch corresponds to.

Each passenger contributes at most one reward regardless of which branch a
label took, so the label tracks the concrete committed triple directly
(`served::Dict{Int, Tuple{Int,Int}}`) rather than a running-maximum proxy --
no reward-layer preprocessing needed. `exact/`'s replay-based reconstruction
of concrete `(p,j,k)` assignments (`exact/exact.jl`'s
`_replay_joint_routing_assignment_route`) has no counterpart here: since
credit is assigned exactly once, at extension time, for whichever candidate a
given branch actually committed to, a finished label's own `served` field
already *is* the final answer -- see `darp.jl`'s `_pricing_candidate_from_label`,
a trivial projection like `route_covering/exact/exact.jl`'s, not a replay.

Still unlimited-capacity, synchronized-start, revisit-tolerant -- same
physical route contract as `RouteCoveringPricingLabel`/
`JointRoutingAssignmentPricingLabel` (see either's module docstring). Only
what a route gets *credit* for, and how that credit is chosen, differs.

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
`served[p] = (origin, destination)` is the one assignment `p` was committed
to by this label's particular sequence of commit/skip branch choices --
unlike `exact/`'s reward-layer proxy, this already is the final answer for a
finished label; no replay step recovers it after the fact. See this file's
module docstring for the commit-or-skip branching rule.
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
    JointRoutingAssignmentDarpAction

`(next_node, commit_subset)`: a physical move to `next_node`, paired with
which subset of the not-yet-served passengers newly eligible there
(`JointRoutingAssignmentDarpEligibility`, `data.jl`) this particular branch
commits -- everyone eligible but left out of `commit_subset` stays
uncommitted. One label-setting engine "action" (`label_setting/types.jl`'s
`_pricing_candidate_next_nodes`/`_pricing_extend_label` hooks) is one such
pair, not just a bare node id -- see `labels.jl`'s module docstring for why
this is how branching is expressed without changing the shared engine.
"""
const JointRoutingAssignmentDarpAction = Tuple{Int, Vector{Tuple{Int, Int, Float64}}}

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
