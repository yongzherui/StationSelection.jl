"""
Label/bitsets/dominance/pricing-data types for `darp/`: a literal onboard-
bitset DARP-style pricer, built as a comparison point against `darp_modified/`'s
retroactive commit-or-skip and `exact/`'s reward-layer running-max. See
`context.jl`/`hooks.jl` for the search-context wiring and `driver.jl` for the
standalone comparison entrypoint; this file is the one to read for the
reward model and dominance argument.

This is a DARP-style label-setting algorithm adapted to this pricing problem,
not a general classical DARP solver: starts are synchronized, capacity is
unlimited, the only pickup time window is the shared `max_wait_time`, and the
pricing graph uses station-to-station metric-closure arcs rather than separate
depot/pickup/delivery vertices with service times.

# Reward model: boarding commits to a specific (j,k), dropoff is deterministic

Unlike `darp_modified/` (where "claiming" happens retroactively, at whichever
node ends up being the destination, using whichever origin's clock happens to
be live), this pricer makes boarding an explicit, committed decision at the
*origin*: at a station that's some not-yet-resolved passenger's candidate
origin (including a route's initial station), the search branches over every valid *board selection* -- for each
such passenger, either skip them here, or commit to exactly one of their
candidate destinations reachable from this origin (never more than one:
elementarity). That committed `(j,k)` is now fixed; dropoff, later, at `k` is
not a further decision -- it's the deterministic resolution of a promise
already made.

This is why boarding needs its own combinatorial enumeration
(`_joint_routing_assignment_darp_board_subsets`, `data.jl`) distinct from
`darp_modified/`'s: choosing between two different destinations *is* a
consequential decision here (different final targets, different rewards,
different deadlines), not a free reward-maximization the way picking the best
of several simultaneously-live options was there.

# No `station_age`

Boarding is decided the instant the vehicle is *at* the candidate origin,
using `label.time` directly -- there is nothing to remember about a
not-yet-boarded candidate between visits, so unlike every other pricer in
this package, this label carries no station-clock resource at all.

# `onboard`: a liability, not an opportunity -- and the resulting dominance shape

`onboard::Dict{Int, Tuple{Int,Int,Float64}}` (`p => (j, k, age since boarding)`)
tracks passengers currently committed but not yet delivered. It is the direct
analogue of textbook DARP's open-request set `Ω`, and it inherits `Ω`'s
subset-direction *reversal* from `station_age`: a station clock is an
opportunity (more support, fresher values, is better for the label claiming
to dominate), but an onboard commitment is an obligation still owed (fewer
commitments, at least as fresh on the ones shared, is better). So dominance
requires `onboard(a) ⊆ onboard(b)` -- the *opposite* support direction from
`station_age`'s `_pricing_dominates_at_state` twins elsewhere in this
package -- with `age_a(p) <= age_b(p)` on every commitment both still share.
Because the support and value directions don't couple the same way
`label_setting/utils.jl`'s sparse-age machinery assumes, this pricer has its
own small merge-walk (`_joint_routing_assignment_darp_onboard_ages_dominate`,
`dominate.jl`) rather than reusing that utility directly.

Reward is credited immediately at pickup, so an onboard entry is purely a
future feasibility liability rather than an unbooked economic opportunity.
`served::Set{Tuple{Int,Int,Int}}` (completed `(p,j,k)` triples whose reward was
already credited at pickup) uses **plain** subset dominance -- no compensation, no budget, at
your explicit direction: a triple-indexed compensated charge would need to
collapse back to passenger identity to stay sound (see the discussion that
led here), which defeats the point of tracking explicit triples in the first
place.

# Ride-limit violations are hard infeasibility

If it becomes impossible to honor *any* current commitment -- including ones
not being resolved at the node just reached -- the whole label is discarded,
not just that one commitment. This is a deliberate departure from every other
pricer in this package (which never invalidates a whole label over one
missed opportunity) and from `darp_modified/`'s recommended alternative
(prune just the dead entry) -- chosen because it's closer to how textbook
DARP resource-extension functions actually behave: an infeasible extension
simply produces no child, full stop. See
`_joint_routing_assignment_darp_candidate_next_nodes` (`extend.jl`) for where
this is enforced -- as an action-generation filter, not a per-entry prune,
since the shared engine's `_pricing_extend_label` hook cannot itself signal
"no child produced."
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
    n_passengers::Int
    # max positive reward over p's candidates -- an admissible (loose, not
    # tight) upper bound used only by the remaining-reward priority bound,
    # never by dominance (which is plain/exact here, no upper-bound weight
    # needed).
    passenger_weight::Vector{Float64}
    candidates::Vector{PassengerAssignmentCandidate}       # dense triple universe
    candidate_index::Dict{Tuple{Int, Int, Int}, Int}       # (p,j,k) -> dense index into candidates
    candidates_by_origin::Dict{Int, Vector{Int}}
    candidates_by_destination::Dict{Int, Vector{Int}}
    candidates_by_passenger::Vector{Vector{Int}}
end

"""
    JointRoutingAssignmentDarpAction

`(next_node, board_subset)`: a physical move to `next_node`, paired with
which not-yet-resolved passengers newly board *here*, each committed to one
specific candidate destination (`board_subset::Vector{(p, origin, destination,
reward)}`). One label-setting engine "action" (`label_setting/types.jl`'s
`_pricing_candidate_next_nodes`/`_pricing_extend_label` hooks) is one such
pair -- see `extend.jl`'s module docstring for why branching over board
selections has to be expressed as multiple actions rather than multiple
children of one action.
"""
const JointRoutingAssignmentDarpAction = Tuple{Int, Vector{Tuple{Int, Int, Int, Float64}}}

"""
A partial unlimited-capacity, synchronized-start route. `onboard[p] = (j, k,
age)` is passenger `p`'s committed pair and elapsed time since boarding it;
`served` is every `(p,j,k)` triple already delivered; reward enters
`reduced_cost` when the corresponding commitment boards. When an incomplete
label is harvested early, unfinished onboard commitments are omitted from the
assignments and their credited rewards are refunded from the projected
column's reduced cost.
`reduced_cost`. See this file's module docstring for the boarding/dropoff
contract and why there is no `station_age` field here.
"""
struct JointRoutingAssignmentDarpPricingLabel
    current::Int
    route::Vector{Int}
    time::Float64
    onboard::Dict{Int, Tuple{Int, Int, Float64}}
    served::Set{Tuple{Int, Int, Int}}
    tau::Float64
    reduced_cost::Float64
    route_length::Int
end

const JointRoutingAssignmentDarpLabelId = Int
const JointRoutingAssignmentDarpLabelOrderKey = Tuple{Float64, Float64, Int, Int}

"""
Hot-path mirror of a `JointRoutingAssignmentDarpPricingLabel`. `served_bits`/
`onboard_bits` are `served`/`keys(onboard)` reindexed to the dense candidate
universe (`JointRoutingAssignmentDarpPricingData.candidate_index`).
`onboard_age_idx`/`onboard_age_val`/`onboard_age_mask` are `onboard`'s ages as
the same sparse sorted-array representation `station_age` uses elsewhere in
this package, just keyed by dense triple index instead of station id.
"""
struct JointRoutingAssignmentDarpLabelBitsets
    served_bits::BitSet
    onboard_bits::BitSet
    onboard_age_idx::Vector{Int32}
    onboard_age_val::Vector{Float64}
    onboard_age_mask::UInt64
end

"""
Every piece of label state the dominance scan can reject a candidate with
using only scalar comparisons -- see `PricingLabelEntry`'s docstring
(`label_setting/types.jl`). `n_onboard` is `length(onboard)`, i.e. how many
commitments are currently owed.
"""
struct JointRoutingAssignmentDarpDominanceFilters
    reduced_cost::Float64
    time::Float64
    onboard_age_mask::UInt64
    route_length::Int32
    n_onboard::Int32
end

"""
    JointRoutingAssignmentDarpDominanceRules{BoundedStops}

Only one switch -- unlike every other pricer in this package, there is no
`Compensated` parameter: dominance here is unconditionally plain/exact on
both `served` and `onboard` (see this file's module docstring for why a
triple-indexed compensated charge would be unsound without secretly
reconstructing passenger-level bookkeeping).
"""
struct JointRoutingAssignmentDarpDominanceRules{BoundedStops} <: AbstractPricingDominanceRules end

_joint_routing_assignment_darp_dominance_rules(bounded_max_stops::Bool) =
    JointRoutingAssignmentDarpDominanceRules{bounded_max_stops}()
