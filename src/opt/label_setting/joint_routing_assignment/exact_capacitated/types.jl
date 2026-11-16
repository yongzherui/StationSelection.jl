"""
`:exact_capacitated` -- the exact joint routing+assignment pricer with a vehicle capacity,
solved by relaxing the capacity on all but a growing set of *critical* riders.

# Relaxation

Pick a set `S` of riders (the critical riders). The relaxed pricing problem counts only
`S`'s riders against the capacity: riders outside `S` weigh nothing. Every route's best
relaxed reward is therefore at least its best capacitated reward, so the relaxed reduced
cost is a lower bound on the capacitated one, route by route. With `S` empty the relaxed
problem is `:exact`'s; with `S` = everyone it is the capacitated problem itself.

The search treats the two kinds of rider differently:

  - a rider outside `S` is credited exactly as `exact/` does: retroactively at a dropoff
    stop, through the running-max reward layers, and upgraded whenever a later stop pays
    more. No branching.
  - a rider in `S` carries load. A search action is a next stop together with the set of
    `S` riders credited there (each through one of its live clocks); a credit is fixed
    once made, and the credited rider's whole reward-layer mask is marked active.

# Capacity state (riders in `S` only)

Write `F(τ)` for the most `S` riders on board on any arc driven in the last `τ` seconds. `F`
is nondecreasing, `F(0) = 0`, and the label stores it at the age of every live clock:
`station_load[j] = F(station_age[j])`. Crediting rider `p` (load `Q_p`) through clock `j`
needs `F(age_j) + Q_p <= capacity` and then

  - clocks no older than `j` (`age_c <= age_j`): `F(age_c) += Q_p`;
  - older clocks: `F(age_c) = max(F(age_c), F(age_j) + Q_p)`.

# Dominance (proof at `_capacitated_dominates`)

`a` may discard `b` at the same stop if `a` is no later, uses no more stops (when bounded),
every live clock of `b` is live in `a`, no older, with `F_a(age_a) <= G_b(age_a)` (`G_b(τ)`
the largest stored load of a `b` clock aged at most `τ`), and `exact/`'s compensated
reward-layer test passes: `rc_a + w(layers_a ∖ layers_b) <= rc_b`. An `S` rider's credit
activates its full mask, whose weight is its largest reward, so the one layer test covers
both kinds of rider.

# Columns and the loop that grows `S` (`search.jl`, `_run_label_setting`)

A finished label's route is re-solved exactly with every rider's load counted
(`_capacitated_route_optimum`), and that capacitated optimum is the column. A relaxed search
that exhausts with no relaxed reduced cost below `-tol` proves no capacitated column prices
negative. Otherwise, if the relaxed optimum is not matched by a capacitated route, the
riders on over-capacity arcs of the best relaxed routes join `S` and the search reruns.
`S` only grows (each pass adds at least one rider), so the loop ends, and it is kept
between pricing calls (per scenario, on the model), so later CG iterations start from it.

By default every rider is in `S` from the start (`initial_critical = :all`): on the
instances tried (Zhuzhou config_01, k = 5) the relaxation with `S` empty was tens of
thousands below the capacitated optimum and `S` grew to almost everyone anyway, so there is
one pass. `initial_critical = :empty` keeps the growing loop.

# What keeps the search small (measured on config_01, k = 5, 2026-11-16)

  - pruning: a label whose reduced cost minus its remaining bound is not below
    `rho * (best capacitated reduced cost found)` (and `-tol`) is neither inserted nor
    extended (`rho = 1` default: each round returns the most negative column and the
    columns found on the way; `rho = 0` keeps every negative column and is far slower);
  - the remaining bound (`_capacitated_remaining_bound`): each rider is worth its reward
    minus `(w / C) * Q * ride` (future drive time carries at most `C` per arc), bounded by
    fractional knapsacks on ride time, on stops, and by when the rider rides: riders credited
    later through a clock visited before a live clock `c` ride every arc since `c`, so they
    fit in `C - F(age_c)`; riders boarding later either finish inside the pickup window or
    are aboard when it closes (at most `C`);
  - after the pickup window a stop that credits nobody is never needed when travel times
    are shortest paths (`pd.metric`), so it is not generated;
  - per next stop, every subset of riders credited there is scored from one scan of the
    child that credits nobody (`_capacitated_scan!`), and only subsets that can still beat
    the threshold become labels;
  - dominance reads `F` at every arc start (`arc_load`), not only at clock ages.
"""

export JointRoutingAssignmentCapacitatedPricingData, JointRoutingAssignmentCapacitatedPricingLabel

"""Pricing data: `exact/`'s data for the same candidates, plus the capacity, per-rider loads
(`Q_p`, requests in the group), each rider's largest reward and the union of its reward
layers, and `critical[p]` (rider `p` is in `S`; shared and grown by the search loop)."""
struct JointRoutingAssignmentCapacitatedPricingData
    base::JointRoutingAssignmentPricingData
    capacity::Int
    load::Dict{Int, Int}
    max_reward::Dict{Int, Float64}
    full_mask::Dict{Int, RewardLayerBitset}
    opportunity::Dict{Tuple{Int, Int, Int}, PassengerAssignmentOpportunity}
    critical::Vector{Bool}
    # Stop growing S after the first pass that finds a capacitated column priced below
    # -tol (column generation needs some improving column, not the most improving one).
    stop_at_first_column::Bool
    # How many of the best relaxed routes (by relaxed reduced cost) feed S per pass.
    growth_routes::Int
    # shortest-path closure of the travel times over `base.nodes` (a lower bound on any
    # drive), its node numbering, and each rider's opportunities
    sp::Matrix{Float64}
    node_idx::Dict{Int, Int}
    opps_by_p::Dict{Int, Vector{PassengerAssignmentOpportunity}}
    metric::Bool
end

_is_critical(pd::JointRoutingAssignmentCapacitatedPricingData, p::Int) =
    p <= length(pd.critical) && @inbounds pd.critical[p]

"""An `S` rider's fixed credit: `(p, j, k, pickup position, dropoff position)` into `route`."""
const CapacitatedCredit = Tuple{Int, Int, Int, Int, Int}

struct JointRoutingAssignmentCapacitatedPricingLabel
    current::Int
    route::Vector{Int}
    time::Float64
    station_age::Dict{Int, Float64}
    station_load::Dict{Int, Int}
    station_pos::Dict{Int, Int}
    activated_reward_layers::RewardLayerBitset   # non-critical layers + credited critical riders' full masks
    noncrit_layers::RewardLayerBitset            # the non-critical riders' layers alone
    credited::Vector{CapacitatedCredit}
    credited_set::BitSet
    tau::Float64
    reduced_cost::Float64
    route_length::Int
    # time at each route position, and the critical load on each arc (arc i: position i -> i+1).
    # `station_load[c] == maximum(arc_load[station_pos[c]:end])`; the arcs give `F` at every
    # arc start, which dominance reads (`_capacitated_load_lower_bound`).
    pos_time::Vector{Float64}
    arc_load::Vector{Int}
end

"""A search action: drive to `node` and credit the `S` riders `credits` there, each
`(p, j, reward)`."""
struct JointRoutingAssignmentCapacitatedAction
    node::Int
    credits::Vector{Tuple{Int, Int, Float64}}
    # the child, when the subset search already built it (the probe that credits nobody here)
    child::Union{Nothing, JointRoutingAssignmentCapacitatedPricingLabel}
end
JointRoutingAssignmentCapacitatedAction(node::Int, credits::Vector{Tuple{Int, Int, Float64}}) =
    JointRoutingAssignmentCapacitatedAction(node, credits, nothing)

"""Dominance mirror. `age_idx`/`age_val` are `exact/`'s sparse node-indexed ages (read by its
remaining-reward bound) and `load_val` the matching clock loads; `ages_sorted`/`loads_sorted`
hold the same clocks in increasing age with prefix-max loads for `G_b` lookups.
`max_load == 0` (no `S` rider aboard on any live arc) lets dominance skip the load walk."""
struct JointRoutingAssignmentCapacitatedLabelBitsets
    noncrit_bits::RewardLayerBitset
    credited_set::BitSet
    # uncredited critical riders this label can still credit (some option passes the
    # pickup-window / ride-limit / load tests from the label's state) => the largest reward
    # among those options; see `_capacitated_dominates`
    reach::Vector{Float64}   # indexed by rider; 0 = cannot be credited any more
    # the label's remaining-reward bound (`_capacitated_remaining_bound`), computed with `reach`
    bound::Float64
    age_idx::Vector{Int32}
    age_val::Vector{Float64}
    age_mask::UInt64
    load_val::Vector{Int}
    ages_sorted::Vector{Float64}
    loads_sorted::Vector{Int}
    max_load::Int
end

struct JointRoutingAssignmentCapacitatedDominanceFilters
    reduced_cost::Float64
    time::Float64
    age_mask::UInt64
    route_length::Int32
    n_live_ages::Int32
end

struct JointRoutingAssignmentCapacitatedDominanceRules{BoundedStops, Compensated} <: AbstractPricingDominanceRules end

const CapacitatedBestSignature = Tuple{RewardLayerBitset, Vector{Int}}

"""Index for the capacity-aware remaining-reward bound (`_capacitated_remaining_bound`).
Opportunity `i` is `pd.base.opportunities[i]` (the search index's numbering). The
critical/non-critical split is refreshed at the start of every pass of the loop growing `S`;
`sp` is the shortest-path closure of the travel matrix (a lower bound on any drive)."""
mutable struct CapacitatedBoundIndex
    opp_p::Vector{Int}
    opp_reward::Vector{Float64}
    opp_q::Vector{Int}
    opp_origin_idx::Vector{Int}
    sp::Matrix{Float64}
    opp_critical::Vector{Bool}
    crit_opps_by_origin_idx::Vector{Vector{Int}}
    noncrit_origin_union_mask::Vector{RewardLayerBitset}
    # per-rider scratch (index p), reset after each use
    v_best::Vector{Float64}
    v_wmin::Vector{Float64}
    v_live::Vector{Float64}
    v_later::Vector{Float64}
    v_later_a::Vector{Float64}
    v_later_aw::Vector{Float64}
    touched::Vector{Int}
    v_live_old::Vector{Float64}   # live clock visited before this stop (rode the arc into it)
    v_live_new::Vector{Float64}   # the clock opened at this stop
    # live-clock options (rider, clock age, value) and the live clocks (age, load), for the
    # per-clock capacity splits in `_capacitated_compose!`
    lrec_p::Vector{Int}
    lrec_age::Vector{Float64}
    lrec_v::Vector{Float64}
    slot_age::Vector{Float64}
    slot_load::Vector{Int}
    v_tmp_old::Vector{Float64}
    v_tmp_new::Vector{Float64}
    kv::Vector{Float64}
    kw::Vector{Float64}
    kq::Vector{Float64}
    kv2::Vector{Float64}
    kw2::Vector{Float64}
    korder::Vector{Int}
end

"""Scratch for `_capacitated_scan!` / `_capacitated_scan_eval!` (one per search context)."""
mutable struct CapacitatedScan
    p::Vector{Int}
    slot::Vector{Int}
    q::Vector{Int}
    v::Vector{Float64}
    w::Vector{Float64}
    cls::Vector{UInt8}   # 1 live clock, 2 later clock, 3 later clock finishing by tstar
    now::Float64
    horizon_end::Float64
    tstar::Float64
    route_length::Int
    slot_age::Vector{Float64}
end
CapacitatedScan() = CapacitatedScan(Int[], Int[], Int[], Float64[], Float64[], UInt8[], 0.0, 0.0, 0.0, 0, Float64[])
