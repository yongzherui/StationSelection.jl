"""
The dominance rejection census: off-by-default instrumentation counting which
condition in `dominate.jl`'s `_pricing_dominates_at_state` was the first to
reject each tested pair. Never touched by production pricing -- it exists
purely to inform that predicate's condition ordering (cheapest-and-likeliest-
to-reject first is only meaningful against measured rejection rates, and this
is where those come from) -- see `julia scripts/diagnose.jl dominance_audit`.

The counters are declared here, but every increment lives inline in
`dominate.jl`'s `_pricing_dominates_at_state`, guarded by its `Instrumented`
type parameter: moving the increments out here would defeat the point --
with `Instrumented = false` (every production search), the guard is
constant-folded away, so an uninstrumented scan pays nothing for their
existence. This file holds the counters and the reader that turns them into
a report; `dominate.jl` is what actually counts.
"""

# ── dominance rejection census (diagnostics) ─────────────────────────────────
"""
Rejection census for `_pricing_dominates_at_state` (passenger method), one counter
per condition plus one for "dominates".

Only written when the dominance rules carry `Instrumented = true`, which the
production search never sets; with `Instrumented = false` the increments are
constant-folded away, so an uninstrumented scan pays nothing for their existence.
Ordering the conditions cheapest-and-likeliest-to-reject first is only meaningful
against measured rejection rates, and this is where those come from -- see
`julia scripts/diagnose.jl dominance_audit`.
"""
const JOINT_ROUTING_ASSIGNMENT_DOMINANCE_CONDITIONS = (
    :time, :live_clock_support, :route_length,
    :reduced_cost, :station_age, :compensation, :dominates, :age_mask,
)
const JOINT_ROUTING_ASSIGNMENT_DOMINANCE_REJECTIONS = zeros(Int, length(JOINT_ROUTING_ASSIGNMENT_DOMINANCE_CONDITIONS))

# Named indices keep the instrumented hot loop readable while retaining the
# zero-allocation integer indexing used by the counters.
const JRA_REJECT_TIME = 1
const JRA_REJECT_LIVE_CLOCK_SUPPORT = 2
const JRA_REJECT_ROUTE_LENGTH = 3
const JRA_REJECT_REDUCED_COST = 4
const JRA_REJECT_STATION_AGE = 5
const JRA_REJECT_COMPENSATION = 6
const JRA_DOMINATES = 7
const JRA_REJECT_AGE_MASK = 8

"""
Read out and reset the rejection census. Returns
`condition => count` pairs in evaluation order.
"""
function joint_routing_assignment_dominance_rejections(; reset::Bool=true)
    counts = [JOINT_ROUTING_ASSIGNMENT_DOMINANCE_CONDITIONS[i] => JOINT_ROUTING_ASSIGNMENT_DOMINANCE_REJECTIONS[i]
              for i in eachindex(JOINT_ROUTING_ASSIGNMENT_DOMINANCE_CONDITIONS)]
    reset && fill!(JOINT_ROUTING_ASSIGNMENT_DOMINANCE_REJECTIONS, 0)
    return counts
end
