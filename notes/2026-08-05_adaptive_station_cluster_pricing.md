# Adaptive station-cluster pricing lower bound

`solve_adaptive_cluster_lower_bound` wraps the passenger free-assignment label
pricer with a nested geographic partition.  The exact reduced-cost convention is

`beta * (route travel + repositioning) - sum(best passenger reward)`.

For clusters `A,B`, routing uses a componentwise minimum over `A x B`, passenger
reward uses a maximum, and the ride allowance uses a maximum.  These extrema may
have different witnesses.  The label never chooses a physical station inside a
cluster; entering, picking up, collecting reward, and leaving may therefore use
different hidden stations.  A non-singleton intra-cluster opportunity uses a
zero-or-optimistic-cost shadow visit to retain this property.

Thus mapping a physical route to its cluster sequence never increases routing
consumption, never lowers its available reward, and never tightens a ride-time
allowance.  Every exact route has a feasible relaxed image of no greater reduced
cost, proving

`LB_cluster <= cbar_exact`.

An exhausted cluster search with `LB_cluster >= -tol` certifies that exact pricing
has no improving column.  A negative bound is only guidance.  A timed-out search
returns `-Inf`, because the best relaxed route found so far is an upper bound on
the relaxed optimum and cannot certify the exact problem.

Splits use only a parent's stations and preserve unchanged aggregate cache
entries.  They remove hidden station combinations, so exact refined bounds obey
`LB_K <= LB_K+1 <= cbar_exact`; a decrease beyond numerical tolerance emits a
warning.  The hard cluster budget is checked before every binary split.  The
partition persists when rewards are refreshed with `refresh_cluster_rewards!`;
resetting is explicit, or enabled by `reset_refinement_each_cg_iteration=true`.

The optional `exact_pricer(route, assignments)` callback receives cluster
guidance.  It may prioritize cluster members and witness stations, but a
certifying exact search must retain all physical stations.  If the callback finds
a negative exact route, refinement stops with `FoundExactNegativeColumn`.
