SUPERSEDED 2026-08-25.

These runs used AggregateODRouteBaseFormulation for BOTH arms -- Base+CGSolver as
"cg_exact" vs Base+DirectMIPSolver as "enumeration". That compared a solver against
itself on a formulation nobody uses under CG, rather than the intended comparison of
Joint+CG (the production CG path) against Base+Direct (the enumeration baseline).

They are also affected by the Base+CG livelock fixed the same day
(notes/2026-08-25_study6_cg_livelock_stale_tau_columns.md): 5 of 30 cg_exact runs spun
to max_iterations=1000, and n15_p16_s3_seed45_ms4 returned an objective 0.2% worse than
enumeration while claiming optimality.

Kept only as the evidence trail for that bug. Do not use for any result.
