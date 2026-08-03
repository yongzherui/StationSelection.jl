# Nearest Open pricer: PFA mechanical optimisation port (2026-08-03)

This carries the semantics-preserving changes documented in
`2026-08-03_pfa_pricer_mechanical_optimization_pass.md` into the aggregate
OD-route label-setting pricer used by the Nearest Open Policy.

The applicable changes are: sparse sorted station-age mirrors and their `UInt64`
support prefilter; cheapest-first dominance checks; scalar state inline in vector
dominance buckets; a type-specialized bounded-stop switch; vector-backed live
labels; direct single-child extension; concretely typed signature storage; reuse
of the priority returned by the queue; and construction of route-string sort keys
once per harvested column. PFA reward-layer compensation and assignment replay
do not exist in this pricer and were therefore not ported.

The focused pricing suite passes all 83 tests that do not require a solver. Five
integration testsets are currently blocked before model creation because the
configured Gurobi token server is unreachable.

The following commit is intentionally a separate refactor that extracts the
mechanical primitives shared by the aggregate and PFA pricers. Keeping the port
separate makes semantic review and performance bisecting possible.
