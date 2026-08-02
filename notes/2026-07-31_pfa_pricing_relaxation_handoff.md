# PFA pricing-relaxation exploration: handoff

Date: 2026-07-31. This note is the context-reset entry point for the passenger
free-assignment (PFA) pricing work. Detailed derivations remain in:

- `notes/2026-07-31_passenger_mcf_relaxed_pricer.md`
- `notes/2026-07-31_pfa_state_space_relaxation_design.md`
- `notes/2026-07-31_passenger_path_relaxation_roadmap.md`
- `notes/2026-07-31_two_stop_seeding_and_bound_trajectory.md`

## Current recommended/default method

Use **station-simple warm start followed by exact revisit-tolerant pricing**.
`run_passenger_free_assignment_column_generation` now defaults to
`station_simple_warm_start=true`. The station-simple phase harvests elementary
columns; after exhausting that restricted universe, CG switches to the exact
revisit-tolerant pricer. Only the exact phase may certify the unrestricted PFA
pricing problem. Set `station_simple_warm_start=false` for a pure-exact control.

On n=20/p=16, seed 42, three scenarios, max_stops=5:

| method | wall | certified unrestricted optimum? |
| --- | ---: | --- |
| station-simple only | 98.3s | no; result was 0.419% weaker |
| reward-coarsened L=2 + exact tail | 356.2s | yes |
| pure exact revisit | 513.1s | yes |
| station-simple warm start + exact | **198.4s** | yes, **2.59x** vs pure exact |

## Relaxations explored

### Time-expanded multicommodity flow: rejected

Valid lower bound, 0 false certificates, but 0% useful certification. At n=10
it was about 200x slower than label search and remained 1500-1850 reduced-cost
units below zero at exact convergence. Fractional vehicle flow collected several
routes' reward while paying roughly one route's cost. The implementation remains
off by default in `pricing/passenger/mcf_relaxation.jl` as a reference.

### Reward-ladder coarsening: valid harvester, mixed CG value

`reward_coarsening_levels=2` rounds assignment rewards upward, searches with the
relaxed ladder, and exactly replays every returned route. Certification is always
exact. Microbenchmarks showed 2.3x exhaustive-search speed and about 0.9% mean
column-quality shortfall at n=20, but full-CG results were size-dependent:

- n=10: 0.84x exact pricing speed;
- n=15: 0.76x;
- n=20: 1.44x end-to-end wall improvement, but 6644 columns versus 2512 exact.

It is kept opt-in, not default. It is a valid route harvester, not a certificate.

### Passenger Lagrangian uniqueness: rejected

Keeps one physical route but allows independent assignment alternatives and
dualizes at-most-one assignment per passenger. Multiplicity 4-6 confirmed that
passenger uniqueness is a real missing coupling. Five multiplier rounds were
still 1563-1974 reduced-cost units below exact at n=10 and cost about 6-10x an
exact search after warm-up. Best relaxed routes also replayed poorly. Reference:
`pricing/passenger/lagrangian_relaxation.jl`.

### Passenger-specific DSSR: rejected for acceleration

Noncritical assignments are independent pseudo-passengers; passengers duplicated
by the relaxed best route are promoted into exact state. It is valid, monotone,
finite, and reached exact optima in all tests, but typically promoted 6-7 of the
8 active passengers. At n=20 the no-initial-state variant took 123-291s versus
2.7-8.9s exact. Starting with eight exact passengers was simply the exact pricer.
Reference: `pricing/passenger/passenger_dssr.jl`.

### Exact post-W destination-only completion: rejected as a hot bound

After the pickup cutoff, an exact elementary destination suffix required only
about 2.6-4.8 states per sampled label and microseconds per query. It was safe and
cheap, but did not change enough search:

- n=15 warm comparisons were neutral overall;
- n=20 total wall was 16.60s exact versus 16.48s with the bound (1.01x);
- label reductions were below 2%;
- sampled n=20 tightening was effectively zero.

Keep `pricing/passenger/post_w_completion.jl` as an exact micro-oracle/reference,
but leave `use_post_w_completion_bound=false`.

## Validation state

- Passenger pricing tests: 587/587 pass.
- Two-stop/default-CG tests: 292/292 pass.
- Relaxation implementations are opt-in or disconnected from CG except the new
  station-simple-warm-start default.
- Runtime invariants exactly replay relaxed routes and check relaxation direction.

## New focus after context reset: station-subset enumeration in the CG tail

The next idea is **not another route-level reward relaxation**. Smartly enumerate
candidate station subsets in late CG iterations and use them to prove that no
negative-reduced-cost route exists, avoiding exhaustive label pricing.

The proof obligation must be explicit. For current duals and scenario `s`, let
`R(S)` be the best feasible route whose distinct visited-station set is contained
in subset `S`. A tail certificate needs a family `F` such that either:

1. every feasible improving route is contained in at least one `S in F`, and
   every `R(S)` is solved/certified non-improving; or
2. excluded subsets/classes have a valid lower bound on reduced cost that is
   nonnegative.

Merely enumerating promising subsets and finding no column is not a certificate.
The design must cover or validly bound the omitted station sets.

### Recommended first investigation

1. Measure the distinct-station sets of all columns generated and selected in
   converged/tail iterations. Existing evidence says selected routes use roughly
   2-7 distinct stations and only 3-7 columns are selected.
2. Build a cheap subset lower bound from station-level collectible passenger
   reward minus a valid minimum route cost for visiting that subset.
3. Enumerate subsets best-bound-first; solve the within-subset pricing problem
   exactly (the station budget is now fixed and small).
4. Maintain a global lower bound for every unenumerated subset class. Stop with
   a proof only when both enumerated and unenumerated classes are non-improving.
5. Compare against exact certification on saved tail dual snapshots at n=15/20/30:
   certificate agreement, subsets expanded, total labels, and wall time.

Potential representations to test incrementally:

- fixed cardinality layers `|S|=2,3,...` with admissible subset bounds;
- best-first branch-and-bound over include/exclude station decisions;
- passenger-induced candidate sets (union of attractive origins/destinations),
  with a valid bound for stations omitted from the candidate core;
- reuse/warm-start subset bounds across adjacent CG iterations;
- exact route pricing restricted to `S`, with dominance signatures localized to
  the much smaller node universe.

The first deliverable should be a tail-snapshot diagnostic and correctness oracle,
not a CG integration. Pre-register rejection criteria: zero false certificates,
and retain only if total subset-bound plus restricted-pricing wall beats the exact
certification pass consistently at n=15/20 before scaling to n=30.

## Working files introduced or changed in this exploration

- `src/opt/optimize/aggregate_od_route/pricing/passenger/data.jl`
- `src/opt/optimize/aggregate_od_route/pricing/passenger/column_generation.jl`
- `src/opt/optimize/aggregate_od_route/pricing/passenger/lagrangian_relaxation.jl`
- `src/opt/optimize/aggregate_od_route/pricing/passenger/passenger_dssr.jl`
- `src/opt/optimize/aggregate_od_route/pricing/passenger/post_w_completion.jl`
- `src/opt/optimize.jl`
- `scripts/diag_passenger_station_simple_vs_revisit_objective.jl`
- `scripts/diag_passenger_lagrangian_gap.jl`
- `scripts/sbatch_diag_passenger_post_w.sh`
- `test/opt/test_passenger_free_assignment_pricing.jl`
- `test/opt/test_passenger_free_assignment_seeding.jl`

Do not infer that these working-tree changes have been committed. Inspect
`git status` before continuing and preserve unrelated pre-existing files.
