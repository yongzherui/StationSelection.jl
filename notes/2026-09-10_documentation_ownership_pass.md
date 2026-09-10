# Documentation ownership: one owner per fact, and it caught two real errors

2026-09-10. A readability pass over `src/` turned into a correctness pass. Recording the
criterion and the evidence, because the criterion is cheap to apply and the evidence is
stronger than a readability argument.

## The measurement that started it

`src/` carries **13,345 lines of code against 11,994 lines of prose** (2,033 `#` comments +
9,961 docstring lines), a 0.90 prose:code ratio -- plus 31 KB of `README.md` *inside*
`src/`, plus `CLAUDE.md`, plus ~40 notes. Four layers.

No layer owned any given fact. The measurement "a cut-free round certified 0 times across
~1130 attempts" existed in **seven places**: `CLAUDE.md`, `opt/optimize.jl`,
`label_setting/README.md`, `relaxed_cluster/relaxation.jl`, `relaxed_cluster/README.md`,
`cg/pricing_config.jl`, `cg/solver.jl`.

## The criterion

**A struct earns only documentation that describes that struct**, judged by how it reads as
a rendered docs page rather than as a source comment. Concretely:

| layer | owns |
| --- | --- |
| `notes/` | every measured number, with instance, seed and job id |
| `CLAUDE.md` | the map: what is live, what pairs with what, where it lives |
| a docstring | that unit's contract -- what it does, invariants, failure modes. Cites a note for a number rather than restating it |
| `src/**/README.md` | how a subsystem's files fit together |

Test: a number appears once and everything else cites it.

## It is a correctness criterion, not a readability one

Two errors were found by *relocating* text, and neither would have been found by shortening
it.

**1. A validity proof parked on a config struct had a false premise, and it survived a
year.** `BendersSubproblemConfig` carried the "activated subproblem and its dual completion"
section -- 92 lines proving a property of `_benders_activated_complete_duals!`, a function
in a different file. Its step 3 asserted the activated restriction "needs the route universe
left UNRESTRICTED -- only candidates shrink". That is false: candidate generation is
reward-driven, so driving unbuilt-station candidates to `rho <= 0` removes those stations
from the searched *route* universe too. The conclusion still holds, via the
triangle-inequality shortcut the same docstring said it deliberately avoided.

It survived because **nothing anyone did forced them to read it.** A pricer change does not
open a config struct. The false premise and the true one support the same conclusion, so
nothing downstream ever failed.

**2. Moving that proof next to its function immediately exposed a second gap.** The
`max_wait_time` argument showed both left-hand sides decreasing under shortcutting and never
said the right-hand sides were unchanged -- a real step for the ride limit, since
`routing_cost(j,k)` reads as route-dependent and is not (`j,k` are the *assignment's*
endpoints, computed per candidate in `pricing_round.jl` before any route exists). The gap
had survived several re-readings by its own author on the config struct and fell out on the
first reading next to the code.

**That is the mechanism.** A proof parked away from its code gets skimmed; one sitting on the
function it constrains gets read by whoever is about to change that function.

## Two more instances of the same class

- **Docstrings naming deleted models.** 12 `Used by:` lines in `variables/`,
  `constraints/` and `objectives/` named `TwoStageSingleDetourModel`, `TwoStageODPolicy` and
  `SingleStagePolicy` -- gone in the Problem/Formulation/Solver rewrite. `TwoStageODPolicy`
  and `SingleStagePolicy` were not even in `CLAUDE.md`'s removed-names list, so a reader had
  nothing to resolve them against. These are the files someone reads to learn which
  formulation composes which block.
- **Five scaffolding structs whose stated purpose had evaporated.**
  `formulations/aggregate_od_route/benders/{y,xy,yz,yzh,yx}.jl` each described itself as "the
  formulation-level counterpart to the `BendersY`/`XY`/`YZ`/`YZH` decomposition marker" --
  and those solver-level markers were already deleted. Every surviving mention of them was
  inside those five files. Deleted (`17f4184`); they decomposed the *Base* formulation, and
  the live Joint+Benders pair is untouched.
- **A constant's value restated in another file's prose.**
  `two_tier/tuning.jl` said "the default cut-round cap of 65". That number was
  `RELAXED_CLUSTER_MAX_CUTS + 1`, an identity that held only while every cut-adding round
  spent a mask bit for good; cut management reclaims subsumed bits, so the round cap is now
  set on its own terms (96). Fixed by NAMING the constant rather than quoting a value -- the
  only form that cannot go stale. (First attempt at this fix was wrong: it read the 65 as a
  stale copy of the same constant and conflated two genuinely different quantities.)

- **Six files asserting that a live feature does not exist -- the worst instance found.**
  The barren-support cache and cut management were removed as unnecessary at a measured load
  of 0.5-0.75 cuts per attempt, then *restored* (3767740) when n=40 seed 42 began exhausting
  all 64 mask bits and reporting `:cut_mask_full`. The removal was documented in six places
  and the restoration in none, so `CLAUDE.md`, `label_setting/README.md`,
  `cg/pricing_config.jl`, `study9/README.md` and both studies' `run_benchmark.jl` all told a
  reader the code was absent -- while `_relaxed_cluster_add_cut!` was reclaiming mask bits on
  every call. `certify.jl`'s module docstring said "both were removed" three files from its
  own `results.jl` sibling saying "they are back".

  This is the failure mode that costs the most: a reader who believes a feature is absent does
  not go looking for the code, and the two benchmark assertions even carried the message "cut
  management no longer exists" while correctly guarding a column that must be false *because
  the switch* is gone. Six copies of a claim is six chances to miss one; the fix is that the
  claim have one owner.

## A docs site makes the readability half measurable

`docs/` builds a Documenter site (`sbatch scripts/sbatch_build_docs.sh`, ~45 s warm).
Rendered page weight is a direct readout of docstring bloat that reading source does not
give: `pricing.html` came out at 156 KB against `solvers/direct.html`'s 8 KB.

**But page weight cannot distinguish bloat from correctly-placed ownership.**
`certification.html` is large *because* `certify.jl` is now the designated owner of the
relaxed-cluster mechanism. Weight flags candidates; only reading decides.

The build also found 75 docstrings that rendered on no page at all, including the whole of
`export_variables.jl` (975 lines, the largest file in the package) -- documentation that
existed and was unreachable. `checkdocs=:exports` is what reports that, and it is the reason
to keep it on.

## Still open

- The remaining four-layer duplication. Only the CG/relaxed-cluster cluster was deduplicated;
  `label_setting/README.md` (16 KB) and the seven-way `~1130` restatement are untouched.
- `AggregateODRouteFeasibilityFormulation` is live (own `build_model`, powers
  `check_feasibility`/`SOLVE_INFEASIBLE`) and appears **0 times** in `CLAUDE.md`, which says
  "6 + 2 derived". It is 7 + 2, and the type is unexported.
- `Manifest.toml` carries a stale `MbedTLS_jll` entry, dropped from the stdlib in Julia 1.12.
  Nothing in the package calls `Pkg.dependencies()`, so 93,107 passing tests never noticed;
  Documenter's source-link resolution does, and fails. Wants a deliberate `Pkg.resolve()`
  when no arrays are in flight.
