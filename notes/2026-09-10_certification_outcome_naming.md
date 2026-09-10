# Certification outcome naming: `:refuted` is retired, `:column_found` is the name

**Status 2026-09-10: BOTH RENAMES APPLIED.** `:refuted` -> `:negative_rc_column_found`
(commit 1370222) -> `:column_found` (this change), the second landing the same day after
peer sessions confirmed they were clear of the files. Suite green at 93,107 both times.
This note exists so nobody re-derives the argument a fourth time.

**No run ever wrote the middle name.** Measured at rename time: 0 result CSVs carried
`certification_negative_rc_column_rounds`, 312 carried `certification_refuted_rounds`, and
no Study 9/10 array was in flight (only three unrelated `benders_lpo` jobs). So the schema
is a clean two-way split, old vs new, with no third spelling to reconcile.

## The misunderstanding this is all about

`:refuted` was one of the three outcomes of a relaxed-cluster certification attempt, and it
does **not** mean the attempt failed. The `:relaxed_cluster` / `:relaxed_cluster_two_tier`
mode **prices first and certifies second**:

- the relaxation names a promising cluster route and hands over its support `T`;
- the real exact pricer searches `stations(T)` exhaustively;
- if it finds an improving real column, that column is harvested and **is that iteration's
  pricing round** -- CG adds it and iterates. The regular full-station pricing round is
  skipped entirely, which is the whole speedup;
- if it finds nothing, `T` is **barren**, which licenses a no-good cut, and the cuts are
  what a later attempt certifies from.

MEASURED: 96% of attempts end in the column-finding branch
(`notes/2026-09-06_relaxed_cluster_harvesting_refinement_and_cuts.md`). That is the mode
working as designed, not a 96% failure rate. Calling it "refuted" made it read as failure
and it was misread as failure repeatedly -- in `CGSolver`'s own docstring, in
`benchmarks/lib/cg_benchmark.jl` ("the two failure modes"), in `cg/state.jl`, and worst in
a 2026-09-10 runtime-audit artifact that headed a section "Failures are refuted, not
starved". All of those are now corrected.

## What `refuted_escalated` was (the question that started this)

It was a value of the per-iteration log field `certification_outcome`. One CG iteration can
run **two** certification rounds:

1. `_cg_certification_phase!` (`src/opt/solvers/cg/loop.jl:225`) -- the ordinary round over
   **all** scenarios at `pricing_time_limit_sec`. Logs `certified` / `column_found` /
   `inconclusive`.
2. `_cg_escalate_inconclusive_scenarios!` (`loop.jl:298`) -- fires when any scenario came
   back `:inconclusive`, re-running **only those scenarios** at the longer
   `certifying_pricing_time_limit_sec`. It then **overwrites** `certification_outcome` with
   the `_escalated` variant (`loop.jl:327`/`330`/`333`).

So the suffix means "this is the second, longer, scenario-restricted round's verdict".
`refuted_escalated` therefore meant: *we spent the escalated budget on scenarios that had
been inconclusive, and the longer search turned up an improving column after all.*

That is the **false-negative rescue**, and it is close to the most positive outcome in the
enum after `certified` -- 63% of inconclusive attempts find real columns when re-searched at
a longer budget (seed 45 scenario 2: `0.0 -> -1854.2` at identical duals,
`notes/2026-09-09_per_scenario_escalation_negative_result.md`). Neither `refuted_escalated`
nor rename 1's `negative_rc_column_found_escalated` said that -- the second was 34 characters
that led with the evidence and never named the escalation as the thing that paid off. It is
now **`escalation_found_column`**.

## Rename 1 (applied 2026-09-10, commit 1370222)

`:refuted` -> `:negative_rc_column_found`, plus everything hanging off it:

| old | new |
| --- | --- |
| `:refuted` | `:negative_rc_column_found` |
| `certification_refuted_rounds` | `certification_negative_rc_column_rounds` |
| `cg_certification_refuted_rounds` (metadata key + benchmark CSV column) | `cg_certification_negative_rc_column_rounds` |
| `round_refuted` / `any_refuted` | `round_negative_rc_column` / `any_negative_rc_column` |
| `"refuted"` / `"refuted_escalated"` (log values) | `"negative_rc_column_found"` / `"negative_rc_column_found_escalated"` |

Untouched on purpose: the **feasibility-gate** refutation in `optimize/run_opt.jl`,
`optimize/aggregate_od_route/direct/build_feasibility.jl`, `solvers/utils/common.jl` and
`test/opt/test_solve_status.jl`. There "refuted" genuinely means the instance was refuted
before any solve, and that meaning is correct.

Suite green at 93,107 assertions after this rename.

## Rename 2 (applied 2026-09-10, this change)

Rename 1 fixed the "failure" reading but introduced four problems of its own, which is why
it was superseded within the day.

**(a) The symbol names the witness, not the verdict.** The loop's question is "is `T`
barren?" and the answers are barren / not barren / don't know. The codebase's own vocabulary
for this is already **barren** and **spurious** (`certification/certify.jl`,
`utils/refinement/refine.jl`). `:negative_rc_column_found` names the *evidence* instead, and
every pricer in the package finds negative-reduced-cost columns -- the symbol no longer
locates itself in the certification loop at all.

**(b) "Refuted" was ambiguous in a second direction, which rename 1 did not address.** It
never meant "the relaxation was refuted"; under that reading it is backwards, because the
**barren** branch is the one where the relaxed route is shown to be spurious. It meant "this
attempt to prove optimality was refuted". Some prose still says "refute the relaxation" for
the column branch, which contradicts that. Retiring the word is right; the replacement
should come off the barren/column axis, not the refute axis.

**(c) Duplicate vocabulary.** `RelaxedClusterCertificationResult.improving_found` is the
good short name and was deliberately left alone -- so the boolean is `improving_found` while
the symbol it corresponds to is `:negative_rc_column_found`. Two names for one concept in
adjacent types.

**(d) The counter parses badly.** `certification_negative_rc_column_rounds` reads as "rounds
of negative rc columns" and can be misread as a count of columns, sitting next to
`certification_rounds` and `certification_inconclusive_rounds`. And
`round_negative_rc_column::Bool` is a Bool named like a noun.

### Final naming

`:column_found` for the outcome -- short, says what happened, and pairs naturally with the
barren branch without claiming anything about failure.

| after rename 1 | final |
| --- | --- |
| `:negative_rc_column_found` | **`:column_found`** |
| `certification_negative_rc_column_rounds` | `certification_column_found_rounds` |
| `cg_certification_negative_rc_column_rounds` (metadata + CSV) | `cg_certification_column_found_rounds` |
| `round_negative_rc_column` | `round_found_column` |
| `any_negative_rc_column` | `any_column_found` |
| `"negative_rc_column_found"` (log value) | `"column_found"` |
| `"negative_rc_column_found_escalated"` | `"escalation_found_column"` |
| `"inconclusive_escalated"` | `"escalation_inconclusive"` |
| `"certified_escalated"` | `"escalation_certified"` |

The three escalated values are reordered deliberately: `escalation_found_column` reads as
"the escalation found a column", which is the fact worth logging, rather than "a negative rc
column was found, escalated".

**Two peer sessions independently objected to `:column_found`** (2026-09-10): it drops the
"negative reduced cost" half, which is precisely what makes the column *improving* rather
than merely found, and `certify.jl` leans on that distinction. Suggested alternatives were
`:improving_column` and `:improving_column_found`. `:column_found` was chosen by the user
and is what is in the tree; recorded here because the history of this particular name is a
history of it being misread, so a third rename should not come as a surprise.

**STILL OPEN (problem (c)):** `RelaxedClusterCertificationResult.improving_found` was left
alone in both renames, so the boolean is `improving_found` while the symbol it projects is
`:column_found`. Either rename the field `column_found`, or document it as the boolean
projection of `outcome === :column_found`. Deliberately not bundled in, because it is a
public field of a result struct that the solver, the round driver and every benchmark
analysis read, and it deserves its own change rather than riding along in a naming sweep.

### Sites touched

Symbol construction: `certification/certify.jl:348`,
`certification/two_tier/loop.jl:469`. Dispatch: `certification/round.jl:126`.
Struct fields: `certification/results.jl:133`, `solvers/cg/state.jl:64` and `:119`.
Metadata key: `solvers/cg/metadata.jl:62`. Log values: `solvers/cg/loop.jl:244`, `327`,
`330`, `333`. Verbose print: `optimize/aggregate_od_route/benders/subproblem_cg.jl:352`.
Readers: `benchmarks/lib/cg_benchmark.jl:136`,
`benchmarks/study9_relaxed_cluster_scalability/run_benchmark.jl:249`,
`benchmarks/study10_scenario_count_frontier/run_benchmark.jl:249`,
`benchmarks/diagnostics/relaxed_cluster_certification_probe.jl:149` and `:186`.
Tests: `test/opt/test_joint_routing_assignment_relaxed_cluster_pricing.jl:751`, `779`,
`780`, `1192`, `1206`, `1217`.

Watch two formatting traps that already bit once during rename 1: the `%9s`/`%12s` printf
headers in `benchmarks/diagnostics/relaxed_cluster_certification_probe.jl:122` and
`benchmarks/diagnostics/instance_geometry_vs_certification.jl:146` overflow if a long symbol
name is pasted into them. Both now read `priced` and `uncertified`; a blind
search-and-replace will break the column alignment again.

## Two pre-existing wrinkles this exposed -- NOT caused by either rename

Worth fixing, separately from the naming:

1. **The counters double-increment inside one iteration.** Both `_cg_certification_phase!`
   and `_cg_escalate_inconclusive_scenarios!` do `st.certification_rounds += 1`, and both can
   increment the column counter. So one CG iteration can add 2 to each. The counters count
   *attempts*, not iterations, and nothing in the docstrings says so.
2. **The escalated overwrite loses the ordinary round's verdict.** If the ordinary round
   found columns and the escalated round came back inconclusive, the logged
   `certification_outcome` is `inconclusive_escalated` and the columns are invisible in that
   field -- though `round_negative_rc_column` still ORs them in and the harvest is still
   appended. Consequence:
   `test/opt/test_joint_routing_assignment_relaxed_cluster_pricing.jl:777` asserts one log
   row per attempt (`attempted == cg_certification_rounds`), which holds only because
   escalation never fires in that test instance. That invariant is not true in general.

## Benchmark CSV compatibility

Rename 1 already changed the CSV column written by Study 9 and Study 10's `run_benchmark.jl`
from `certification_refuted_rounds` to `certification_negative_rc_column_rounds`; rename 2
would change it again to `certification_column_found_rounds`. Nothing in any `analyze*.jl`
reads that column, so nothing breaks -- but archives under `benchmarks/experiments/` carry
the old headers, so a script pooling old and new runs by column name must accept all three.
