# BendersY/BendersYZ/BendersYZH/BendersXY: Terminal Return Used the Wrong Incumbent

*2026-07-21*

## Status: FIXED. Root cause was a plain variable-swap bug, not a soundness gap in the cuts or
termination criterion. All 210 tests in
`test/opt/test_aggregate_od_route_nearest_open_alignment.jl` pass with the fix.

## Supersedes a wrong note

This note replaces (the now-deleted) `notes/2026-07-21_benders_lp_relaxation_integrality_gap.md`,
written earlier the same session. That note diagnosed the same symptom (a Zhuzhou instance where
`BendersYZH`/repriced `BendersYZ` reported a worse objective than un-repriced `BendersY`/`BendersYZ`
found) as a deep soundness gap: cuts derived from a continuous-`lambda` LP relaxation of what
should be an integer route-covering subproblem, making the classical Benders stopping criterion
unsound. That explanation was **wrong** — or at least, not what actually caused this symptom. It
was written and documented before checking the one thing that would have falsified it in five
minutes: what station set did the "wrong" run actually converge to, and did its own incumbent
tracking already have the right answer sitting in memory. It did.

## What actually happened

Re-ran `BendersYZH` (repriced) on the same instance and printed both `result.objective_value` and
`result.metadata["benders_incumbent_objective"]`:

```
objective_value              = 9448.031184060867
benders_incumbent_objective  = 9410.311025133655   # matches the "wrong" BendersY run exactly
```

`BendersYZH`'s own search *did* find 9410.31 (at a station set differing from `BendersY`'s by a
single station: `117` vs `133`) and correctly recorded it as `best_ub`/`best_result`, updated every
iteration via:

```julia
if !isnothing(final_result.objective_value) && final_result.objective_value < best_ub
    best_ub = final_result.objective_value
    best_result = final_result
end
```

But the terminal return, reached once `cuts_added_this_iteration == 0`, was:

```julia
return _opt_result_from_benders(final_result, Dict{String, Any}(...))
```

`final_result` is *this iteration's* incumbent -- whichever `y_hat` the master happened to be
sitting on when the loop stopped, not necessarily the best one ever found. The metadata dict
passed alongside it correctly reports `best_ub` under `"benders_incumbent_objective"` (which is
why the right answer was visible in the metadata, just not in the field anyone actually reads),
but the `OptResult` object itself -- `objective_value`, `solution`, `model`, everything downstream
-- was built from `final_result`, silently discarding whatever better incumbent `best_result` was
already holding.

This is not a Benders-theory bug. It's a plain use of the wrong local variable at the return site.

## Why it's easy to miss

It only produces a visibly wrong answer when the *final* iteration's `y_hat` (whatever the master
happens to be examining right as the stopping condition first triggers) differs from the best
`y_hat` visited earlier in the search -- i.e. when the search doesn't end at its own best point.
That's presumably uncommon (Benders search typically improves close to monotonically), which is
why 210 existing alignment tests never caught it -- apparently none of their fixtures happened to
have the final iterate differ from the best one. It reproduced on a specific generated Zhuzhou
instance (`n_stations=20, l=10, n_pairs=16, endpoint_overlap=2.0, seed=42, n_scenarios=3`) during
an unrelated convergence-rate comparison, purely because that instance's search trajectory
happened to end somewhere other than its own optimum.

## Fix

Changed `final_result` to `best_result` in the terminal return at all five call sites (the pattern
is duplicated once per decomposition, plus twice in `BendersXY` for its two run functions):

- `src/opt/optimize/aggregate_od_route/benders/y.jl:698`
- `src/opt/optimize/aggregate_od_route/benders/yz.jl:453`
- `src/opt/optimize/aggregate_od_route/benders/yzh.jl:487`
- `src/opt/optimize/aggregate_od_route/benders/xy.jl:335` and `:473`

`best_result` is guaranteed non-`nothing` by this point (the loop's first iteration always beats
the `Inf` initial `best_ub`, given a finite objective), so no additional nil-handling was needed.

Verified: `test/opt/test_aggregate_od_route_nearest_open_alignment.jl` -- 210/210 pass post-fix.

## Lesson for next time

Don't write up a "root cause" note before checking the cheapest disconfirming test available. Here
that test was: pull the actual `y_hat`/station set the "wrong" run converged to and diff it against
the "right" run's -- a five-minute check that immediately would have shown `best_ub` already had the
right number in `result.metadata`, pointing straight at the return-value bug instead of a purely
theoretical soundness argument about LP relaxations. The LP-relaxation-vs-integer distinction
described in the retracted note is still a real mathematical fact about these subproblems, but it
is not established to cause any observed symptom in this codebase -- don't reuse that reasoning
without re-deriving and re-verifying it from scratch.
