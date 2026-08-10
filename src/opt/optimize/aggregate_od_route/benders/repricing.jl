"""
    _solve_benders_subproblem_lp_with_repricing(data, model, mapping, columns, optimizer_env,
        silent, decomposition_name; build_lp, extra_checks!, max_reprice_rounds) -> (v_hat, rho,
        pool, n_new_columns_total, rounds, fully_exhausted, max_objective_delta)

Generic repricing certification loop shared by every decomposition's subproblem
(`BendersY`/`BendersYZ`/`BendersYZH` -- `BendersXY` has no repricing companion, its subproblem
fixes `x` fully and is structurally immune to the gap this closes): after each LP solve, extract
the fixing-constraint duals and the coverage-row duals, run genuine label-setting pricing against
them, and if pricing finds any column with negative reduced cost, that pool was not actually
complete for this subproblem's own dual structure -- fold the new columns in and re-solve,
repeating until an exhaustive pricing pass finds nothing more (mirroring standard column
generation's own `cg_stop_reason == :optimality_proven` convergence). This is a certification
check for dual-basis degeneracy, not a corrective loop for an underpriced LP: the caller's own
priming solve already established the objective this subproblem must attain, and an improvement
here (rather than an unchanged value under a possibly different dual basis) indicates a defect,
not a fix -- hence the tight `objective_delta` tolerance check every round after the first.

`build_lp(pool) -> (m, fix_cons, cover_cons, extra)` captures everything specific to one
decomposition's subproblem shape (which variable is fixed, at what value, over which requests) --
`extra` is whatever the caller's `extra_checks!` needs from this round's build that isn't `m`
itself (e.g. `BendersY`'s `x` dict, for `_assert_x_matches_nearest_open`; `nothing` for
decompositions with no extra check). `extra_checks!(m, extra)` runs immediately after a
successful solve, before duals are extracted -- defaults to a no-op.
"""
function _solve_benders_subproblem_lp_with_repricing(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    mapping::AggregateODRouteMap,
    columns::Vector{AggregateODRouteColumn},
    optimizer_env,
    silent::Bool,
    decomposition_name::AbstractString;
    build_lp,
    extra_checks! = (m, extra) -> nothing,
    max_reprice_rounds::Int=10_000,
)
    pool = copy(columns)
    v_hat = NaN
    baseline_v_hat = nothing
    max_objective_delta = 0.0
    rho = nothing
    n_new_columns_total = 0
    rounds = 0
    fully_exhausted = false
    for round in 1:max_reprice_rounds
        rounds = round
        m, fix_cons, cover_cons, extra = build_lp(pool)
        optimize!(m)
        primal_status(m) == MOI.FEASIBLE_POINT ||
            throw(ArgumentError("$decomposition_name repricing subproblem LP failed with status $(termination_status(m))"))
        extra_checks!(m, extra)
        v_hat = objective_value(m)
        if isnothing(baseline_v_hat)
            baseline_v_hat = v_hat
        else
            objective_delta = abs(v_hat - baseline_v_hat)
            max_objective_delta = max(max_objective_delta, objective_delta)
            objective_delta <= 1e-6 * max(1.0, abs(baseline_v_hat)) || throw(ArgumentError(
                "$decomposition_name repricing changed subproblem objective: before=$(baseline_v_hat), " *
                "after=$(v_hat), delta=$(objective_delta). Repricing is expected to certify the " *
                "same LP value, not improve it."
            ))
        end
        rho = Dict(key => dual(con) for (key, con) in fix_cons)

        duals = _extract_nearest_open_y_subproblem_coverage_duals(cover_cons)
        all_new_columns, pricing_exhausted = _price_aggregate_od_route_subproblem_columns(
            data, model, mapping, pool, duals,
        )
        if isempty(all_new_columns)
            if pricing_exhausted
                fully_exhausted = true
                break
            end
            round == max_reprice_rounds && throw(ArgumentError(
                "$decomposition_name repricing did not exhaust pricing within max_reprice_rounds=$(max_reprice_rounds); " *
                "no cut can be certified from the current duals."
            ))
            continue
        end
        pricing_exhausted ||
            @warn "$decomposition_name subproblem repricing: pricing hit its time limit before exhausting the search " *
                "while new columns were still being found -- completeness not fully proven this round" round
        @warn "$decomposition_name subproblem repricing found columns beyond the seeded pool -- pool was not complete " *
            "for this subproblem's own dual structure (dual degeneracy or genuine pool gap)" round n_new=length(all_new_columns)
        n_new_columns_total += length(all_new_columns)
        pool = _deduplicate_aggregate_od_route_columns(vcat(pool, all_new_columns))
        round == max_reprice_rounds && throw(ArgumentError(
            "$decomposition_name repricing found negative route columns in the final allowed round " *
            "max_reprice_rounds=$(max_reprice_rounds); the expanded LP must be re-solved and " *
            "re-priced to exhaustion before its cut is valid."
        ))
    end
    fully_exhausted || throw(ArgumentError("$decomposition_name repricing terminated without pricing exhaustion"))
    return v_hat, rho, pool, n_new_columns_total, rounds, fully_exhausted, max_objective_delta
end
