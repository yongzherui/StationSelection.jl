"""
Single-pass driver for pricing-aware dual selection.

Per CG iteration this does exactly one thing: hand back a *substitute* set of
duals for the ordinary pricing path to use. It performs no pricing of its own and
certifies nothing -- termination remains entirely the exact pricer's job, unchanged
from plain column generation.

    RMP solve -> [select_pricing_duals!] -> ordinary pricing -> add columns -> repeat

Because the returned duals are RMP-optimal by construction (`D = z_RMP` plus
`Phi_r <= 0` for every pooled column, both re-verified numerically), substituting
them cannot invalidate the optimality certificate: any RMP-optimal dual that
prices out proves full-LP optimality (see the weak-duality argument in
`dual_selection.jl`).

If the selector LP is not optimal, or the extracted point fails validation, the
caller keeps the RMP's own duals and proceeds as ordinary CG. Those paths WARN --
an earlier silent version of this fallback hid a bug that disabled the feature on
97 of 105 iterations while looking like a mere slowdown.
"""

export PassengerDualSelectionRoundLog
export select_pricing_duals!

struct PassengerDualSelectionRoundLog
    rmp_objective::Float64
    original_dual_objective::Float64
    selected_dual_objective::Float64
    selector_seconds::Float64
    n_positive_rho::Int
    sum_positive_rho::Float64
    max_positive_rho::Float64
    reward_distance_from_previous::Float64
    used_selected_duals::Bool
    fallback_reason::Symbol
end

"""
    select_pricing_duals!(selector, master, z_rmp, original_dual_objective, reference_rewards)
        -> (alpha, u, v, rewards, ok, log)

`ok == false` means the caller must fall back to the RMP's own duals; the returned
dicts are then meaningless and must not be used.
"""
function select_pricing_duals!(
    selector::PricingAwareDualSelector,
    master::PassengerFreeAssignmentMaster,
    z_rmp::Float64,
    original_dual_objective::Float64,
    reference_rewards::Dict{Tuple{Int, Int, Int}, Float64},
)
    empty_alpha = Dict{Int, Float64}()
    empty_uv = Dict{Tuple{Int, Int}, Float64}()
    empty_rw = Dict{Tuple{Int, Int, Int}, Float64}()

    t0 = time()
    sync_rmp_columns!(selector, values(master.columns))
    update_optimal_face!(selector, z_rmp)
    status = solve_dual_selector!(selector, reference_rewards)
    # Deliberately NOT `status == MOI.OPTIMAL`. Under the `:l0_count` MIP any
    # feasible incumbent is already on the optimal face and dual-feasible, hence a
    # valid dual -- so a TimeLimit/MIPGap stop is usable. `validate_selected_dual`
    # below is the real gate and re-checks that numerically. (An INFEASIBLE solve
    # yields no primal point and still falls back.)
    if primal_status(selector.model) != MOI.FEASIBLE_POINT
        secs = time() - t0
        @warn "dual selector LP not optimal; using the RMP's own duals this iteration" status z_rmp
        return empty_alpha, empty_uv, empty_uv, empty_rw, false,
            PassengerDualSelectionRoundLog(
                z_rmp, original_dual_objective, NaN, secs,
                0, 0.0, 0.0, 0.0, false, :selector_not_optimal)
    end

    alpha, u, v, eta, s, D = extract_selected_duals(selector)
    ok, diag = validate_selected_dual(
        selector, alpha, u, v, eta, s, D, z_rmp, values(master.columns))
    secs = time() - t0
    if !ok
        @warn "dual selector failed validation; using the RMP's own duals this iteration" diag
        return empty_alpha, empty_uv, empty_uv, empty_rw, false,
            PassengerDualSelectionRoundLog(
                z_rmp, original_dual_objective, D, secs,
                0, 0.0, 0.0, 0.0, false, :validation_failed)
    end

    rewards = compute_pricing_rewards(selector, alpha, u, v)
    pos = [r for r in values(rewards) if r > 0]
    ref_dist = 0.0
    for (tr, r) in rewards
        ref_dist += abs(r - get(reference_rewards, tr, 0.0))
    end

    return alpha, u, v, rewards, true,
        PassengerDualSelectionRoundLog(
            z_rmp, original_dual_objective, D, secs,
            length(pos), sum(pos; init=0.0), isempty(pos) ? 0.0 : maximum(pos),
            ref_dist, true, :none)
end
