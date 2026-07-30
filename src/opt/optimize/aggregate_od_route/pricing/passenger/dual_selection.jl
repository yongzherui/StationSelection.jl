"""
Pricing-aware dual selection: pick a *pricing-friendly* point on the RMP's
optimal dual face, then price against it with the ordinary pricing path.

Single-pass scheme, one selector LP per CG iteration:

    1. solve the RMP
    2. take z_RMP and the solver's duals
    3. re-optimize the duals over the RMP-optimal face  <-- this file
    4. price once against the selected duals
    5. add columns, repeat until exact pricing certifies

# Why this is exact

Let `pi` be the selected dual. If `D(pi) = z_RMP` with every RMP column
dual-feasible (so `pi` is optimal for the RMP), and exact pricing then finds no
violating route (`pi` is dual-feasible for the FULL master), then for this min
problem:

    D(pi) = z_RMP                      (pi optimal for the RMP)
    D(pi) <= z_full                    (weak duality, pi dual feasible for full)
    z_RMP >= z_full                    (RMP is a restriction)
    ==> z_RMP = D(pi) = z_full

So **any** RMP-optimal dual that prices out certifies full-LP optimality -- there
is nothing special about the dual the solver happens to return. Termination still
requires the exact pricer to report no violating route with an exhausted search;
this file never certifies anything on its own.

# What is being optimized

With the face pinned, the remaining freedom is used for two pricing-friendly aims:

    min  mu * sum |rho_pjk - rho_bar_pjk|  +  eps * sum max(0, rho_pjk)

  * the **stabilization** term keeps rewards near the previous iteration's,
    damping the bang-bang dual oscillation that slows CG convergence;
  * the **positive-reward** term shrinks how many `(p,j,k)` look attractive.
    That matters directly here: pricing cost scales with the number of
    positive-`rho` opportunities (`~P*n^2`), so fewer of them is a smaller label
    search, not merely fewer iterations.

Advantage over naive dual smoothing (`pi = a*pi_prev + (1-a)*pi_new`): an
interpolated dual generally lands **off** the optimal face, so pricing can report
"improving" columns that are not, requiring mis-pricing safeguards. Constraining
`D = z_RMP` explicitly makes the selected dual RMP-optimal *by construction*.

# Sign convention

    rho_pjk = alpha_p - u_pj - v_pk - W_pjk
    Phi_r   = sum_{(p,j,k) in r} rho_pjk - beta * c_r^route  =  -reduced_cost(r)

A route is improving exactly when `Phi_r > 0`. `route_dual_violation` is the one
shared implementation, so the selector and the pricer cannot drift apart.

# Exact dual of the RMP *as implemented*

`master.jl` differs from a textbook set-partitioning master in three ways that
each change the dual, so this is derived from the implemented primal:

  1. **Coverage is `>= 1`, not `= 1`** ==> `alpha_p >= 0`, not free.
  2. **Unserved slack `v_p >= 0`** with cost `M` ==> `alpha_p <= M`.
  3. **No-vehicle-route `x_same[p,j] >= 0`** with cost `W^same_pj`, in the coverage
     row and *both* linking rows for `j` ==> `alpha_p - u_pj - v_pj <= W^same_pj`.

With (theta_r >= 0, x_pj >= 0, v_p >= 0, y_j in [0,1]):

    (1_p)   sum_r a_rp theta_r + sum_j x_pj + v_p  >= 1        dual alpha_p >= 0
    (2_pj)  sum_r a^O_rpj theta_r + x_pj - y_j     <= 0        dual u_pj    >= 0
    (3_pk)  sum_r a^D_rpk theta_r + x_pk - y_k     <= 0        dual v_pk    >= 0
    (4)     sum_j y_j                               = L        dual eta    free
    (5_j)   y_j                                    <= 1        dual s_j     >= 0

the dual being

    max  D = sum_p alpha_p + L*eta - sum_j s_j
    s.t. (theta_r)  Phi_r <= 0
         (x_pj)     alpha_p - u_pj - v_pj <= W^same_pj
         (v_p)      alpha_p               <= M
         (y_j)      sum_p u_pj + sum_p v_pj + eta - s_j <= 0
         alpha, u, v, s >= 0,  eta free

`C_r` already contains the walking cost of a column's assignments, which is why
`W_pjk` sits inside `rho` rather than separately in `Phi`.

# Historical note

An earlier version of this file implemented an inner *witness loop*: price against
the selected dual, and when a violating route appeared, add it as a constraint
`Phi_r <= t`, minimize `t`, and re-price -- so that a route was only promoted into
the RMP once `t > 0` proved no dual on the face could avoid it. That kept the RMP
small but priced up to `max_rounds` times per CG iteration, and measured 1.2x-4.8x
SLOWER than plain CG despite genuinely cutting iterations (33->17 at n=20). It was
removed in favour of this single-pass form. If RMP size ever matters more than
speed, that is the trade-off to revisit.
"""

export PassengerDualSelectorConfig
export PricingAwareDualSelector
export route_dual_violation
export build_dual_selector, update_optimal_face!, sync_rmp_columns!
export solve_dual_selector!
export extract_selected_duals, compute_pricing_rewards, validate_selected_dual

"""
    PassengerDualSelectorConfig

`use_pricing_aware_dual_selection = false` reproduces ordinary column generation
exactly; nothing in this file runs unless it is enabled.
"""
struct PassengerDualSelectorConfig
    use_pricing_aware_dual_selection::Bool
    rmp_optimal_face_tolerance::Float64
    dual_feasibility_tolerance::Float64
    dual_selector_stabilization_weight::Float64
    dual_selector_positive_reward_weight::Float64
    # `:l1_stabilized` -- LP: min mu*sum|rho-rho_bar| + eps*sum max(0,rho)
    # `:l0_count`      -- MIP: min (number of positive rho)
    #
    # The L1 form penalizes the magnitude SUM, which under a pinned dual objective
    # is satisfied by spreading value into many small positives -- measured to
    # RAISE the positive-rho count by up to 46% over the raw duals, the opposite of
    # what pricing wants. `:l0_count` targets the count directly with a binary
    # indicator per triple.
    dual_selector_objective::Symbol
    dual_selector_mip_time_limit_sec::Float64
    dual_selector_mip_gap::Float64
end

function PassengerDualSelectorConfig(;
    use_pricing_aware_dual_selection::Bool=false,
    rmp_optimal_face_tolerance::Float64=1e-7,
    dual_feasibility_tolerance::Float64=1e-6,
    dual_selector_stabilization_weight::Float64=1.0,
    dual_selector_positive_reward_weight::Float64=1e-4,
    dual_selector_objective::Symbol=:l1_stabilized,
    dual_selector_mip_time_limit_sec::Float64=10.0,
    dual_selector_mip_gap::Float64=0.05,
)
    dual_selector_objective in (:l1_stabilized, :l0_count) ||
        throw(ArgumentError("dual_selector_objective must be :l1_stabilized or :l0_count"))
    return PassengerDualSelectorConfig(
        use_pricing_aware_dual_selection,
        rmp_optimal_face_tolerance,
        dual_feasibility_tolerance,
        dual_selector_stabilization_weight,
        dual_selector_positive_reward_weight,
        dual_selector_objective,
        dual_selector_mip_time_limit_sec,
        dual_selector_mip_gap,
    )
end

"""
    route_dual_violation(assignments, alpha, u, v, walk_cost, beta_route_cost) -> Float64

`Phi_r = sum rho_pjk - beta * c_r^route`; positive means improving. `walk_cost`
must return the SAME weighted walking cost the column's objective coefficient
contains, or `Phi` and the master's reduced cost will disagree.
"""
function route_dual_violation(
    assignments::AbstractVector{<:Tuple{Int, Int, Int}},
    alpha::Dict{Int, Float64},
    u::Dict{Tuple{Int, Int}, Float64},
    v::Dict{Tuple{Int, Int}, Float64},
    walk_cost,
    beta_route_cost::Float64,
)::Float64
    total = 0.0
    for (p, j, k) in assignments
        total += get(alpha, p, 0.0) - get(u, (p, j), 0.0) - get(v, (p, k), 0.0) - walk_cost(p, j, k)
    end
    return total - beta_route_cost
end

_selector_walk_cost(md::PassengerFreeAssignmentMasterData) =
    (p, j, k) -> md.walk_cost_weight * get(md.assignment_walk_cost, (p, j, k), 0.0)

_selector_beta_route_cost(md::PassengerFreeAssignmentMasterData, column) =
    md.route_regularization_weight * (column.tau + md.repositioning_time)

mutable struct PricingAwareDualSelector
    model::Model
    master_data::PassengerFreeAssignmentMasterData
    config::PassengerDualSelectorConfig
    alpha::Dict{Int, VariableRef}
    u::Dict{Tuple{Int, Int}, VariableRef}
    v::Dict{Tuple{Int, Int}, VariableRef}
    eta::VariableRef
    s::Vector{VariableRef}
    # stabilization variables over FEASIBLE triples only (never a dense P x J x J)
    triples::Vector{Tuple{Int, Int, Int}}
    dev_pos::Dict{Tuple{Int, Int, Int}, VariableRef}
    dev_neg::Dict{Tuple{Int, Int, Int}, VariableRef}
    q::Dict{Tuple{Int, Int, Int}, VariableRef}
    # `rho - dev_pos + dev_neg == ref`, created ONCE. The reference moves every CG
    # iteration; re-adding these rows would stack contradictory equalities on the
    # same variables and make the model permanently infeasible from the 2nd call on
    # (a real bug that silently disabled the whole feature).
    dev_link::Dict{Tuple{Int, Int, Int}, ConstraintRef}
    # `:l0_count` only: z_pjk = 1 allows rho_pjk > 0, z = 0 forces rho <= 0.
    z::Dict{Tuple{Int, Int, Int}, VariableRef}
    face_lower::Union{Nothing, ConstraintRef}
    face_upper::Union{Nothing, ConstraintRef}
    rmp_route_constraints::Dict{Any, ConstraintRef}
end

function build_dual_selector(
    master_data::PassengerFreeAssignmentMasterData,
    config::PassengerDualSelectorConfig,
    optimizer_env,
)::PricingAwareDualSelector
    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    set_silent(m)
    n = length(master_data.nodes)

    # alpha_p in [0, M]: `>= 0` because coverage is `>=`, `<= M` from the slack's
    # own dual constraint. Both are properties of the implemented primal.
    alpha = Dict{Int, VariableRef}()
    for p in master_data.passengers
        alpha[p.id] = @variable(m, lower_bound = 0.0, upper_bound = master_data.unserved_penalty,
                                base_name = "alpha[$(p.id)]")
    end

    u = Dict{Tuple{Int, Int}, VariableRef}()
    v = Dict{Tuple{Int, Int}, VariableRef}()
    for p in master_data.passengers
        for j in master_data.feasible_pickups[p.id]
            u[(p.id, j)] = @variable(m, lower_bound = 0.0, base_name = "u[$(p.id),$j]")
        end
        for k in master_data.feasible_dropoffs[p.id]
            v[(p.id, k)] = @variable(m, lower_bound = 0.0, base_name = "v[$(p.id),$k]")
        end
    end

    eta = @variable(m, base_name = "eta")
    s = @variable(m, [1:n], lower_bound = 0.0, base_name = "s")

    # station rows: sum_p u_pj + sum_p v_pj + eta - s_j <= 0
    for (jdx, _node) in enumerate(master_data.nodes)
        terms = AffExpr(0.0)
        for p in master_data.passengers
            haskey(u, (p.id, jdx)) && add_to_expression!(terms, 1.0, u[(p.id, jdx)])
            haskey(v, (p.id, jdx)) && add_to_expression!(terms, 1.0, v[(p.id, jdx)])
        end
        @constraint(m, terms + eta - s[jdx] <= 0.0)
    end

    # same-station rows: alpha_p - u_pj - v_pj <= W^same_pj
    for p in master_data.passengers
        for j in master_data.same_station_options[p.id]
            w_same = master_data.walk_cost_weight * master_data.same_station_walk_cost[(p.id, j)]
            expr = AffExpr(0.0)
            add_to_expression!(expr, 1.0, alpha[p.id])
            haskey(u, (p.id, j)) && add_to_expression!(expr, -1.0, u[(p.id, j)])
            haskey(v, (p.id, j)) && add_to_expression!(expr, -1.0, v[(p.id, j)])
            @constraint(m, expr <= w_same)
        end
    end

    triples = Tuple{Int, Int, Int}[]
    for p in master_data.passengers
        for (j, k) in master_data.feasible_assignments[p.id]
            push!(triples, (p.id, j, k))
        end
    end

    return PricingAwareDualSelector(
        m, master_data, config, alpha, u, v, eta, s,
        triples,
        Dict{Tuple{Int, Int, Int}, VariableRef}(),
        Dict{Tuple{Int, Int, Int}, VariableRef}(),
        Dict{Tuple{Int, Int, Int}, VariableRef}(),
        Dict{Tuple{Int, Int, Int}, ConstraintRef}(),
        Dict{Tuple{Int, Int, Int}, VariableRef}(),
        nothing, nothing,
        Dict{Any, ConstraintRef}(),
    )
end

"""`Phi_r` as a JuMP expression in the selector's dual variables."""
function _selector_violation_expr(selector::PricingAwareDualSelector, column)
    md = selector.master_data
    expr = AffExpr(0.0)
    for (p, j, k) in column.assignments
        haskey(selector.alpha, p) || continue
        add_to_expression!(expr, 1.0, selector.alpha[p])
        haskey(selector.u, (p, j)) && add_to_expression!(expr, -1.0, selector.u[(p, j)])
        haskey(selector.v, (p, k)) && add_to_expression!(expr, -1.0, selector.v[(p, k)])
        add_to_expression!(expr, -md.walk_cost_weight * get(md.assignment_walk_cost, (p, j, k), 0.0))
    end
    add_to_expression!(expr, -_selector_beta_route_cost(md, column))
    return expr
end

_selector_dual_objective_expr(selector::PricingAwareDualSelector) =
    sum(values(selector.alpha); init=AffExpr(0.0)) +
    selector.master_data.l * selector.eta -
    sum(selector.s; init=AffExpr(0.0))

"""
    update_optimal_face!(selector, z_rmp)

Pin `D` to the current RMP value as a two-sided band. The band is **relative**: an
absolute `1e-7` against an objective of order `1e3` is a relative tolerance below
the LP solver's own feasibility tolerance, which makes the constraint numerically
infeasible and silently disables the whole feature. Must be re-called whenever the
RMP changes.
"""
function update_optimal_face!(selector::PricingAwareDualSelector, z_rmp::Float64)
    isnothing(selector.face_lower) || delete(selector.model, selector.face_lower)
    isnothing(selector.face_upper) || delete(selector.model, selector.face_upper)
    eps_face = max(selector.config.rmp_optimal_face_tolerance,
                   1e-9 * max(1.0, abs(z_rmp)))
    D = _selector_dual_objective_expr(selector)
    selector.face_lower = @constraint(selector.model, D >= z_rmp - eps_face)
    selector.face_upper = @constraint(selector.model, D <= z_rmp + eps_face)
    return selector
end

"""
    sync_rmp_columns!(selector, columns)

Enforce `Phi_r <= 0` for every column currently in the primal RMP. These rows are
never weakened or omitted -- they are what keeps the selected point dual feasible
for the restricted master, i.e. genuinely on its optimal face.
"""
function sync_rmp_columns!(selector::PricingAwareDualSelector, columns)
    for column in columns
        sig = _passenger_free_assignment_column_signature(column)
        haskey(selector.rmp_route_constraints, sig) && continue
        selector.rmp_route_constraints[sig] =
            @constraint(selector.model, _selector_violation_expr(selector, column) <= 0.0)
    end
    return selector
end

"""
    solve_dual_selector!(selector, reference_rewards) -> status

The single selector LP: over the RMP-optimal face, minimize

    mu * sum |rho - rho_bar|  +  eps * sum max(0, rho)

with the absolute value linearized by `dev_pos`/`dev_neg` and the positive part by
`q >= rho, q >= 0`. Variables and their linking rows are created once; later calls
only move the reference via the row's RHS.
"""
function solve_dual_selector!(
    selector::PricingAwareDualSelector,
    reference_rewards::Dict{Tuple{Int, Int, Int}, Float64},
)
    selector.config.dual_selector_objective == :l0_count &&
        return _solve_dual_selector_l0!(selector)
    return _solve_dual_selector_l1!(selector, reference_rewards)
end

"""
`rho_pjk` as a JuMP expression in the dual variables.
"""
function _selector_rho_expr(selector::PricingAwareDualSelector, tr::Tuple{Int, Int, Int})
    md = selector.master_data
    p, j, k = tr
    rho = AffExpr(0.0)
    add_to_expression!(rho, 1.0, selector.alpha[p])
    haskey(selector.u, (p, j)) && add_to_expression!(rho, -1.0, selector.u[(p, j)])
    haskey(selector.v, (p, k)) && add_to_expression!(rho, -1.0, selector.v[(p, k)])
    add_to_expression!(rho, -md.walk_cost_weight * get(md.assignment_walk_cost, tr, 0.0))
    return rho
end

"""
    _solve_dual_selector_l0!(selector) -> status

MIP form: minimize the **number** of attractive assignments over the RMP-optimal
dual face,

    min sum_z  s.t.  rho_pjk <= M_pjk * z_pjk,  z binary,  (face + dual feasibility)

`z = 0` forces `rho <= 0`; minimizing `sum z` therefore counts exactly the triples
that cannot be made unattractive. This targets what actually drives pricing cost
(the count of positive-`rho` opportunities), which the L1 objective provably does
not -- it penalizes the magnitude sum and was measured to increase the count.

Big-M: `rho_pjk <= alpha_p - W_pjk <= unserved_penalty - W_pjk` since `u, v >= 0`,
so that is a valid per-triple bound and tighter than a single global constant.

**Solving to optimality is not required.** Any feasible point is on the face and
dual-feasible, hence a valid dual -- so a time limit / MIP gap is safe here in a
way it never is for the pricing certificate. `validate_selected_dual` is the real
gate, and it runs on whatever incumbent comes back.
"""
function _solve_dual_selector_l0!(selector::PricingAwareDualSelector)
    m = selector.model
    md = selector.master_data
    cfg = selector.config

    if isempty(selector.z)
        for tr in selector.triples
            w = md.walk_cost_weight * get(md.assignment_walk_cost, tr, 0.0)
            big_m = max(1e-6, md.unserved_penalty - w)
            zv = @variable(m, binary = true)
            selector.z[tr] = zv
            @constraint(m, _selector_rho_expr(selector, tr) <= big_m * zv)
        end
    end

    # Build the objective OUTSIDE the macro: `@objective` cannot rewrite a
    # `sum(...; init=...)` keyword call (MutableArithmetics tries to iterate the
    # `init` symbol and throws `MethodError: iterate(::Symbol)`).
    obj = AffExpr(0.0)
    for zv in values(selector.z)
        add_to_expression!(obj, 1.0, zv)
    end
    @objective(m, Min, obj)
    set_optimizer_attribute(m, "TimeLimit", cfg.dual_selector_mip_time_limit_sec)
    set_optimizer_attribute(m, "MIPGap", cfg.dual_selector_mip_gap)
    optimize!(m)
    return termination_status(m)
end

function _solve_dual_selector_l1!(
    selector::PricingAwareDualSelector,
    reference_rewards::Dict{Tuple{Int, Int, Int}, Float64},
)
    m = selector.model
    md = selector.master_data
    cfg = selector.config

    if isempty(selector.dev_pos)
        for tr in selector.triples
            p, j, k = tr
            selector.dev_pos[tr] = @variable(m, lower_bound = 0.0)
            selector.dev_neg[tr] = @variable(m, lower_bound = 0.0)
            selector.q[tr] = @variable(m, lower_bound = 0.0)

            rho = AffExpr(0.0)
            add_to_expression!(rho, 1.0, selector.alpha[p])
            haskey(selector.u, (p, j)) && add_to_expression!(rho, -1.0, selector.u[(p, j)])
            haskey(selector.v, (p, k)) && add_to_expression!(rho, -1.0, selector.v[(p, k)])
            add_to_expression!(rho, -md.walk_cost_weight * get(md.assignment_walk_cost, tr, 0.0))

            selector.dev_link[tr] = @constraint(
                m, rho - selector.dev_pos[tr] + selector.dev_neg[tr] == 0.0)
            @constraint(m, selector.q[tr] >= rho)
        end
    end

    obj = AffExpr(0.0)
    for tr in selector.triples
        set_normalized_rhs(selector.dev_link[tr], get(reference_rewards, tr, 0.0))
        add_to_expression!(obj, cfg.dual_selector_stabilization_weight, selector.dev_pos[tr])
        add_to_expression!(obj, cfg.dual_selector_stabilization_weight, selector.dev_neg[tr])
        add_to_expression!(obj, cfg.dual_selector_positive_reward_weight, selector.q[tr])
    end

    @objective(m, Min, obj)
    optimize!(m)
    return termination_status(m)
end

"""
    extract_selected_duals(selector) -> (alpha, u, v, eta, s, D)

Numeric duals in the same `>= 0` orientation `route_dual_violation` expects, plus
the achieved dual objective.
"""
function extract_selected_duals(selector::PricingAwareDualSelector)
    alpha = Dict{Int, Float64}(p => value(var) for (p, var) in selector.alpha)
    u = Dict{Tuple{Int, Int}, Float64}(key => value(var) for (key, var) in selector.u)
    v = Dict{Tuple{Int, Int}, Float64}(key => value(var) for (key, var) in selector.v)
    eta = value(selector.eta)
    s = [value(var) for var in selector.s]
    D = sum(values(alpha); init=0.0) + selector.master_data.l * eta - sum(s; init=0.0)
    return alpha, u, v, eta, s, D
end

"""`rho_pjk = alpha_p - u_pj - v_pk - W_pjk` over feasible triples only."""
function compute_pricing_rewards(
    selector::PricingAwareDualSelector,
    alpha::Dict{Int, Float64},
    u::Dict{Tuple{Int, Int}, Float64},
    v::Dict{Tuple{Int, Int}, Float64},
)::Dict{Tuple{Int, Int, Int}, Float64}
    md = selector.master_data
    rewards = Dict{Tuple{Int, Int, Int}, Float64}()
    for tr in selector.triples
        p, j, k = tr
        rewards[tr] = get(alpha, p, 0.0) - get(u, (p, j), 0.0) - get(v, (p, k), 0.0) -
            md.walk_cost_weight * get(md.assignment_walk_cost, tr, 0.0)
    end
    return rewards
end

"""
    validate_selected_dual(...) -> (ok, diagnostics)

Independent re-check of the extracted numbers, not a restatement of what the
solver was asked to do: `|D - z_rmp|` within tolerance, every current-RMP column
`Phi_r <= tol`, every station row satisfied, and all sign restrictions. A false
verdict must send the caller back to the RMP's own duals; it may never be read as
an optimality statement.
"""
function validate_selected_dual(
    selector::PricingAwareDualSelector,
    alpha::Dict{Int, Float64},
    u::Dict{Tuple{Int, Int}, Float64},
    v::Dict{Tuple{Int, Int}, Float64},
    eta::Float64,
    s::Vector{Float64},
    D::Float64,
    z_rmp::Float64,
    columns,
)
    md = selector.master_data
    cfg = selector.config
    walk = _selector_walk_cost(md)
    face_gap = abs(D - z_rmp)
    ok = face_gap <= max(cfg.rmp_optimal_face_tolerance, 1e-6 * max(1.0, abs(z_rmp)))

    worst_column = 0.0
    for column in columns
        worst_column = max(worst_column, route_dual_violation(
            column.assignments, alpha, u, v, walk, _selector_beta_route_cost(md, column)))
    end
    ok &= worst_column <= cfg.dual_feasibility_tolerance

    worst_station = -Inf
    for (jdx, _node) in enumerate(md.nodes)
        acc = eta - s[jdx]
        for p in md.passengers
            acc += get(u, (p.id, jdx), 0.0) + get(v, (p.id, jdx), 0.0)
        end
        worst_station = max(worst_station, acc)
    end
    ok &= worst_station <= cfg.dual_feasibility_tolerance

    worst_sign = 0.0
    for val in values(alpha); worst_sign = max(worst_sign, -val); end
    for val in values(u); worst_sign = max(worst_sign, -val); end
    for val in values(v); worst_sign = max(worst_sign, -val); end
    for val in s; worst_sign = max(worst_sign, -val); end
    ok &= worst_sign <= cfg.dual_feasibility_tolerance

    return ok, (
        face_gap=face_gap, worst_column_violation=worst_column,
        worst_station_violation=worst_station, worst_sign_violation=worst_sign,
    )
end
