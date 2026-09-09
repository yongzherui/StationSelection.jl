"""
Turning a `JointRoutingAssignmentBendersSubproblemResult` into master rows.

The row itself is the generic `add_benders_optimality_cut!`
(`constraints/benders_cuts.jl`); what lives here is which `Theta` each scenario's cut
lands on (`benders_cut_group`, so `SingleCut` aggregates and `MultiCut` does not) and the
deduplication that makes `add_benders_cut!`'s return value meaningful.

# Two records of each cut, for two different jobs

`m[:benders_cut_signatures]` is the ROUNDED (6-decimal) dedup key -- rounding is required
there, since an exact-equality test on raw `Float64` duals would miss the repeats the set
exists to catch. `m[:benders_cuts]` is the list of `(group, ConstraintRef)` actually added.
The two are not interchangeable: anything auditing cut VALIDITY must read the ConstraintRefs,
because a cut rebuilt from its rounded signature is stronger than the real one by up to
~5e-7 per term and will report violations that the real cut does not have.

# Why deduplicate

The subproblem is a covering LP with a great deal of symmetry, so its optimal duals are
routinely degenerate: two iterations at the same incumbent -- or at different incumbents
with the same active columns -- can produce bit-identical `(alpha, Gamma)` and therefore
the identical cut. Adding it again changes nothing about the master, so the next iteration
returns the same incumbent, and the loop spins to `max_iterations` reporting progress it
is not making. Counting only genuinely new cuts lets the loop detect that and stop with
`cut_repeated`.
"""

"""
    _add_joint_routing_assignment_benders_cuts!(m, subproblem_result, cut_mode) -> Int

Add one cut per cut group and return how many were new.

`SingleCut` sums the scenarios' cut data into one row before adding
(`sum_s alpha_s - sum_j (sum_s Gamma[s,j]) y_j >= Theta`, which is exactly the sum of the
per-scenario cuts and therefore valid for the same reason, just weaker); `MultiCut`
adds one row per scenario.
"""
function _add_joint_routing_assignment_benders_cuts!(
        m::JuMP.Model,
        subproblem_result::JointRoutingAssignmentBendersSubproblemResult,
        cut_mode::AbstractBendersCutMode,
    )::Int
    theta_cuts = m[:benders_cut_variables]::Dict{Int, VariableRef}
    signatures = m[:benders_cut_signatures]::Set{Any}
    added_rows = m[:benders_cuts]::Vector{Tuple{Int, ConstraintRef}}
    y = m[:y]

    grouped = Dict{Int, Tuple{Float64, Dict{Int, Float64}}}()
    for scenario_result in subproblem_result.scenarios
        group = benders_cut_group(cut_mode, scenario_result.scenario)
        constant, coefficients = get!(grouped, group) do
            (0.0, Dict{Int, Float64}())
        end
        constant += scenario_result.cut_constant
        for (j, gamma) in scenario_result.y_coefficients
            coefficients[j] = get(coefficients, j, 0.0) + gamma
        end
        grouped[group] = (constant, coefficients)
    end

    n_added = 0
    for group in sort!(collect(keys(grouped)))
        constant, coefficients = grouped[group]
        signature = _benders_cut_signature(group, constant, coefficients)
        signature in signatures && continue
        push!(signatures, signature)
        row = add_benders_optimality_cut!(m, theta_cuts[group], constant, y, coefficients)
        # Keep the real ConstraintRef, not just the signature. The signature is ROUNDED to 6
        # decimals for dedup, so reconstructing a cut from it is not the cut that was added --
        # rounding the constant up and each coefficient down strengthens the reconstruction by
        # up to ~5e-7 per term, which is enough to fake a validity violation in an audit at
        # objective magnitudes of 1e4. An auditor needs the exact row: `normalized_rhs` gives
        # the constant and `normalized_coefficient(row, y[j])` the coefficients.
        push!(added_rows, (group, row))
        n_added += 1
    end
    return n_added
end

"""
    _benders_cut_signature(group, constant, coefficients) -> Tuple

A comparable identity for one cut row. Rounded to 6 decimals: an exact-equality test on
raw `Float64` duals would miss the repeats this exists to catch (the same degenerate
vertex reached by a different pivot sequence differs in the last bits), while a coarser
rounding would start conflating genuinely different cuts and stop the loop early.
Zero coefficients are dropped so that a station absent from one iteration's dual and
present-at-zero in the next still compares equal.
"""
function _benders_cut_signature(group::Int, constant::Float64, coefficients::Dict{Int, Float64})
    nonzero = [(j, round(v; digits = 6)) for (j, v) in coefficients if v != 0.0]
    sort!(nonzero; by = first)
    return (group, round(constant; digits = 6), nonzero)
end
