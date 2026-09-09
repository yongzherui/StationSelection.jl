"""
Benders cut-placeholder variables (`Theta`) -- the master-side stand-ins for the
second-stage cost that optimality cuts progressively underestimate.

Generic over formulations on purpose: nothing about a cut placeholder is specific to the
joint routing+assignment decomposition, so a future Benders decomposition of any other
formulation reuses this file rather than writing its own `Theta`.
"""

using JuMP

export add_benders_cut_variables!
export benders_cut_group

"""
    benders_cut_group(cut_mode::AbstractBendersCutMode, scenario::Int) -> Int

Which cut group scenario `scenario` belongs to -- i.e. which `Theta` variable carries its
second-stage cost.

`MultiCut(:scenario)` maps every scenario to itself (one `Theta` each);
`SingleCut` maps every scenario to `0`, the single aggregate group's key. The variable
builder below and the cut builder
(`add_benders_optimality_cut!`, `constraints/benders_cuts.jl`) both key off this one
function, so the two can't disagree about which placeholder a cut belongs on.
"""
benders_cut_group(::MultiCut, scenario::Int) = scenario
benders_cut_group(::SingleCut, scenario::Int) = 0

"""
    add_benders_cut_variables!(m, data, cut_mode; theta_lower_bound=0.0)
        -> Dict{Int, VariableRef}

Create the master's `Theta` variables, keyed by cut group (`benders_cut_group`).

**`theta_lower_bound` is load-bearing, not cosmetic.** A Benders master starts with no cuts, so
`Theta` is unconstrained from below; if the first stage also carries no cost of its own --
which is exactly the case for the joint routing+assignment master, whose entire objective
is second-stage -- then iteration 1 is *unbounded* rather than merely uninformative, and
the loop breaks on a non-optimal master before it has ever seen a subproblem. The default
`0.0` is valid for any decomposition whose second-stage cost is non-negative (walking and
route costs both are here); a decomposition with negative recourse cost must pass a real
bound instead.
"""
function add_benders_cut_variables!(
        m::Model,
        data::StationSelectionData,
        cut_mode::AbstractBendersCutMode;
        theta_lower_bound::Float64=0.0,
    )::Dict{Int, VariableRef}
    theta = Dict{Int, VariableRef}()
    for s in 1:n_scenarios(data)
        group = benders_cut_group(cut_mode, s)
        haskey(theta, group) && continue
        theta[group] = @variable(m, lower_bound = theta_lower_bound, base_name = "Theta[$group]")
    end
    return theta
end
