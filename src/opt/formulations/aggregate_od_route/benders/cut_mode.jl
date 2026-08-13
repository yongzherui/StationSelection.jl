"""
Cut-aggregation modes shared by every Benders-decomposed `AggregateODRoute` formulation
(`AggregateODRouteBendersYXFormulation` and friends) -- *how many* `theta` cut-placeholder
variables the master carries and how subproblem cut groups are formed, independent of
which first-stage variables the master fixes.
"""

export AbstractBendersCutMode
export SingleCut
export MultiCut

"""
    AbstractBendersCutMode

Root type for how a Benders master aggregates its subproblem into cut-placeholder
`theta` variables.
"""
abstract type AbstractBendersCutMode end

"""
    SingleCut

One `theta` variable for the whole problem: every scenario's subproblem is solved and
summed into a single cut per master iteration.
"""
struct SingleCut <: AbstractBendersCutMode end

"""
    MultiCut(dimension=:scenario)

One `theta` variable (and one cut) per `dimension` group -- only `:scenario` is
currently supported, giving one `theta[s]` per scenario `s`. Typically converges in
fewer outer iterations than `SingleCut` at the cost of a larger master.
"""
struct MultiCut <: AbstractBendersCutMode
    dimension::Symbol

    function MultiCut(dimension::Symbol=:scenario)
        dimension == :scenario ||
            throw(ArgumentError("only MultiCut(:scenario) is currently supported"))
        new(dimension)
    end
end
