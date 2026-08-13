"""
StationSelectionProblem - the shared "what" of a station-selection problem: choose `l`
stations to build, with a global walking-distance feasibility radius. Everything about
how demand gets served, weighted, or staged (including whether there even is a second,
per-scenario activation stage) belongs to the paired `AbstractFormulation`, not here --
see `opt/abstract.jl`'s `AbstractProblem` docstring for the `Problem`/`Formulation` split
this is built around.
"""

export StationSelectionProblem

"""
    StationSelectionProblem <: AbstractProblem

# Fields
- `data`: instance data (stations, requests, costs, scenarios)
- `l`: number of stations selected in the first stage
- `max_walking_distance`: walking feasibility radius. Shared across every formulation
  that restricts station-pair assignment by walk distance -- not a formulation-specific
  encoding detail, since it reflects a real passenger constraint independent of how the
  model represents routing/assignment.
"""
struct StationSelectionProblem <: AbstractProblem
    data::StationSelectionData
    l::Int
    max_walking_distance::Float64

    function StationSelectionProblem(
            data::StationSelectionData,
            l::Int;
            max_walking_distance::Number=300,
        )
        l > 0 || throw(ArgumentError("l must be positive"))
        max_walking_distance >= 0 ||
            throw(ArgumentError("max_walking_distance must be non-negative"))
        new(data, l, Float64(max_walking_distance))
    end
end
