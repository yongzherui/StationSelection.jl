"""
StationSelectionProblem - the shared "what" of a station-selection problem: choose `k`
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
- `k`: number of stations selected in the first stage
- `max_walking_distance`: walking feasibility radius. Shared across every formulation
  that restricts station-pair assignment by walk distance -- not a formulation-specific
  encoding detail, since it reflects a real passenger constraint independent of how the
  model represents routing/assignment.
- `door_to_door_ratio`: the simulator's G2 service guarantee, as a pair filter. An
  assignment of OD `(o, d)` to stations `(j, k)` is offered only if
  `walk(o, j) + drive(j, k) + walk(k, d) <= door_to_door_ratio * drive(o, d)`, with
  walking and routing costs both read as travel times in seconds. The simulator promises
  destination arrival `<= request + door_to_door_ratio * t_V(o, d)`; the filter charges no
  waiting and no sharing detour, so it is a necessary condition only -- a pair it keeps can
  still miss G2 in simulation, a pair it drops always does. `Inf` (the default) switches it
  off and reproduces every selection made before 2026-09-27; the simulator's default is 2.0.
"""
struct StationSelectionProblem <: AbstractProblem
    data::StationSelectionData
    k::Int
    max_walking_distance::Float64
    door_to_door_ratio::Float64

    function StationSelectionProblem(
            data::StationSelectionData,
            k::Int;
            max_walking_distance::Number=300,
            door_to_door_ratio::Number=Inf,
        )
        k > 0 || throw(ArgumentError("k must be positive"))
        isfinite(max_walking_distance) && max_walking_distance > 0 ||
            throw(ArgumentError("max_walking_distance must be finite and positive"))
        door_to_door_ratio >= 1 ||
            throw(ArgumentError("door_to_door_ratio must be >= 1 (Inf switches it off), got $door_to_door_ratio"))
        isfinite(door_to_door_ratio) && !has_routing_costs(data) &&
            throw(ArgumentError("door_to_door_ratio needs routing costs (drive(o, d)); data has none"))
        new(data, k, Float64(max_walking_distance), Float64(door_to_door_ratio))
    end
end
