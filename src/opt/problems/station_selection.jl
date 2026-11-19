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
- `walking_limit_s`: walking feasibility radius. Shared across every formulation
  that restricts station-pair assignment by walk distance -- not a formulation-specific
  encoding detail, since it reflects a real passenger constraint independent of how the
  model represents routing/assignment.
- `door_to_door_limit_ratio`: the simulator's door-to-door limit, as a pair filter. An
  assignment of OD `(o, d)` to stations `(j, k)` is offered only if
  `walk(o, j) + drive(j, k) + walk(k, d) <= door_to_door_limit_ratio * drive(o, d)`, with
  walking and routing costs both read as travel times in seconds. The simulator promises
  destination arrival `<= request + door_to_door_limit_ratio * t_V(o, d)`; the filter charges no
  waiting and no sharing detour, so it is a necessary condition only -- a pair it keeps can
  still miss the door-to-door limit in simulation, a pair it drops always does. `Inf` (the default) switches it
  off and reproduces every selection made before 2026-09-27; the simulator's default is 2.0.
  An OD the filter leaves with no pair at all (`o == d`) is an `ArgumentError` at map build
  (`check_door_to_door_servable`), not a silent drop.
"""
struct StationSelectionProblem <: AbstractProblem
    data::StationSelectionData
    k::Int
    walking_limit_s::Float64
    door_to_door_limit_ratio::Float64

    function StationSelectionProblem(
            data::StationSelectionData,
            k::Int;
            walking_limit_s::Union{Number, Nothing}=nothing,
            door_to_door_limit_ratio::Union{Number, Nothing}=nothing,
            max_walking_distance::Union{Number, Nothing}=nothing,   # old name of walking_limit_s
            door_to_door_ratio::Union{Number, Nothing}=nothing,     # old name of door_to_door_limit_ratio
        )
        walking_limit_s = something(_renamed_kwarg(walking_limit_s, max_walking_distance,
                                                   :walking_limit_s, :max_walking_distance), 300)
        door_to_door_limit_ratio = something(_renamed_kwarg(door_to_door_limit_ratio, door_to_door_ratio,
                                                            :door_to_door_limit_ratio, :door_to_door_ratio), Inf)
        k > 0 || throw(ArgumentError("k must be positive"))
        isfinite(walking_limit_s) && walking_limit_s > 0 ||
            throw(ArgumentError("walking_limit_s must be finite and positive"))
        door_to_door_limit_ratio >= 1 ||
            throw(ArgumentError("door_to_door_limit_ratio must be >= 1 (Inf switches it off), got $door_to_door_limit_ratio"))
        isfinite(door_to_door_limit_ratio) && !has_routing_costs(data) &&
            throw(ArgumentError("door_to_door_limit_ratio needs routing costs (drive(o, d)); data has none"))
        new(data, k, Float64(walking_limit_s), Float64(door_to_door_limit_ratio))
    end
end
