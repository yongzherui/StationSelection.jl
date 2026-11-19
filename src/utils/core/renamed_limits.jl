# Service-limit names (renamed 2026-11-18 to the project's limit names, parent CLAUDE.md "Service guarantees"):
#   max_walking_distance -> walking_limit_s            (walking limit; a time in seconds)
#   door_to_door_ratio   -> door_to_door_limit_ratio   (door-to-door limit)
#   detour_factor        -> ride_time_limit_ratio      (ride-time limit, ratio term)
# The old names are still accepted, with a warning, as keyword arguments of `StationSelectionProblem` and the route
# formulations, and still read as fields (`getproperty`, defined at the end of the module), so scripts written before
# the rename keep working. They will be removed.

"The current name of a field that was renamed on 2026-11-18 (any other name unchanged)."
@inline _current_limit_field(f::Symbol) =
    f === :max_walking_distance ? :walking_limit_s :
    f === :door_to_door_ratio ? :door_to_door_limit_ratio :
    f === :detour_factor ? :ride_time_limit_ratio : f

"Resolve a keyword argument that has an old name: error if both are given, warn if only the old one is."
function _renamed_kwarg(new, old, newname::Symbol, oldname::Symbol)
    old === nothing && return new
    new === nothing || throw(ArgumentError("both `$(newname)` and its old name `$(oldname)` were given"))
    @warn "Keyword `$(oldname)` was renamed to `$(newname)`; the old name will be removed." maxlog = 1 _id = oldname
    return old
end
