export RouteCoveringProblem

"""
    RouteCoveringProblem <: AbstractProblem

Fixed-station, fixed-assignment aggregate OD route-covering problem: the shape of a
Benders subproblem once both first-stage `y` and the assignment are pinned, leaving
only route activation (`theta`) free. The assignment map keys are `(scenario,
origin_station, destination_station)` and values are the assigned `(pickup_station,
dropoff_station)` pair. Carries its own `problem`/`formulation` pair rather than
composing `StationSelectionProblem` directly, since fixing
`open_stations`/`fixed_assignments` is itself a problem-level decision layered on top
of the base aggregate-OD-route problem.

Not wired to any `build_model`/`Solver` currently -- kept as a reminder of the shape a
future Benders subproblem should reuse (see `opt/formulations/aggregate_od_route/
benders/*.jl` for the corresponding master-side Formulation markers) rather than
reinventing fixed-station/fixed-assignment plumbing from scratch.
"""
struct RouteCoveringProblem <: AbstractProblem
    problem::StationSelectionProblem
    formulation::AbstractFormulation
    open_stations::Vector{Int}
    fixed_assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}}

    function RouteCoveringProblem(
            problem::StationSelectionProblem,
            formulation::AbstractFormulation,
            open_stations::AbstractVector{<:Integer},
            fixed_assignments::AbstractDict{<:Tuple{Int, Int, Int}, <:Tuple{Int, Int}},
        )
        unique_open = sort!(unique(Int.(open_stations)))
        length(unique_open) == problem.l ||
            throw(ArgumentError("open_stations must contain exactly l unique stations"))
        assignments = Dict{NTuple{3, Int}, Tuple{Int, Int}}()
        for (key, pair) in fixed_assignments
            assignments[(Int(key[1]), Int(key[2]), Int(key[3]))] = (Int(pair[1]), Int(pair[2]))
        end
        new(problem, formulation, unique_open, assignments)
    end
end
