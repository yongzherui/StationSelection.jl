export RouteCoveringProblem
export AnyAggregateODRouteProblem

"""
    RouteCoveringProblem <: AbstractProblem

Fixed-station, fixed-assignment aggregate OD route-covering problem. The
assignment map keys are `(scenario, origin_station, destination_station)` and
values are the assigned `(pickup_station, dropoff_station)` pair. Carries its own
`problem`/`formulation` pair rather than composing `AggregateODRouteProblem`
directly, since fixing `open_stations`/`fixed_assignments` is itself a problem-level
decision layered on top of the base aggregate-OD-route problem.
"""
struct RouteCoveringProblem <: AbstractProblem
    problem::AggregateODRouteProblem
    formulation::AbstractFormulation
    open_stations::Vector{Int}
    fixed_assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}}

    function RouteCoveringProblem(
            problem::AggregateODRouteProblem,
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

const AnyAggregateODRouteProblem = Union{
    AggregateODRouteProblem,
    RouteCoveringProblem,
}
