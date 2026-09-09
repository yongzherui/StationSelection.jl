"""
Station-linking constraints for the joint routing+assignment CG master, built directly
off `AggregateODRouteMap`. Station budget (`sum(y) == l`) isn't declared here --
`add_station_limit_constraint!` (`constraints/base.jl`) already does exactly that, so
`build_model` calls it directly instead of a formulation-specific sibling.
"""

export add_joint_routing_assignment_station_linking_constraints!

"""
    add_joint_routing_assignment_station_linking_constraints!(m, data, mapping, y;
        scenarios=1:n_scenarios(data)) -> (pickup_link, dropoff_link)

Disaggregated `((s,p), j)`/`((s,p), k)` linking rows, written as `-y[j] <= 0` (not
`0 <= y[j]`) so the normalized form JuMP stores is unambiguous: a route column's `theta`
coefficient of `+1.0`, added later via `set_normalized_coefficient`, then yields exactly
`theta - y[j] <= 0`. `j`/`k` range over the pickup/dropoff sides of every feasible
`(j,k)` in `valid_jk_pairs[(o,d)]` (`WALK_ONLY_PAIR`, when present, is skipped: it's
station-free by construction, linked instead via `add_walk_variables!`,
which adds no `y`/`z` linking at all).

`scenarios` restricts row creation to a subset, mirroring `add_walk_variables!`'s and
`add_joint_routing_assignment_coverage_constraints!`'s kwargs of the same name -- see the
latter for why (one Benders subproblem model per scenario).

**`y` is a plain `Vector{VariableRef}` in both the monolith and the Benders subproblem**,
which is why this function is reused verbatim by both rather than growing a numeric-RHS
variant. The subproblem creates `y` as an ordinary relaxed station variable and pins it
per iteration with `JuMP.fix(y[j], yhat[j]; force=true)`, so these rows keep the exact
`theta - y[j] <= 0` normalized form documented above and there is no second code path to
drift from this one. See `AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation`'s
docstring for the rest of the argument.
"""
function add_joint_routing_assignment_station_linking_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    y::Vector{VariableRef};
    scenarios::AbstractVector{Int}=1:n_scenarios(data),
)
    pickup_link = Dict{Tuple{Tuple{Int, Int}, Int}, ConstraintRef}()
    dropoff_link = Dict{Tuple{Tuple{Int, Int}, Int}, ConstraintRef}()
    for s in scenarios
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = mapping.Q_s[s][p]
            demand > 0 || continue
            key2 = (s, p)
            pickups = Set{Int}()
            dropoffs = Set{Int}()
            for pair in get_valid_jk_pairs(mapping, o, d)
                is_walk_only_pair(pair) && continue
                j, k = pair
                push!(pickups, j)
                push!(dropoffs, k)
            end
            for j in pickups
                pickup_link[(key2, j)] = @constraint(m, -y[j] <= 0.0)
            end
            for k in dropoffs
                dropoff_link[(key2, k)] = @constraint(m, -y[k] <= 0.0)
            end
        end
    end
    return pickup_link, dropoff_link
end
