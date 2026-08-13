"""
Station-linking constraints for the joint routing+assignment CG master, built directly
off `AggregateODRouteMap`. Station budget (`sum(y) == l`) isn't declared here --
`add_station_limit_constraint!` (`constraints/base.jl`) already does exactly that, so
`build_model` calls it directly instead of a formulation-specific sibling.
"""

export add_joint_routing_assignment_station_linking_constraints!

"""
    add_joint_routing_assignment_station_linking_constraints!(m, data, mapping, y)
        -> (pickup_link, dropoff_link)

Disaggregated `((s,o,d), j)`/`((s,o,d), k)` linking rows, written as `-y[j] <= 0` (not
`0 <= y[j]`) so the normalized form JuMP stores is unambiguous: a route column's `theta`
coefficient of `+1.0`, added later via `set_normalized_coefficient`, then yields exactly
`theta - y[j] <= 0`. `j`/`k` range over the pickup/dropoff sides of every feasible
`(j,k)` in `valid_jk_pairs[(o,d)]` (`WALK_ONLY_PAIR`, when present, is skipped -- this
formulation doesn't yet support the station-free option).
"""
function add_joint_routing_assignment_station_linking_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    y::Vector{VariableRef},
)
    pickup_link = Dict{Tuple{NTuple{3, Int}, Int}, ConstraintRef}()
    dropoff_link = Dict{Tuple{NTuple{3, Int}, Int}, ConstraintRef}()
    for s in 1:n_scenarios(data)
        for (o, d) in mapping.Omega_s[s]
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            key3 = (s, o, d)
            pickups = Set{Int}()
            dropoffs = Set{Int}()
            for pair in get_valid_jk_pairs(mapping, o, d)
                is_walk_only_pair(pair) && continue
                j, k = pair
                push!(pickups, j)
                push!(dropoffs, k)
            end
            for j in pickups
                pickup_link[(key3, j)] = @constraint(m, -y[j] <= 0.0)
            end
            for k in dropoffs
                dropoff_link[(key3, k)] = @constraint(m, -y[k] <= 0.0)
            end
        end
    end
    return pickup_link, dropoff_link
end
