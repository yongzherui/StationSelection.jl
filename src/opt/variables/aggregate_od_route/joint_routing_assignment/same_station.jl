"""
Same-station ("no vehicle route") assignment variable for the joint routing+assignment CG
master, built directly off `AggregateODRouteMap`.

`y` is not declared here: it's the same `add_station_selection_variables!`
(`variables/base.jl`, `relax_integrality=true`) every other `AggregateODRouteProblem`
formulation uses, since `mapping`'s station indices are already `1:data.n_stations`.
"""

export add_joint_routing_assignment_same_station_variables!

"""
    add_joint_routing_assignment_same_station_variables!(m, data, mapping, coverage, pickup_link, dropoff_link)
        -> Dict{Tuple{NTuple{3,Int},Int}, VariableRef}

Same-station ("no vehicle route") assignment `x_same[(s,o,d),j] >= 0` for every `j` where
`valid_jk_pairs[(o,d)]` contains the real same-station pair `(j,j)`
(`is_same_station_pair`) -- present whenever `create_aggregate_od_route_map`'s
`allow_same_station=true` produced one instead of routing that OD through
`WALK_ONLY_PAIR`. Demand groups whose only same-station option is `WALK_ONLY_PAIR`
(only possible when `allow_walk_only=true`) get no same-station variable here -- this
formulation doesn't yet support the station-free option, so those groups fall back to a
real two-station route or the unserved slack `v`.

Wired directly into the already-built `coverage`/`pickup_link`/`dropoff_link` rows they
share with route columns. No explicit upper bound: `x_same[key,j] <= y[j] <= 1` already
follows from the pickup row.
"""
function add_joint_routing_assignment_same_station_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    coverage::Dict{NTuple{3, Int}, ConstraintRef},
    pickup_link::Dict{Tuple{NTuple{3, Int}, Int}, ConstraintRef},
    dropoff_link::Dict{Tuple{NTuple{3, Int}, Int}, ConstraintRef},
)::Dict{Tuple{NTuple{3, Int}, Int}, VariableRef}
    x_same = Dict{Tuple{NTuple{3, Int}, Int}, VariableRef}()
    for s in 1:n_scenarios(data)
        for (o, d) in mapping.Omega_s[s]
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            key3 = (s, o, d)
            for pair in get_valid_jk_pairs(mapping, o, d)
                is_same_station_pair(pair) || continue
                j = pair[1]
                x = @variable(m, lower_bound = 0.0, base_name = "x_same[$s,$o,$d,$j]")
                x_same[(key3, j)] = x
                set_normalized_coefficient(coverage[key3], x, 1.0)
                haskey(pickup_link, (key3, j)) && set_normalized_coefficient(pickup_link[(key3, j)], x, 1.0)
                haskey(dropoff_link, (key3, j)) && set_normalized_coefficient(dropoff_link[(key3, j)], x, 1.0)
            end
        end
    end
    m[:joint_routing_assignment_x_same] = x_same
    return x_same
end
