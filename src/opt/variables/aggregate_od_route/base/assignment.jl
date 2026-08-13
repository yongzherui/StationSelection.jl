"""
OD-to-station-pair assignment variable for `AggregateODRouteBaseFormulation`'s
`y`/`x`/`θ` master. `x` is decoupled from route activation `θ` -- linked via
`add_aggregate_od_route_base_route_linking_constraints!`
(`constraints/aggregate_od_route/base/linking.jl`) -- unlike
`AggregateODRouteJointRoutingAssignmentFormulation`, where route columns carry
assignment directly and there is no separate `x`.
"""

export add_aggregate_od_route_base_assignment_variables!

"""
    add_aggregate_od_route_base_assignment_variables!(m, data, mapping; scenarios=1:n_scenarios(data), relax_integrality=false) -> Dict{NTuple{5,Int}, VariableRef}

Assignment `x[s,o,d,j,k]` for every demand-positive `(s,o,d)` with `s in scenarios` and
every feasible `(j,k) in get_valid_jk_pairs(mapping,o,d)` that requires a real vehicle
route (`requires_no_vehicle_route` pairs excluded, matching `θ`'s own domain --
this formulation doesn't yet support the station-free/same-station option).

`scenarios` restricts variable creation to a subset of scenarios -- used by
`AggregateODRouteBendersYXFormulation`'s subproblem, which is solved one scenario at a
time (default keeps every existing caller's behavior unchanged). `relax_integrality`
declares `x` continuous on `[0,1]` instead of binary, mirroring
`add_aggregate_od_route_theta_variables!`'s own kwarg -- the Benders subproblem needs
this for valid LP duals off its `y`-fixing constraints.
"""
function add_aggregate_od_route_base_assignment_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap;
    scenarios::AbstractVector{Int}=1:n_scenarios(data),
    relax_integrality::Bool=false,
)::Dict{NTuple{5, Int}, VariableRef}
    x = Dict{NTuple{5, Int}, VariableRef}()
    for s in scenarios
        for (o, d) in mapping.Omega_s[s]
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            for pair in get_valid_jk_pairs(mapping, o, d)
                requires_no_vehicle_route(pair) && continue
                j, k = pair
                if relax_integrality
                    x[(s, o, d, j, k)] = @variable(m, lower_bound = 0.0, upper_bound = 1.0, base_name = "x[$s,$o,$d,$j,$k]")
                else
                    x[(s, o, d, j, k)] = @variable(m, binary = true, base_name = "x[$s,$o,$d,$j,$k]")
                end
            end
        end
    end
    m[:aggregate_od_route_base_x] = x
    return x
end
