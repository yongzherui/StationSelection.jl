"""
Linking constraints for `AggregateODRouteBaseFormulation`'s `y`/`x`/`θ` master: `x` to
station selection `y`, and `x` to route activation `θ`. Station budget (`sum(y) == l`)
isn't declared here -- `add_station_limit_constraint!` (`constraints/base.jl`) already
does exactly that.
"""

export add_aggregate_od_route_base_station_linking_constraints!
export add_aggregate_od_route_base_route_linking_constraints!

"""
    add_aggregate_od_route_base_station_linking_constraints!(m, x, y) -> (pickup_link, dropoff_link)

`x[s,p,j,k] <= y[j]`, `x[s,p,j,k] <= y[k]` for every declared `x` -- an assignment
can only use a station pair whose both endpoints are built.
"""
function add_aggregate_od_route_base_station_linking_constraints!(
    m::Model,
    x::Dict{NTuple{4, Int}, VariableRef},
    y::Vector{VariableRef},
)
    pickup_link = Dict{NTuple{4, Int}, ConstraintRef}()
    dropoff_link = Dict{NTuple{4, Int}, ConstraintRef}()
    for (key, var) in x
        _, _, j, k = key
        pickup_link[key] = @constraint(m, var <= y[j])
        dropoff_link[key] = @constraint(m, var <= y[k])
    end
    return pickup_link, dropoff_link
end

"""
    add_aggregate_od_route_base_route_linking_constraints!(m, mapping, x, theta) -> Dict{NTuple{4,Int}, ConstraintRef}

`x[s,p,j,k] <= sum(theta[r,s] for r covering (j,k))` -- an assignment can only use a
station pair some active route in that scenario actually covers. `theta` is keyed
`(column_id, s)`, matching `add_aggregate_od_route_theta_variables!`'s `m[:theta_compat]`
convention; `mapping.columns_by_pair[(j,k)]` lists every route id covering `(j,k)`.
"""
function add_aggregate_od_route_base_route_linking_constraints!(
    m::Model,
    mapping::AggregateODRouteMap,
    x::Dict{NTuple{4, Int}, VariableRef},
    theta::Dict{Tuple{Int, Int}, VariableRef},
)::Dict{NTuple{4, Int}, ConstraintRef}
    route_link = Dict{NTuple{4, Int}, ConstraintRef}()
    for (key, var) in x
        s, _, j, k = key
        terms = AffExpr(0.0)
        for r in get(mapping.columns_by_pair, (j, k), Int[])
            haskey(theta, (r, s)) && add_to_expression!(terms, theta[(r, s)])
        end
        route_link[key] = @constraint(m, var <= terms)
    end
    return route_link
end
