"""
Variable+constraint declarations for `BendersYZ`'s `lifted_routing_lower_bound`/
`common_od_mcf_lower_bound` multicommodity arc-flow relaxation
(`lifted_routing_lower_bound.jl`'s `_build_lifted_routing_lower_bound_exprs!`). See that file's
module docstring for why this relaxation is a valid lower bound.
"""

"""
    add_arc_flow_variables!(m, y, stations_list) -> (f, f0, fj0)

Aggregate arc-flow + depot arcs for one scenario's relaxation: `f[(j,k)] >= 0` per ordered
station pair, `f0[j]`/`fj0[j] >= 0` per station (route start/end, capturing route count and the
flat repositioning-time fee), each coupled to `y` (`<= y[j]`, `<= y[k]`) so an unopened station
carries no flow. Deliberately uncapped above.
"""
function add_arc_flow_variables!(m::JuMP.Model, y, stations_list::Vector{Int})
    f = Dict{Tuple{Int, Int}, VariableRef}()
    for j in stations_list, k in stations_list
        j == k && continue
        f[(j, k)] = @variable(m, lower_bound = 0.0)
        @constraint(m, f[(j, k)] <= y[j])
        @constraint(m, f[(j, k)] <= y[k])
    end
    f0 = Dict{Int, VariableRef}()
    fj0 = Dict{Int, VariableRef}()
    for j in stations_list
        f0[j] = @variable(m, lower_bound = 0.0)
        fj0[j] = @variable(m, lower_bound = 0.0)
        @constraint(m, f0[j] <= y[j])
        @constraint(m, fj0[j] <= y[j])
    end
    return f, f0, fj0
end

"""
    add_flow_conservation_constraints!(m, f, f0, fj0, stations_list)

Routes are walks, so in == out at every stop (depot arcs close the loop).
"""
function add_flow_conservation_constraints!(m::JuMP.Model, f, f0, fj0, stations_list::Vector{Int})
    for j in stations_list
        out_flow = sum(f[(j, k)] for k in stations_list if k != j; init = 0.0) + fj0[j]
        in_flow = sum(f[(k, j)] for k in stations_list if k != j; init = 0.0) + f0[j]
        @constraint(m, out_flow == in_flow)
    end
    return nothing
end

"""
    add_commodity_reachability_constraints!(m, f, stations_list, net_supply) -> g

Per-commodity sub-flow coupled to the aggregate arc-flow `f` (`g[(u,v)] <= f[(u,v)]`), balanced
against `net_supply` (the request's fractionally-selected pickup/dropoff endpoint selectors,
positive at pickup candidates and negative at dropoff candidates).
"""
function add_commodity_reachability_constraints!(
    m::JuMP.Model, f, stations_list::Vector{Int}, net_supply::Dict{Int, AffExpr},
)
    g = Dict{Tuple{Int, Int}, VariableRef}()
    for u in stations_list, v in stations_list
        u == v && continue
        g[(u, v)] = @variable(m, lower_bound = 0.0)
        @constraint(m, g[(u, v)] <= f[(u, v)])
    end
    for u in stations_list
        out_flow = sum(g[(u, v)] for v in stations_list if v != u; init = 0.0)
        in_flow = sum(g[(v, u)] for v in stations_list if v != u; init = 0.0)
        @constraint(m, out_flow - in_flow == get(net_supply, u, AffExpr(0.0)))
    end
    return g
end
