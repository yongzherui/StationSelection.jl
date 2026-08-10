"""
Coverage constraints linking a fixed/free assignment variable to the shared `lambda` route
pool (`sum(lambda covering this pair) >= assigned_var`), for the four Benders subproblem
builders. Two methods: one keyed by `(request, pair)` (`BendersY`/`BendersXY`/`BendersYZ`), one
by `(physical_pair, pair)` with per-scenario occurrence expansion (`BendersYZH`) -- both skip
walk-only/same-station pairs (`requires_no_vehicle_route`), which use no vehicle route and so
need no coverage row, and both return `cover_cons` keyed by the flat `(request, pair)` shape so
downstream dual-extraction code (`_extract_nearest_open_y_subproblem_coverage_duals`) works
unmodified regardless of which method built the model.
"""

"""
    add_benders_route_coverage_constraints!(m, lambda, requests, feasible_pairs, columns, assigned_vars) -> cover_cons

`BendersY`/`BendersXY`/`BendersYZ` variant: `assigned_vars` is the subproblem's own `x`
(or `y`-subproblem's `x`), keyed `(request, pair)`.
"""
function add_benders_route_coverage_constraints!(
    m::JuMP.Model,
    lambda,
    requests,
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    columns::Vector{AggregateODRouteColumn},
    assigned_vars::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef},
)
    cover_cons = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, ConstraintRef}()
    for request in requests
        s, _o, _d = request
        for pair in feasible_pairs[request]
            requires_no_vehicle_route(pair) && continue
            covering = [idx for (idx, column) in enumerate(columns) if pair in column.od_pairs]
            cover_cons[(request, pair)] =
                @constraint(m, sum(lambda[idx, s] for idx in covering; init = 0.0) >= assigned_vars[(request, pair)])
        end
    end
    return cover_cons
end

"""
    add_benders_route_coverage_constraints!(m, lambda, group_physical_pairs, group_occurrences, feasible_pairs_by_p, columns, h) -> cover_cons

`BendersYZH` variant: one `h`-covering row per `(physical_pair, pair)` *per scenario occurrence*
(`h` is scenario-compressed, so the same `h[(p,pair)]` feeds a coverage row in every scenario
`p` occurs in), re-keyed to the flat `(s,o,d), pair)` shape on output.
"""
function add_benders_route_coverage_constraints!(
    m::JuMP.Model,
    lambda,
    group_physical_pairs::Vector{Tuple{Int, Int}},
    group_occurrences::Dict{Tuple{Int, Int}, Vector{Int}},
    feasible_pairs_by_p::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    columns::Vector{AggregateODRouteColumn},
    h::Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, VariableRef},
)
    cover_cons = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, ConstraintRef}()
    for p in group_physical_pairs
        for pair in feasible_pairs_by_p[p]
            requires_no_vehicle_route(pair) && continue
            covering = [idx for (idx, column) in enumerate(columns) if pair in column.od_pairs]
            for s in group_occurrences[p]
                con = @constraint(m, sum(lambda[idx, s] for idx in covering; init = 0.0) >= h[(p, pair)])
                cover_cons[((s, p[1], p[2]), pair)] = con
            end
        end
    end
    return cover_cons
end
