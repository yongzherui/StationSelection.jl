"""
Per-request pair-assignment constraint builders shared by Benders master and subproblem
builders (`src/opt/optimize/aggregate_od_route/benders/{y,xy}.jl`) for the two non-nearest-open
resolution styles: `:pair_chain`'s ranked dominance ordering, and `BendersXY`'s fully free
(unranked, no dominance) master assignment. (The nearest-open styles' pair assignment already
lives in `_add_nearest_open_pair_assignment!`, `constraints/aggregate_od_route/core.jl`.)
"""

"""
    add_ranked_pair_assignment_constraints!(m, data, y, request, pairs; binary=false) -> (x_by_pair, sum_con)

`:pair_chain`'s dominance-ranking encoding: `pairs` ranked cheapest-first
(`_ranked_request_pairs`), each pair linked to its own stations' `y` and dominated by every
strictly-cheaper prior pair (`x[pair] <= 2 - y[pj] - y[pk]`) so only the cheapest currently-open
pair can be selected. `binary=true` for `BendersXY`'s master (only `y` needs to be genuinely
integer there, see `_add_nearest_open_endpoint_master_x!`'s docstring); `binary=false` (default)
for `BendersY`'s subproblem LP, where `x` is a continuous relaxation.
"""
function add_ranked_pair_assignment_constraints!(
    m::JuMP.Model,
    data::StationSelectionData,
    y,
    request::NTuple{3, Int},
    pairs::Vector{Tuple{Int, Int}};
    binary::Bool=false,
)
    ranked = _ranked_request_pairs(data, request, pairs)
    x_by_pair = Dict{Tuple{Int, Int}, VariableRef}()
    for pair in ranked
        x_by_pair[pair] = binary ? @variable(m, binary = true) : @variable(m, lower_bound = 0.0, upper_bound = 1.0)
    end
    sum_con = @constraint(m, sum(x_by_pair[pair] for pair in ranked) == 1.0)
    for (rank_idx, pair) in enumerate(ranked)
        j, k = pair
        @constraint(m, x_by_pair[pair] <= y[j])
        @constraint(m, x_by_pair[pair] <= y[k])
        for prior in ranked[1:max(rank_idx - 1, 0)]
            pj, pk = prior
            @constraint(m, x_by_pair[pair] <= 2.0 - y[pj] - y[pk])
        end
    end
    return x_by_pair, sum_con
end

"""
    add_unlinked_pair_assignment_constraints!(m, request, pairs) -> (x_by_pair, sum_con)

`BendersYZ`'s subproblem `x`: continuous `[0,1]` per feasible pair, `sum == 1`, no `y`/`z`
linking here -- the caller links each `x_by_pair` externally to the fixed `z` chain
(`_add_endpoint_x_linking!`, `constraints/aggregate_od_route/core.jl`), since `z` (not `y`) is
what's fixed in this decomposition's subproblem.
"""
function add_unlinked_pair_assignment_constraints!(
    m::JuMP.Model,
    request::NTuple{3, Int},
    pairs::Vector{Tuple{Int, Int}},
)
    x_by_pair = Dict{Tuple{Int, Int}, VariableRef}()
    for pair in pairs
        x_by_pair[pair] = @variable(m, lower_bound = 0.0, upper_bound = 1.0)
    end
    sum_con = @constraint(m, sum(x_by_pair[pair] for pair in pairs; init = 0.0) == 1.0)
    return x_by_pair, sum_con
end

"""
    add_free_pair_assignment_constraints!(m, y, request, pairs) -> (x_by_pair, sum_con)

`BendersXY`'s free-assignment master (`assignment_policy` not `NearestOpenAggregateODAssignmentPolicy`):
one binary `x` per feasible pair, no dominance ranking, linked to `y` only for non-walk-only
pairs.
"""
function add_free_pair_assignment_constraints!(
    m::JuMP.Model,
    y,
    request::NTuple{3, Int},
    pairs::Vector{Tuple{Int, Int}},
)
    isempty(pairs) && throw(ArgumentError("BendersXY master has no feasible station pair for $(request)"))
    x_by_pair = Dict{Tuple{Int, Int}, VariableRef}()
    for pair in pairs
        var = @variable(m, binary = true)
        x_by_pair[pair] = var
        if !is_walk_only_pair(pair)
            j, k = pair
            @constraint(m, var <= y[j])
            @constraint(m, var <= y[k])
        end
    end
    sum_con = @constraint(m, sum(x_by_pair[pair] for pair in pairs) == 1.0)
    return x_by_pair, sum_con
end
