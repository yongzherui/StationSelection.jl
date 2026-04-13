"""
Unmet-demand variables v_{jkts} for RouteFleetLimitModel.

v_{jkts} ∈ ℤ₊ absorbs any gap between passenger assignment and route capacity
when the route-linking constraint is expressed as an equality:

    Σ x_{pjks} = v_{jkts} + Σ_r α^r_{jkts}    ∀ j,k,t,s
"""

export add_v_jkts_variables!


"""
    add_v_jkts_variables!(m, data, mapping::FleetLimitODMap) -> Int

Add one unmet-demand variable v_{jkts} ∈ ℤ₊ for each (s, j_idx, k_idx, t_id)
combination that has at least one α^r_{jkts} variable (i.e., is reachable by
some route). Variables are stored in `m[:v_jkts]` keyed by NTuple{4,Int}.

Must be called after `add_alpha_r_jkts_variables!`.
"""
function add_v_jkts_variables!(
    m       :: Model,
    data    :: StationSelectionData,
    mapping :: FleetLimitODMap
)::Int
    before = JuMP.num_variables(m)

    # Collect unique (s, j_idx, k_idx, t_id) from existing alpha keys
    jkt_keys = Set{NTuple{4, Int}}()
    for (s, _r_idx, j_idx, k_idx, t_id) in keys(m[:alpha_r_jkts])
        push!(jkt_keys, (s, j_idx, k_idx, t_id))
    end

    v_jkts = Dict{NTuple{4, Int}, VariableRef}()
    for key in jkt_keys
        v_jkts[key] = @variable(m, integer = true, lower_bound = 0)
    end

    m[:v_jkts] = v_jkts
    return JuMP.num_variables(m) - before
end
