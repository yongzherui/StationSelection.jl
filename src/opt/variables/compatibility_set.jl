"""
Variables for CompatibilitySetModel.
"""

export add_od_activation_variables!
export add_compatibility_theta_variables!

function add_od_activation_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::CompatibilitySetODMap;
    relax_integrality::Bool=false,
)::Int
    before = JuMP.num_variables(m)
    u = Dict{NTuple{3, Int}, VariableRef}()
    for s in 1:n_scenarios(data)
        for (j, k) in get(mapping.active_jk_s, s, Tuple{Int, Int}[])
            if relax_integrality
                u[(j, k, s)] = @variable(m, lower_bound = 0.0, upper_bound = 1.0)
            else
                u[(j, k, s)] = @variable(m, binary = true)
            end
        end
    end
    m[:u] = u
    return JuMP.num_variables(m) - before
end

function add_compatibility_theta_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::CompatibilitySetODMap;
    relax_integrality::Bool=false,
)::Int
    before = JuMP.num_variables(m)
    theta = Dict{Tuple{Int, Int}, VariableRef}()
    for column in mapping.columns
        for s in 1:n_scenarios(data)
            if relax_integrality
                theta[(column.id, s)] = @variable(m, lower_bound = 0.0, upper_bound = 1.0)
            else
                theta[(column.id, s)] = @variable(m, binary = true)
            end
        end
    end
    m[:theta_compat] = theta
    return JuMP.num_variables(m) - before
end
