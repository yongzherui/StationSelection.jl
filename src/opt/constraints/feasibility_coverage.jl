export add_feasibility_coverage_constraints!

"""
    add_feasibility_coverage_constraints!(m, data, mapping, max_walking_distance) -> Int

Add per-station coverage constraints to ensure every station location j has at least one
active station within walking distance in each scenario:

    ∑_{k ∈ N_j} z[k,s] ≥ 1   ∀ j ∈ J, s ∈ S

N_j = {k : w(j,k) ≤ max_walking_distance}.  Since w(j,j) = 0, j ∈ N_j always so no
station location is ever isolated.  Returns the number of constraints added.
"""
function add_feasibility_coverage_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::NominalTwoStageODMap,
        max_walking_distance::Float64,
    )::Int
    before = _total_num_constraints(m)
    n = data.n_stations
    S = length(data.scenarios)

    for j in 1:n
        N_j = [k for k in 1:n if data.walking_costs[j, k] <= max_walking_distance]
        isempty(N_j) && continue
        for s in 1:S
            @constraint(m, sum(m[:z][k, s] for k in N_j) >= 1)
        end
    end

    return _total_num_constraints(m) - before
end
