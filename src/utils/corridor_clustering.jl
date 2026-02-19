"""
Corridor clustering utilities for corridor models (ZCorridorODModel, XCorridorODModel).

Clusters stations by routing distance with a diameter constraint, then computes
corridor data (inter-cluster movement indices and costs).
"""

export cluster_stations_by_diameter, compute_cluster_diameter, compute_corridor_data

"""
    compute_cluster_diameter(station_indices::Vector{Int}, data::StationSelectionData,
                             array_idx_to_station_id::Vector{Int}) -> Float64

Compute the maximum pairwise routing distance within a cluster.

# Arguments
- `station_indices`: Array indices of stations in the cluster
- `data`: Problem data with routing costs
- `array_idx_to_station_id`: Mapping from array index to station ID

# Returns
- Maximum pairwise routing distance (diameter), or 0.0 for single-station clusters
"""
function compute_cluster_diameter(
        station_indices::Vector{Int},
        data::StationSelectionData,
        array_idx_to_station_id::Vector{Int}
    )::Float64
    n = length(station_indices)
    n <= 1 && return 0.0

    max_dist = 0.0
    for i in 1:n
        for j in (i+1):n
            id_i = array_idx_to_station_id[station_indices[i]]
            id_j = array_idx_to_station_id[station_indices[j]]
            d = get_routing_cost(data, id_i, id_j)
            if d > max_dist
                max_dist = d
            end
        end
    end
    return max_dist
end

"""
    _kmedoids_milp(n_stations::Int, data::StationSelectionData,
                   array_idx_to_station_id::Vector{Int},
                   max_diameter::Float64;
                   optimizer_env=nothing) -> (cluster_labels, medoids, n_clusters)

Solve an exact MILP to find the minimum number of clusters such that the maximum
pairwise routing distance within any cluster is ≤ max_diameter.

# Formulation
- `m[j]` ∈ {0,1}: whether station j is a medoid (cluster center)
- `x[i,j]` ∈ {0,1}: whether station i is assigned to medoid j
- Minimize Σ_j m[j] (number of clusters)
- Subject to:
  - Each station assigned to exactly one medoid
  - Can only assign to a station that is a medoid
  - Diameter constraint: if d[i1,i2] > D, then i1 and i2 cannot be in the same cluster
"""
function _kmedoids_milp(
        n_stations::Int,
        data::StationSelectionData,
        array_idx_to_station_id::Vector{Int},
        max_diameter::Float64;
        optimizer_env=nothing
    )
    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    model = Model(() -> Gurobi.Optimizer(optimizer_env))
    set_silent(model)

    # Variables
    @variable(model, m_var[1:n_stations], Bin)   # m[j] = 1 if j is a medoid
    @variable(model, x[1:n_stations, 1:n_stations], Bin)  # x[i,j] = 1 if i assigned to j

    # Objective: minimize number of medoids
    @objective(model, Min, sum(m_var[j] for j in 1:n_stations))

    # Each station assigned to exactly one medoid
    for i in 1:n_stations
        @constraint(model, sum(x[i, j] for j in 1:n_stations) == 1)
    end

    # Can only assign to a medoid
    for i in 1:n_stations, j in 1:n_stations
        @constraint(model, x[i, j] <= m_var[j])
    end

    # A medoid must be assigned to itself
    for j in 1:n_stations
        @constraint(model, x[j, j] >= m_var[j])
    end

    # Diameter constraint: if d[i1,i2] > D, they cannot be in the same cluster
    for i1 in 1:n_stations
        id_i1 = array_idx_to_station_id[i1]
        for i2 in (i1+1):n_stations
            id_i2 = array_idx_to_station_id[i2]
            d = get_routing_cost(data, id_i1, id_i2)
            if d > max_diameter
                for j in 1:n_stations
                    @constraint(model, x[i1, j] + x[i2, j] <= 1)
                end
            end
        end
    end

    optimize!(model)

    # Extract solution
    m_val = value.(m_var)
    x_val = value.(x)

    medoid_indices = [j for j in 1:n_stations if m_val[j] > 0.5]
    n_clusters = length(medoid_indices)

    # Map medoid array indices to 1-based cluster labels
    medoid_to_cluster = Dict(medoid_indices[c] => c for c in 1:n_clusters)

    cluster_labels = Vector{Int}(undef, n_stations)
    for i in 1:n_stations
        for j in medoid_indices
            if x_val[i, j] > 0.5
                cluster_labels[i] = medoid_to_cluster[j]
                break
            end
        end
    end

    return cluster_labels, medoid_indices, n_clusters
end

"""
    cluster_stations_by_diameter(data::StationSelectionData,
                                 array_idx_to_station_id::Vector{Int},
                                 max_diameter::Float64;
                                 optimizer_env=nothing)
        -> (cluster_labels, medoids, n_clusters)

Find the minimum number of clusters such that the maximum intra-cluster
routing distance diameter ≤ max_diameter, using an exact MILP formulation.

# Arguments
- `data`: Problem data with routing costs
- `array_idx_to_station_id`: Mapping from array index to station ID
- `max_diameter`: Maximum allowed intra-cluster routing distance diameter
- `optimizer_env`: Optional Gurobi environment (created if not provided)

# Returns
- `cluster_labels::Vector{Int}`: Station array index → cluster label (1-based)
- `medoids::Vector{Int}`: Array indices of medoid stations
- `n_clusters::Int`: Number of clusters found
"""
function cluster_stations_by_diameter(
        data::StationSelectionData,
        array_idx_to_station_id::Vector{Int},
        max_diameter::Float64;
        optimizer_env=nothing
    )
    n = data.n_stations
    return _kmedoids_milp(n, data, array_idx_to_station_id, max_diameter; optimizer_env=optimizer_env)
end

"""
    compute_corridor_data(cluster_labels::Vector{Int}, medoids::Vector{Int},
                          n_clusters::Int, data::StationSelectionData,
                          array_idx_to_station_id::Vector{Int})
        -> (corridor_indices, cluster_station_sets, corridor_costs)

Compute corridor indices and costs between all pairs of clusters.

A corridor g = (a, b) represents movement between cluster a and cluster b.
Total corridors = n_clusters² (including self-corridors where a == b).

# Returns
- `corridor_indices::Vector{Tuple{Int,Int}}`: g → (cluster_a, cluster_b)
- `cluster_station_sets::Vector{Vector{Int}}`: cluster_id → station array indices
- `corridor_costs::Vector{Float64}`: r_g = routing distance between medoids
"""
function compute_corridor_data(
        cluster_labels::Vector{Int},
        medoids::Vector{Int},
        n_clusters::Int,
        data::StationSelectionData,
        array_idx_to_station_id::Vector{Int}
    )
    # Build cluster station sets
    cluster_station_sets = [Int[] for _ in 1:n_clusters]
    for (i, c) in enumerate(cluster_labels)
        push!(cluster_station_sets[c], i)
    end

    # Build corridor indices: all pairs (a, b) for a in 1:n_clusters, b in 1:n_clusters
    corridor_indices = Tuple{Int,Int}[]
    corridor_costs = Float64[]

    for a in 1:n_clusters
        for b in 1:n_clusters
            push!(corridor_indices, (a, b))
            # r_g = routing distance between medoids
            id_a = array_idx_to_station_id[medoids[a]]
            id_b = array_idx_to_station_id[medoids[b]]
            push!(corridor_costs, get_routing_cost(data, id_a, id_b))
        end
    end

    return corridor_indices, cluster_station_sets, corridor_costs
end
