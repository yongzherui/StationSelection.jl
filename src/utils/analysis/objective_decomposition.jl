"""
Post-hoc objective decomposition from exported optimization variables.

Reconstructs each term of the objective function from exported CSV files,
enabling per-term attribution without re-running the solver.
"""

using CSV
using DataFrames
using JSON
using Printf

export ObjectiveDecomposition, decompose_objective


"""
    ObjectiveDecomposition

Per-term attribution of the reported objective value, reconstructed from
exported CSV files (no solver re-run required).

Fields are grouped by component:
- Walking & routing: shared by all OD models
- Corridor penalty: ZCorridor / XCorridor / XCorridor+FR
- Flow regularisation: XCorridor+FR
- Vehicle routing: TSD only (0.0 for all other models)
- Totals: computed vs reported objective
"""
struct ObjectiveDecomposition
    model_type::String

    # --- Walking & passenger routing (all OD models) ---
    walking_cost::Float64                   # Σ q*(walk_o+walk_d)*x  [unweighted]
    routing_cost_raw::Float64               # Σ q*c_jk*x  [unweighted]
    in_vehicle_time_weight::Float64         # w_ivt parameter
    weighted_routing_cost::Float64          # w_ivt * routing_cost_raw

    # --- Corridor penalty (ZCorridor, XCorridor, XCorridor+FR) ---
    corridor_cost_raw::Float64              # Σ_{g,s} r_g*f_corridor[g,s]  [unweighted]
    corridor_weight::Float64                # γ (0.0 if no corridor model)
    corridor_penalty::Float64               # corridor_weight * corridor_cost_raw

    # --- Flow regularisation (XCorridor+FR) ---
    n_activated_routes::Int                 # Σ_s |{(j,k) active in s}|
    flow_activation_cost_raw::Float64      # Σ_{s,(j,k)} c_jk * f_flow[s][(j,k)]  [unweighted]
    flow_regularization_weight::Float64     # μ (0.0 if no FR)
    flow_regularization_penalty::Float64    # flow_regularization_weight * flow_activation_cost_raw

    # --- Vehicle routing (TSD only; 0.0 for all other models) ---
    vehicle_flow_cost_raw::Float64          # Σ c_jk*f_jk  [unweighted]
    same_source_savings_raw::Float64        # Σ savings from u variables  [unweighted]
    same_dest_savings_raw::Float64          # Σ savings from v variables  [unweighted]
    vehicle_routing_weight::Float64         # γ for TSD (0.0 otherwise)
    vehicle_routing_cost::Float64           # vehicle_routing_weight * vehicle_flow_cost_raw
    pooling_savings::Float64                # vehicle_routing_weight * (ss+sd savings)

    # --- Totals ---
    computed_total::Float64                 # sum of all weighted components
    reported_objective::Union{Float64, Nothing}  # from metrics.json (nothing if unavailable)
end


"""
    decompose_objective(run_dir::String, data::StationSelectionData) -> ObjectiveDecomposition

Reconstruct each component of the objective from exported CSVs in `run_dir`.

`run_dir` is the selection run directory (e.g. `runs/selection/2026-02-27_job1_.../`)
containing `metrics.json` and `variable_exports/`.

# Algorithm
1. Reads `metrics.json` for model type, weights, and reported objective.
2. Computes per-scenario OD counts from `data`.
3. Iterates `assignment_variables.csv` for walking cost, routing cost, and
   activated route count.
4. If `corridor_costs.csv` and `corridor_usage.csv` are present, accumulates
   the corridor penalty term.
5. If TSD flow/pooling CSVs are present, computes vehicle routing and pooling savings.
6. Returns a fully populated ObjectiveDecomposition.

Note: For TwoStageSingleDetourModel, walking and routing costs use per-scenario
OD counts rather than per-time-window counts; the computed_total may differ
slightly from the reported objective when the same OD pair appears in multiple
time windows within a scenario.
"""
function decompose_objective(run_dir::String, data::StationSelectionData)::ObjectiveDecomposition
    # Step 1: Read metrics.json
    metrics_file = joinpath(run_dir, "metrics.json")
    isfile(metrics_file) || error("metrics.json not found in: $run_dir")
    metrics = JSON.parsefile(metrics_file)

    model_type = get(metrics, "method", "Unknown")
    model_params = get(metrics, "model", Dict{String, Any}())

    in_vehicle_time_weight     = Float64(get(model_params, "in_vehicle_time_weight",     1.0))
    corridor_weight            = Float64(get(model_params, "corridor_weight",            0.0))
    flow_regularization_weight = Float64(get(model_params, "flow_regularization_weight", 0.0))
    vehicle_routing_weight     = Float64(get(model_params, "vehicle_routing_weight",     0.0))

    reported_objective = nothing
    solve_info = get(metrics, "solve", nothing)
    if !isnothing(solve_info)
        raw = get(solve_info, "objective_value", nothing)
        isnothing(raw) || (reported_objective = Float64(raw))
    end

    export_dir = joinpath(run_dir, "variable_exports")

    # Step 2: Per-scenario OD counts for legacy/export formats that do not store demand counts in x values.
    od_counts = build_od_counts_from_data(data)

    # Step 3: Assignment variables → walking cost, routing cost, activated routes
    walking_cost     = 0.0
    routing_cost_raw = 0.0
    activated_routes = Set{Tuple{Int, Int, Int}}()  # (pickup_id, dropoff_id, scenario)

    assign_file = joinpath(export_dir, "assignment_variables.csv")
    if isfile(assign_file) && filesize(assign_file) > 0
        assign = CSV.read(assign_file, DataFrame)
        if nrow(assign) > 0
            has_scenario_col = :scenario in propertynames(assign)
            for row in eachrow(assign)
                row.value > 0.5 || continue

                o_id = row.origin_id
                d_id = row.dest_id
                j_id = row.pickup_id
                k_id = row.dropoff_id
                s    = has_scenario_col ? row.scenario : 1

                q = if model_type == "ClusteringTwoStageODModel"
                    1
                elseif s <= length(od_counts)
                    get(od_counts[s], (o_id, d_id), 0)
                else
                    0
                end

                walk = get_walking_cost_by_id(data, o_id, j_id) + get_walking_cost_by_id(data, k_id, d_id)
                walking_cost     += walk * row.value * q
                routing_cost_raw += get_routing_cost_by_id(data, j_id, k_id) * row.value * q
                push!(activated_routes, (j_id, k_id, s))
            end
        end
    end

    n_activated_routes = length(activated_routes)

    # Step 4: Flow regularisation — cost-weighted route activations
    # f_flow is now tightly bounded (≥ and ≤ x-derived sums), so flow_activation.csv
    # correctly reflects activated routes even at frw=0.  Fall back to activated_routes
    # for older runs that pre-date the flow_activation export.
    flow_activation_cost_raw = 0.0
    route_file = joinpath(export_dir, "flow_activation.csv")
    if isfile(route_file) && filesize(route_file) > 0
        routes = CSV.read(route_file, DataFrame)
        for row in eachrow(routes)
            row.value > 0.5 || continue
            flow_activation_cost_raw += get_routing_cost_by_id(data, row.pickup_id, row.dropoff_id)
        end
    elseif !isempty(activated_routes)
        # Fallback: sum c_{jk} over distinct (j,k,s) triples from assignment variables
        for (j_id, k_id, _) in activated_routes
            flow_activation_cost_raw += get_routing_cost_by_id(data, j_id, k_id)
        end
    end

    # Step 5: Corridor penalty — requires corridor_costs.csv (written by Part 0 export fix)
    corridor_cost_raw = 0.0
    corridor_costs_file = joinpath(export_dir, "corridor_costs.csv")
    corridor_usage_file = joinpath(export_dir, "corridor_usage.csv")
    if isfile(corridor_costs_file) && isfile(corridor_usage_file)
        costs_df = CSV.read(corridor_costs_file, DataFrame)
        cost_lookup = Dict{Int, Float64}(
            row.corridor_idx => Float64(row.corridor_cost)
            for row in eachrow(costs_df)
        )
        usage_df = CSV.read(corridor_usage_file, DataFrame)
        for row in eachrow(usage_df)
            row.value > 0.5 || continue
            corridor_cost_raw += get(cost_lookup, row.corridor_idx, 0.0)
        end
    end

    # Step 6: TSD vehicle routing (only present for TwoStageSingleDetourModel)
    vehicle_flow_cost_raw   = 0.0
    same_source_savings_raw = 0.0
    same_dest_savings_raw   = 0.0

    flow_file = joinpath(export_dir, "flow_variables.csv")
    if isfile(flow_file) && filesize(flow_file) > 0
        flows = CSV.read(flow_file, DataFrame)
        for row in eachrow(flows)
            row.value > 0.5 || continue
            vehicle_flow_cost_raw += get_routing_cost_by_id(data, row.j_id, row.k_id) * row.value
        end
    end

    ss_file = joinpath(export_dir, "same_source_pooling.csv")
    if isfile(ss_file) && filesize(ss_file) > 0
        ss = CSV.read(ss_file, DataFrame)
        for row in eachrow(ss)
            row.value > 0.5 || continue
            # Savings = c(j,l) - c(k,l) when j detours via k to reach l
            saving = get_routing_cost_by_id(data, row.j_id, row.l_id) -
                     get_routing_cost_by_id(data, row.k_id, row.l_id)
            saving > 0 && (same_source_savings_raw += saving * row.value)
        end
    end

    sd_file = joinpath(export_dir, "same_dest_pooling.csv")
    if isfile(sd_file) && filesize(sd_file) > 0
        sd = CSV.read(sd_file, DataFrame)
        for row in eachrow(sd)
            row.value > 0.5 || continue
            # Savings = c(j,l) - c(j,k) when vehicle continues from k to l
            saving = get_routing_cost_by_id(data, row.j_id, row.l_id) -
                     get_routing_cost_by_id(data, row.j_id, row.k_id)
            saving > 0 && (same_dest_savings_raw += saving * row.value)
        end
    end

    # Step 7: Compute weighted components and total
    weighted_routing_cost       = in_vehicle_time_weight * routing_cost_raw
    corridor_penalty            = corridor_weight * corridor_cost_raw
    flow_regularization_penalty = flow_regularization_weight * flow_activation_cost_raw
    vehicle_routing_cost        = vehicle_routing_weight * vehicle_flow_cost_raw
    pooling_savings             = vehicle_routing_weight * (same_source_savings_raw + same_dest_savings_raw)

    computed_total = (walking_cost + weighted_routing_cost + corridor_penalty +
                      flow_regularization_penalty + vehicle_routing_cost - pooling_savings)

    return ObjectiveDecomposition(
        model_type,
        walking_cost,
        routing_cost_raw,
        in_vehicle_time_weight,
        weighted_routing_cost,
        corridor_cost_raw,
        corridor_weight,
        corridor_penalty,
        n_activated_routes,
        flow_activation_cost_raw,
        flow_regularization_weight,
        flow_regularization_penalty,
        vehicle_flow_cost_raw,
        same_source_savings_raw,
        same_dest_savings_raw,
        vehicle_routing_weight,
        vehicle_routing_cost,
        pooling_savings,
        computed_total,
        reported_objective
    )
end


function Base.show(io::IO, d::ObjectiveDecomposition)
    sep = "─" ^ 58
    println(io, "ObjectiveDecomposition ($(d.model_type))")
    println(io, sep)
    @printf io "  Walking cost:                  %14.2f\n" d.walking_cost
    @printf io "  Routing cost (raw):            %14.2f\n" d.routing_cost_raw
    @printf io "  Weighted routing (×%.2f):      %14.2f\n" d.in_vehicle_time_weight d.weighted_routing_cost
    if d.corridor_weight > 0.0 || d.corridor_cost_raw > 0.0
        @printf io "  Corridor cost raw:             %14.2f\n" d.corridor_cost_raw
        @printf io "  Corridor penalty (γ=%.2f):     %14.2f\n" d.corridor_weight d.corridor_penalty
    end
    if d.flow_regularization_weight > 0.0 || d.n_activated_routes > 0
        @printf io "  Flow activ. cost (raw):       %14.2f  [%d routes]\n" d.flow_activation_cost_raw d.n_activated_routes
        @printf io "  Flow reg. (μ=%.2f):            %14.2f\n" d.flow_regularization_weight d.flow_regularization_penalty
    end
    if d.vehicle_routing_weight > 0.0 || d.vehicle_flow_cost_raw > 0.0
        @printf io "  Vehicle flow cost raw:         %14.2f\n" d.vehicle_flow_cost_raw
        @printf io "  Vehicle routing (γ=%.2f):      %14.2f\n" d.vehicle_routing_weight d.vehicle_routing_cost
        @printf io "  Same-source savings (raw):     %14.2f\n" d.same_source_savings_raw
        @printf io "  Same-dest savings (raw):       %14.2f\n" d.same_dest_savings_raw
        @printf io "  Pooling savings (γ=%.2f):      %14.2f\n" d.vehicle_routing_weight d.pooling_savings
    end
    println(io, sep)
    @printf io "  Computed total:                %14.2f\n" d.computed_total
    if !isnothing(d.reported_objective)
        diff = d.computed_total - d.reported_objective
        match_str = abs(diff) < 1.0 ? "✓ match" : "✗ MISMATCH"
        @printf io "  Reported objective:            %14.2f  %s (diff = %.2f)\n" d.reported_objective match_str diff
    else
        println(io, "  Reported objective:            not available")
    end
end
