# =============================================================================
# ClusteringModel
# =============================================================================

function build_model(
        model::ClusteringModel,
        data::StationSelectionData;
        optimizer_env=nothing
    )::BuildResult

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    mapping = create_map(model, data)

    m = Model(() -> Gurobi.Optimizer(optimizer_env))

    variable_counts = Dict{String, Int}()
    constraint_counts = Dict{String, Int}()
    extra_counts = Dict{String, Int}()

    _build_clustering!(m, data, mapping, model.policy, variable_counts, constraint_counts, extra_counts)

    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end
