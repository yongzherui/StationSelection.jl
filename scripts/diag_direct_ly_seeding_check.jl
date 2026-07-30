"""
    scripts/diag_direct_ly_seeding_check.jl

Directly checks whether :direct_ly's lp_bound=0.0 at zhuzhou_n20_p16_s123/ms4 is a missing-
seed-columns problem or a genuine LP degeneracy: (1) counts the default singleton columns
seeded at build time (same for every assignment_policy -- create_map doesn't depend on it),
(2) solves the :direct_ly LP relaxation once directly (not via CG) with those seed columns
already present, and (3) evaluates every (NP) row's RHS at the LP's own optimal y to see
whether it's genuinely <= 0 everywhere despite routes being available.
"""

using DataFrames, Gurobi, JuMP, Printf, StationSelection
const MOI = JuMP.MOI

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))

function build_data()
    data, meta = generate_zhuzhou_data(DATA_DIR, 20, 16; n_scenarios=1, endpoint_overlap=2.0, seed=123)
    return data
end

function model_for(style::Symbol)
    return AggregateODRouteModel(
        10;
        assignment_policy=NearestOpenAggregateODAssignmentPolicy(style),
        max_walking_distance=600.0,
        route_regularization_weight=10.0,
        walk_cost_weight=0.0,
        repositioning_time=20.0,
        max_stops=4,
        max_wait_time=900.0,
        detour_factor=2.0,
    )
end

env = Gurobi.Env()
data = build_data()

model = model_for(:direct_ly)
mapping = StationSelection.create_map(model, data)
println("=== Seeding check ===")
println("n_stations=", data.n_stations, "  seeded initial columns (singletons)=", length(mapping.columns))
println("sample columns: ", first(mapping.columns, 5))

println("\n=== Direct LP relaxation solve (:direct_ly, seeded columns only, no pricing) ===")
build = StationSelection.build_model(model, data; optimizer_env=env, relax_integrality=true)
m = build.model
set_silent(m)
optimize!(m)
@printf("termination_status=%s objective=%.6f\n", termination_status(m), objective_value(m))

y_val = value.(m[:y])
open_frac = [(j, y_val[j]) for j in eachindex(y_val) if y_val[j] > 1e-6]
println("nonzero y (station_idx, value), n=", length(open_frac), ":")
for (j, v) in open_frac
    @printf("  y[%d] = %.4f\n", j, v)
end

println("\n=== Evaluate every (NP) row's slack (expr - rhs) at this LP-optimal y ===")
coverage = m[:aggregate_od_route_coverage_constraints]
theta = m[:theta_compat]
n_active_theta = count(v -> value(v) > 1e-6, values(theta))
println("theta variables with value > 1e-6: ", n_active_theta, " / ", length(theta), " total theta vars in pool")

slacks = [value(con) for (key, con) in coverage]  # normalized `expr - rhs >= 0` -> value(con) = expr - rhs
@printf("slack stats over %d coverage rows: min=%.6f max=%.6f mean=%.6f\n",
    length(slacks), minimum(slacks), maximum(slacks), sum(slacks) / length(slacks))
# A row's implied RHS is `expr - slack`; since expr = sum of active theta (0 here if n_active_theta==0),
# slack == -rhs whenever no column covers that pair. So printing max(-slack) directly shows the
# largest RHS(y) actually attained across every (NP) row.
implied_rhs = [-s for s in slacks]
@printf("implied RHS(y) stats: min=%.6f max=%.6f (max<=~0 means the LP never needed ANY route)\n",
    minimum(implied_rhs), maximum(implied_rhs))
