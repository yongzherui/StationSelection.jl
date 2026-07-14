"""
Port of `scripts/test_case_generation/generate_test1_vehicle_cases.jl` — Test 1,
Vehicle Capacity Stress.

Test 1 hypothesis: vehicle capacity is fixed at 20 in the source script's
`ab30_m10` base profile; as capacity shrinks (20 -> 15 -> 10 -> 5) with fleet
size fixed at 2, consolidation (the pooled A->M0/M->B route) becomes
necessary to serve demand within the available capacity.

Geometry and demand are NOT regenerated here — this generator calls
`generate_middle_zone_benchmark_instance` in-process (replacing the original
script's `cp(...)` file-copy step) and only overwrites the vehicle fleet
config.

NOTE (open item, confirmed with the user): the original script also swept
fleet size (2..5) at fixed capacity=20. That sub-sweep is intentionally
dropped from this port — no model in `src/` currently enforces a fleet-size
cap (`RouteVehicleCapacityModel`/`RouteFleetLimitModel` referenced in
CLAUDE.md do not exist in the current codebase; only `ExactDARPRouteModel`
has a `vehicle_capacity` field, with no `Σθ≤F` constraint). Only the
capacity sub-sweep (fleet fixed at 2, capacity 20→5) is ported, since that is
checkable via `ExactDARPRouteModel(vehicle_capacity=...)`.
"""

# ---------------------------------------------------------------------------
# Fleet configs — capacity sub-sweep only (fleet-size sub-sweep dropped)
# ---------------------------------------------------------------------------

const T1_BASE_PROFILE = "ab30_m10"

const T1_CAPACITY_SWEEP_FLEET  = 2
const T1_CAPACITY_SWEEP_VALUES = [20, 15, 10, 5]

const T1_FLEET_CONFIGS = [
    (name = @sprintf("F%d_C%d", T1_CAPACITY_SWEEP_FLEET, c),
     fleet_size = T1_CAPACITY_SWEEP_FLEET,
     capacity   = c)
    for c in T1_CAPACITY_SWEEP_VALUES
]

const T1_HYPOTHESIS = (
    "Capacity sub-sweep: fleet=$(T1_CAPACITY_SWEEP_FLEET), capacity varies " *
    "$(T1_CAPACITY_SWEEP_VALUES). Smaller capacities favour the consolidated " *
    "A->M/M0->B route. Profile $(T1_BASE_PROFILE) uses light M-zone demand so " *
    "routing differences are driven by capacity scarcity, not zone saturation."
)

# ---------------------------------------------------------------------------
# Instance struct
# ---------------------------------------------------------------------------

"""
    T1Instance

Test 1 instance: geometry/demand copied unchanged from a
`MiddleZoneBenchmarkInstance`, with `fleet_size`/`capacity` overwritten per
config.
"""
struct T1Instance
    case_name::String
    seed_idx::Int
    seed::Int
    stations::DataFrame
    segments::DataFrame
    orders::DataFrame
    fleet_size::Int
    capacity::Int
    vehicle_speed::Float64
    base_profile::String
    demand_counts::NamedTuple
    suggested_l::Int
    suggested_k::Int
    hypothesis::String
    note::String
    extra::Dict{String,Any}
end

"""
    generate_test1_instance(fleet_config, seed_idx; base_profile=T1_BASE_PROFILE,
        vehicle_speed=25.0) -> T1Instance

`fleet_config` is a `(name, fleet_size, capacity)` NamedTuple, typically one
of `T1_FLEET_CONFIGS`.
"""
function generate_test1_instance(
    fleet_config::NamedTuple,
    seed_idx::Int;
    base_profile::String = T1_BASE_PROFILE,
    vehicle_speed::Float64 = 25.0,
)::T1Instance
    profile = only(filter(p -> p.name == base_profile, MZB_PROFILES))
    base = generate_middle_zone_benchmark_instance(
        base_profile, seed_idx, profile.lambda_AB, profile.lambda_MB,
    )

    note = "geometry and demand copied from source ($(base_profile)); only vehicle fleet config differs"

    return T1Instance(
        fleet_config.name, seed_idx, base.seed,
        base.stations, base.segments, base.orders,
        fleet_config.fleet_size, fleet_config.capacity, vehicle_speed,
        base_profile, base.demand_counts,
        base.suggested_l, base.suggested_k,
        T1_HYPOTHESIS, note,
        base.extra,
    )
end

"""
    build_test1_cases(; base_profile=T1_BASE_PROFILE, n_seeds=5,
        fleet_configs=T1_FLEET_CONFIGS) -> Vector{T1Instance}
"""
function build_test1_cases(;
    base_profile::String = T1_BASE_PROFILE,
    n_seeds::Int = 5,
    fleet_configs = T1_FLEET_CONFIGS,
)::Vector{T1Instance}
    instances = T1Instance[]
    for cfg in fleet_configs
        for seed_idx in 1:n_seeds
            push!(instances, generate_test1_instance(cfg, seed_idx; base_profile = base_profile))
        end
    end
    return instances
end

# ---------------------------------------------------------------------------
# StationSelectionData conversion
# ---------------------------------------------------------------------------

function create_test1_problem_data(instance::T1Instance; kwargs...)::StationSelectionData
    return tc_problem_data(instance.stations, instance.orders, instance.segments; kwargs...)
end

create_test1_station_selection_data(instance::T1Instance; kwargs...) = create_test1_problem_data(instance; kwargs...)

# ---------------------------------------------------------------------------
# Diagnostic printer
# ---------------------------------------------------------------------------

function print_test1_summary(instances::Vector{T1Instance})
    println("\nTest 1 — Vehicle Capacity Stress  (profile: $(T1_BASE_PROFILE))")
    println("  Hypothesis: ", T1_HYPOTHESIS)
    println("  " * "-"^54)
    @printf("  %-10s  %4s  %4s  %4s  %8s  %6s\n",
            "case", "seed", "F", "Cap", "stations", "orders")
    println("  " * "-"^54)
    for inst in instances
        @printf("  %-10s  %4d  %4d  %4d  %8d  %6d\n",
                inst.case_name, inst.seed_idx, inst.fleet_size, inst.capacity,
                nrow(inst.stations), nrow(inst.orders))
    end
    println("  " * "-"^54)
    println("  Total instances: $(length(instances))")
end
