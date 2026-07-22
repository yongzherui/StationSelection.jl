"""
Port of `scripts/test_case_generation/generate_middle_zone_benchmark.jl`.

Minimal middle-zone benchmark geometry (Case C2: north-shifted):

  A (-2,0) --- M0 (0,0) --- B (2,0)
               M  (0,h)       h=0.8 km, off-corridor

8 stations total: terminals A/B, on-corridor VBS M0, off-corridor VBS M, and
four zone demand origins p1-p4 clustered around M. `Test 1` (`test1_vehicle.jl`)
depends on this generator for its geometry/demand and layers a different
vehicle fleet config on top.

Suggested sweep (per the source script): l=4, k=3 (build 4 of 8; model
selects {A, M or M0, B}).
"""

# ---------------------------------------------------------------------------
# Fixed geometry
# ---------------------------------------------------------------------------

const MZB_H_KM = 0.8  # km — north offset of off-corridor VBS stop M

const MZB_VBS_STATIONS = [
    (id = 1, name = "A",  x_km = -2.00, y_km = 0.00,      role = "terminal"),
    (id = 2, name = "M0", x_km = 0.00,  y_km = 0.00,      role = "on_corridor"),
    (id = 3, name = "M",  x_km = 0.00,  y_km = MZB_H_KM,  role = "off_corridor"),
    (id = 4, name = "B",  x_km = 2.00,  y_km = 0.00,      role = "terminal"),
]

const MZB_ZONE_ORIGINS = [
    (id = 5, name = "p1", x_km = -0.20, y_km = MZB_H_KM + 0.15, role = "zone_origin"),
    (id = 6, name = "p2", x_km = 0.20,  y_km = MZB_H_KM + 0.15, role = "zone_origin"),
    (id = 7, name = "p3", x_km = -0.15, y_km = MZB_H_KM - 0.15, role = "zone_origin"),
    (id = 8, name = "p4", x_km = 0.15,  y_km = MZB_H_KM - 0.15, role = "zone_origin"),
]

const MZB_ALL_STATIONS = [MZB_VBS_STATIONS; MZB_ZONE_ORIGINS]

const MZB_SCENARIO_DATE   = Date(2026, 1, 1)
const MZB_WINDOW_START    = Time(8, 0, 0)
const MZB_WINDOW_HOURS    = 3
const MZB_WINDOW_SEC      = MZB_WINDOW_HOURS * 3600
const MZB_WINDOW_START_DT = DateTime(MZB_SCENARIO_DATE, MZB_WINDOW_START)
const MZB_WINDOW_END_DT   = MZB_WINDOW_START_DT + Second(MZB_WINDOW_SEC - 1)

const MZB_SEED_BASE = 42

# Demand profiles: profile name → (lambda_AB, lambda_MB) orders/hr
const MZB_PROFILES = [
    (name = "ab15_m10", lambda_AB = 15, lambda_MB = 10),
    (name = "ab15_m30", lambda_AB = 15, lambda_MB = 30),
    (name = "ab30_m10", lambda_AB = 30, lambda_MB = 10),
    (name = "ab30_m30", lambda_AB = 30, lambda_MB = 30),
    (name = "ab30_m60", lambda_AB = 30, lambda_MB = 60),
    (name = "ab60_m30", lambda_AB = 60, lambda_MB = 30),
    (name = "ab60_m60", lambda_AB = 60, lambda_MB = 60),
]

const MZB_SUGGESTED_L = 4
const MZB_SUGGESTED_K = 3

const MZB_HYPOTHESIS = (
    "Fixed geometry (A, M0, M, B + zone origins p1-p4), varies demand profile " *
    "(lambda_AB, lambda_MB). l=4, k=3: build 4 of 8 stations, activate 3 per " *
    "scenario. The model naturally selects {A, M_or_M0, B} -- p1-p4 are " *
    "dominated as service stations. Route costs at 8 m/s: A->M0->B = 500 s " *
    "(straight), A->M->B = 538 s (detour, Delta = 38.5 s)."
)

# ---------------------------------------------------------------------------
# Instance struct
# ---------------------------------------------------------------------------

"""
    MiddleZoneBenchmarkInstance

One (profile, seed) instance of the middle-zone benchmark geometry. Test 1's
`T1Instance` is built on top of this in-process (see `test1_vehicle.jl`).
"""
struct MiddleZoneBenchmarkInstance
    profile_name::String
    seed_idx::Int
    seed::Int
    stations::DataFrame
    segments::DataFrame
    orders::DataFrame
    lambda_AB::Int
    lambda_MB::Int
    n_vehicles::Int
    vehicle_capacity::Int
    vehicle_speed::Float64
    demand_counts::NamedTuple
    suggested_l::Int
    suggested_k::Int
    hypothesis::String
    extra::Dict{String,Any}
end

"""
    generate_middle_zone_benchmark_instance(profile_name, seed_idx, lambda_AB, lambda_MB;
        seed_base=42, n_vehicles=6, vehicle_capacity=30, vehicle_speed=25.0)
        -> MiddleZoneBenchmarkInstance

Faithful in-memory port of the original script's `generate_instance`, minus
all file I/O.
"""
function generate_middle_zone_benchmark_instance(
    profile_name::String,
    seed_idx::Int,
    lambda_AB::Int,
    lambda_MB::Int;
    seed_base::Int = MZB_SEED_BASE,
    n_vehicles::Int = 6,
    vehicle_capacity::Int = 30,
    vehicle_speed::Float64 = 25.0,
)::MiddleZoneBenchmarkInstance
    seed = seed_base + seed_idx
    rng  = MersenneTwister(seed)

    station_df = tc_build_station_df(MZB_ALL_STATIONS)
    segment_df = tc_build_segment_df(MZB_ALL_STATIONS; veh_speed = 8.0)

    A = MZB_VBS_STATIONS[1]
    B = MZB_VBS_STATIONS[4]
    order_rows = NamedTuple[]
    order_id = Ref(1)

    # Stream 1: A -> B
    n_AB = tc_poisson_draw(lambda_AB * MZB_WINDOW_HOURS, rng)
    for ts in tc_rand_timestamps(n_AB, rng, MZB_WINDOW_START_DT, MZB_WINDOW_SEC)
        tc_push_order!(order_rows, order_id, A.id, B.id, ts)
    end

    # Stream 2: M-zone -> B (round-robin through zone origins)
    n_MB = tc_poisson_draw(lambda_MB * MZB_WINDOW_HOURS, rng)
    for (i, ts) in enumerate(tc_rand_timestamps(n_MB, rng, MZB_WINDOW_START_DT, MZB_WINDOW_SEC))
        origin = MZB_ZONE_ORIGINS[mod1(i, length(MZB_ZONE_ORIGINS))]
        tc_push_order!(order_rows, order_id, origin.id, B.id, ts)
    end

    order_df = DataFrame(order_rows)
    sort!(order_df, :order_time)

    demand_counts = (A_to_B = n_AB, Mzone_to_B = n_MB, n_total = nrow(order_df))

    extra = Dict{String,Any}(
        "h_km" => MZB_H_KM,
        "veh_speed_ms" => 8.0,
        "walk_speed_ms" => 1.4,
        "window" => Dict(
            "start" => "2026-01-01 08:00:00",
            "end" => Dates.format(MZB_WINDOW_END_DT, "yyyy-mm-dd HH:MM:SS"),
            "duration_sec" => MZB_WINDOW_SEC,
        ),
    )

    return MiddleZoneBenchmarkInstance(
        profile_name, seed_idx, seed,
        station_df, segment_df, order_df,
        lambda_AB, lambda_MB,
        n_vehicles, vehicle_capacity, vehicle_speed,
        demand_counts,
        MZB_SUGGESTED_L, MZB_SUGGESTED_K,
        MZB_HYPOTHESIS,
        extra,
    )
end

"""
    build_middle_zone_benchmark_cases(; n_seeds=5, profiles=MZB_PROFILES) -> Vector{MiddleZoneBenchmarkInstance}

Generates all (profile, seed) combinations.
"""
function build_middle_zone_benchmark_cases(; n_seeds::Int = 5, profiles = MZB_PROFILES)
    instances = MiddleZoneBenchmarkInstance[]
    for profile in profiles
        for seed_idx in 1:n_seeds
            push!(instances, generate_middle_zone_benchmark_instance(
                profile.name, seed_idx, profile.lambda_AB, profile.lambda_MB,
            ))
        end
    end
    return instances
end

# ---------------------------------------------------------------------------
# StationSelectionData conversion
# ---------------------------------------------------------------------------

function create_middle_zone_problem_data(instance::MiddleZoneBenchmarkInstance; kwargs...)::StationSelectionData
    return tc_problem_data(instance.stations, instance.orders, instance.segments; kwargs...)
end

create_middle_zone_station_selection_data(instance::MiddleZoneBenchmarkInstance; kwargs...) =
    create_middle_zone_problem_data(instance; kwargs...)

# ---------------------------------------------------------------------------
# Diagnostic printer
# ---------------------------------------------------------------------------

function print_middle_zone_summary(instances::Vector{MiddleZoneBenchmarkInstance})
    println("\nMiddle-Zone Benchmark")
    println("  Hypothesis: ", MZB_HYPOTHESIS)
    println("  " * "-"^62)
    @printf("  %-10s  %4s  %6s  %6s  %6s  %5s  %4s\n",
            "profile", "seed", "l_AB", "l_MB", "orders", "A→B", "M→B")
    println("  " * "-"^62)
    for inst in instances
        @printf("  %-10s  %4d  %6d  %6d  %6d  %5d  %4d\n",
                inst.profile_name, inst.seed_idx, inst.lambda_AB, inst.lambda_MB,
                inst.demand_counts.n_total, inst.demand_counts.A_to_B, inst.demand_counts.Mzone_to_B)
    end
    println("  " * "-"^62)
    println("  Total instances : $(length(instances))")
end
