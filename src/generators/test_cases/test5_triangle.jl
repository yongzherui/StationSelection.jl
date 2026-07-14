"""
Port of `scripts/test_case_generation/generate_test5_triangle_cases.jl` —
Test 5, Equilateral Triangle vs Corridor Baseline.

Compares two structural geometries with identical corridor endpoints A/B,
fleet, and demand parameters:

  `:corridor_base` — classic collinear arrangement (A, M0, B), zone above
  M0. A→M0→B is the direct route; no detour cost.

  `:equilateral` — A, M, B form an equilateral triangle (side = 3 km); M0 =
  midpoint(A,M) is the consolidation stop on segment A→M. Genuine detour
  cost: A→M0→B costs +137 s vs the direct A→B route.

  `:equilateral_with_m1` — same triangle, adds a second intermediate M1 on
  segment M0→B.

Each geometry variant is crossed with two demand configs (`ab30_mb10`
corridor-dominant, `ab20_mb20` balanced) — 6 sub-cases total.

Walking threshold: 1.8 km throughout (not the 1.0 km default used
elsewhere — kept local to this file, not hoisted to `common.jl`).

Suggested sweeps (per the source script, `generate_test5_triangle_cases.jl:754-757`):
  corridor_base        — l=3, k=2  (A, M0, B)
  equilateral_triangle — l=4, k=3  (A, M0, M, B)
  equilateral_with_m1  — l=5, k≤4  (A, M0, M1, M, B; activate subset) — k=4 used here
"""

# ---------------------------------------------------------------------------
# Geometry constants
# ---------------------------------------------------------------------------

const T5_VEH_SPEED  = 8.0
const T5_WALK_SPEED = 1.4
const T5_WALK_THRESHOLD_KM = 1.8

const T5_SIDE_KM  = 3.0
const T5_A_X_KM   = -T5_SIDE_KM / 2
const T5_B_X_KM   = +T5_SIDE_KM / 2

const T5_A_BASE_X_KM = -3.0
const T5_B_BASE_X_KM = +3.0

const T5_M_X_KM = 0.0
const T5_M_Y_KM = T5_SIDE_KM * sqrt(3.0) / 2  # ≈ 2.5981 km

const T5_M0_T    = 1 / 2
const T5_M0_X_KM = T5_A_X_KM + T5_M0_T * (T5_M_X_KM - T5_A_X_KM)  # -0.75 km
const T5_M0_Y_KM = T5_M0_T * T5_M_Y_KM                              # ≈ 1.2990 km

const T5_M1_T    = 0.20
const T5_M1_X_KM = T5_M0_X_KM + T5_M1_T * (T5_B_X_KM - T5_M0_X_KM)  # ≈ -0.30 km
const T5_M1_Y_KM = T5_M0_Y_KM + T5_M1_T * (0.0 - T5_M0_Y_KM)         # ≈  1.039 km

const T5_H_BASE_KM = 0.8  # zone height for corridor_base

const T5_LAMBDA_AB    = 30
const T5_WINDOW_HOURS = 3
const T5_WINDOW_SEC   = T5_WINDOW_HOURS * 3600

const T5_SCENARIO_DATE   = Date(2026, 1, 1)
const T5_WINDOW_START_DT = DateTime(T5_SCENARIO_DATE, Time(8, 0, 0))
const T5_WINDOW_END_DT   = T5_WINDOW_START_DT + Second(T5_WINDOW_SEC - 1)

const T5_N_VEHICLES       = 6
const T5_VEHICLE_CAPACITY = 30
const T5_VEHICLE_SPEED    = 25.0

const T5_SEED_BASE = 42

const T5_CASES = [:corridor_base, :equilateral, :equilateral_with_m1]

const T5_DEMAND_CONFIGS = [
    (lambda_AB = 30, lambda_MB = 10, label = "ab30_mb10"),
    (lambda_AB = 20, lambda_MB = 20, label = "ab20_mb20"),
]

const T5_SWEEPS = Dict(
    :corridor_base       => (l = 3, k = 2),
    :equilateral         => (l = 4, k = 3),
    :equilateral_with_m1 => (l = 5, k = 4),
)

function t5_case_label(case::Symbol)::String
    case == :corridor_base       && return "corridor_base"
    case == :equilateral         && return "equilateral_triangle"
    case == :equilateral_with_m1 && return "equilateral_with_m1"
    error("Unknown Test 5 case: $case")
end

# ---------------------------------------------------------------------------
# Geometry construction
# ---------------------------------------------------------------------------

"""
    t5_build_stations(case) -> (all_stations, vbs_stations, zone_origins, zone_center)

ID layout:
  corridor_base        — A(1) M0(2) B(3) p1-p4(4-7)              7 stations
  equilateral           — A(1) M0(2) M(3) B(4) p1-p4(5-8)         8 stations
  equilateral_with_m1   — A(1) M0(2) M1(3) M(4) B(5) p1-p4(6-9)  9 stations
"""
function t5_build_stations(case::Symbol)
    if case == :corridor_base
        zone_cx, zone_cy = 0.0, T5_H_BASE_KM
        vbs = [
            (id = 1, name = "A",  x_km = T5_A_BASE_X_KM, y_km = 0.0, role = "terminal"),
            (id = 2, name = "M0", x_km = 0.0,             y_km = 0.0, role = "on_corridor"),
            (id = 3, name = "B",  x_km = T5_B_BASE_X_KM, y_km = 0.0, role = "terminal"),
        ]
        B_id = 3
    elseif case == :equilateral
        zone_cx, zone_cy = T5_M_X_KM, T5_M_Y_KM
        vbs = [
            (id = 1, name = "A",  x_km = T5_A_X_KM,  y_km = 0.0,        role = "terminal"),
            (id = 2, name = "M0", x_km = T5_M0_X_KM, y_km = T5_M0_Y_KM, role = "off_corridor"),
            (id = 3, name = "M",  x_km = T5_M_X_KM,  y_km = T5_M_Y_KM,  role = "off_corridor"),
            (id = 4, name = "B",  x_km = T5_B_X_KM,  y_km = 0.0,        role = "terminal"),
        ]
        B_id = 4
    else  # :equilateral_with_m1
        zone_cx, zone_cy = T5_M_X_KM, T5_M_Y_KM
        vbs = [
            (id = 1, name = "A",  x_km = T5_A_X_KM,  y_km = 0.0,        role = "terminal"),
            (id = 2, name = "M0", x_km = T5_M0_X_KM, y_km = T5_M0_Y_KM, role = "off_corridor"),
            (id = 3, name = "M1", x_km = T5_M1_X_KM, y_km = T5_M1_Y_KM, role = "off_corridor"),
            (id = 4, name = "M",  x_km = T5_M_X_KM,  y_km = T5_M_Y_KM,  role = "off_corridor"),
            (id = 5, name = "B",  x_km = T5_B_X_KM,  y_km = 0.0,        role = "terminal"),
        ]
        B_id = 5
    end

    zone = [
        (id = B_id + i, name = off.name,
         x_km = zone_cx + off.dx, y_km = zone_cy + off.dy, role = "zone_origin")
        for (i, off) in enumerate(TC_ZONE_OFFSETS_KM)
    ]

    return [vbs; zone], vbs, zone, (zone_cx, zone_cy)
end

# ---------------------------------------------------------------------------
# Instance struct
# ---------------------------------------------------------------------------

struct T5Instance
    case_name::String
    case_sym::Symbol
    seed_idx::Int
    seed::Int
    demand_label::String
    stations::DataFrame
    segments::DataFrame
    orders::DataFrame
    n_vehicles::Int
    vehicle_capacity::Int
    vehicle_speed::Float64
    demand_counts::NamedTuple
    suggested_l::Int
    suggested_k::Int
    hypothesis::String
    note::String
    extra::Dict{String,Any}
end

const T5_HYPOTHESIS = (
    "corridor_base: long corridor (6 km); M0 at midpoint; no detour; " *
    "A/B terminals >2957 m from all zone origins. " *
    "equilateral_triangle: SIDE=3 km; M0 on A→M; routing A→M0→B costs +137 s. " *
    "A consolidates at M0 (1500 m). " *
    "equilateral_with_m1: adds M1 at t=0.20 on segment M0→B at (-0.30, 1.039 km). " *
    "A (1587 m), apex M (1587 m), and all p_i (max 1780 m) can reach M1. " *
    "Demand configs: ab30_mb10 (corridor-dominant) vs ab20_mb20 (balanced)."
)

function _t5_note(case::Symbol)
    if case == :corridor_base
        "corridor_base: A=(-3,0) B=(3,0). M0 collinear at (0,0); A→M0→B = 750 s (no detour). " *
        "Zone origins at (0,0.8)±offsets; all within 971 m of M0. " *
        "A and B are both ~2957 m from nearest zone origin — well outside 1.8 km threshold."
    elseif case == :equilateral
        "equilateral_triangle: M0=midpoint(A,M) at (-0.75, 1.299 km). " *
        "A→M0→B = 512 s vs direct 375 s (+137 s detour). " *
        "A is 1500 m from M0 ≤ 1800 m: A passengers can consolidate at M0. " *
        "A→M (apex) = 3000 m > 1800 m: A cannot bypass M0 to reach apex directly."
    else
        "equilateral_with_m1: adds M1 at t=0.20 on segment M0→B, at (-0.30, 1.039 km). " *
        "A (1587 m), apex M (1587 m), and all zone origins (max 1780 m) can reach M1. " *
        "Optimizer chooses among M0, M1, M (apex), or combinations."
    end
end

"""
    generate_test5_instance(case, seed_idx, demand_cfg; seed_base=42) -> T5Instance

All geometry variants use the same Poisson draws (same seed, same call
order) so `n_AB`/`n_MB` are identical per seed within a demand config.
"""
function generate_test5_instance(case::Symbol, seed_idx::Int, demand_cfg::NamedTuple;
                                  seed_base::Int = T5_SEED_BASE)::T5Instance
    lambda_AB = demand_cfg.lambda_AB
    lambda_MB = demand_cfg.lambda_MB
    demand_label = demand_cfg.label

    seed = seed_base + seed_idx
    rng  = MersenneTwister(seed)
    label = t5_case_label(case)

    all_stations, vbs_stations, zone_origins, zone_center = t5_build_stations(case)

    station_df = tc_build_station_df(all_stations)
    segment_df = tc_build_segment_df(all_stations; veh_speed = T5_VEH_SPEED)

    A_st = vbs_stations[1]
    B_st = vbs_stations[end]
    order_rows = NamedTuple[]
    order_id = Ref(1)

    n_AB = tc_poisson_draw(lambda_AB * T5_WINDOW_HOURS, rng)
    for ts in tc_rand_timestamps(n_AB, rng, T5_WINDOW_START_DT, T5_WINDOW_SEC)
        tc_push_order!(order_rows, order_id, A_st.id, B_st.id, ts)
    end

    n_MB = tc_poisson_draw(lambda_MB * T5_WINDOW_HOURS, rng)
    for (i, ts) in enumerate(tc_rand_timestamps(n_MB, rng, T5_WINDOW_START_DT, T5_WINDOW_SEC))
        origin = zone_origins[mod1(i, length(zone_origins))]
        tc_push_order!(order_rows, order_id, origin.id, B_st.id, ts)
    end

    order_df = DataFrame(order_rows)
    sort!(order_df, :order_time)

    demand_counts = (A_to_B = n_AB, Mzone_to_B = n_MB, n_total = nrow(order_df))

    Ax = case == :corridor_base ? T5_A_BASE_X_KM : T5_A_X_KM
    Bx = case == :corridor_base ? T5_B_BASE_X_KM : T5_B_X_KM
    m0 = only(filter(s -> s.name == "M0", vbs_stations))
    d_A_M0 = tc_euclid_m(Ax, 0.0, m0.x_km, m0.y_km)
    route_M0_s = (tc_euclid_m(Ax, 0.0, m0.x_km, m0.y_km) + tc_euclid_m(m0.x_km, m0.y_km, Bx, 0.0)) / T5_VEH_SPEED
    direct_s = tc_euclid_m(Ax, 0.0, Bx, 0.0) / T5_VEH_SPEED

    sweep = T5_SWEEPS[case]

    extra = Dict{String,Any}(
        "zone_center_km" => [zone_center[1], zone_center[2]],
        "walk_threshold_km" => T5_WALK_THRESHOLD_KM,
        "dist_A_to_M0_m" => round(d_A_M0, digits = 1),
        "route_A_M0_B_s" => round(route_M0_s, digits = 1),
        "route_A_B_direct_s" => round(direct_s, digits = 1),
        "detour_via_M0_s" => round(route_M0_s - direct_s, digits = 1),
    )

    return T5Instance(
        label, case, seed_idx, seed, demand_label,
        station_df, segment_df, order_df,
        T5_N_VEHICLES, T5_VEHICLE_CAPACITY, T5_VEHICLE_SPEED,
        demand_counts, sweep.l, sweep.k,
        T5_HYPOTHESIS, _t5_note(case), extra,
    )
end

"""
    build_test5_cases(; n_seeds=5, cases=T5_CASES, demand_configs=T5_DEMAND_CONFIGS)
        -> Vector{T5Instance}
"""
function build_test5_cases(; n_seeds::Int = 5, cases = T5_CASES, demand_configs = T5_DEMAND_CONFIGS)::Vector{T5Instance}
    instances = T5Instance[]
    for dcfg in demand_configs
        for case in cases
            for seed_idx in 1:n_seeds
                push!(instances, generate_test5_instance(case, seed_idx, dcfg))
            end
        end
    end
    return instances
end

# ---------------------------------------------------------------------------
# StationSelectionData conversion
# ---------------------------------------------------------------------------

function create_test5_problem_data(instance::T5Instance; kwargs...)::StationSelectionData
    return tc_problem_data(instance.stations, instance.orders, instance.segments; kwargs...)
end

create_test5_station_selection_data(instance::T5Instance; kwargs...) = create_test5_problem_data(instance; kwargs...)

# ---------------------------------------------------------------------------
# Diagnostic printer
# ---------------------------------------------------------------------------

function print_test5_summary(instances::Vector{T5Instance})
    println("\nTest 5 — Equilateral Triangle vs Corridor Baseline")
    println("  Hypothesis: ", T5_HYPOTHESIS)
    for dcfg in T5_DEMAND_CONFIGS
        dc = filter(c -> c.demand_label == dcfg.label, instances)
        println("\n  [$(dcfg.label)  λ_AB=$(dcfg.lambda_AB)  λ_MB=$(dcfg.lambda_MB)]")
        println("  " * "-"^76)
        @printf("  %-24s  %4s  %8s  %5s  %4s  %6s\n",
                "case", "seed", "stations", "A→B", "M→B", "detour_M0")
        println("  " * "-"^76)
        for c in dc
            @printf("  %-24s  %4d  %8d  %5d  %4d  %+6.1f s\n",
                    c.case_name, c.seed_idx, nrow(c.stations),
                    c.demand_counts.A_to_B, c.demand_counts.Mzone_to_B, c.extra["detour_via_M0_s"])
        end
        println("  " * "-"^76)
    end
    println("  Total instances : $(length(instances))")
end
