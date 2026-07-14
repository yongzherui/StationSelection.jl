"""
Port of `scripts/test_case_generation/generate_test2_zone_proximity.jl` — Test 2,
Middle-Zone Proximity to B.

VBS candidate stations and the A→B demand stream are held fixed across all
variants. Only the M-zone demand origins (p1-p4) shift horizontally.

Test 2 hypothesis: when the zone is far from B (centred above M0 at x=0),
passengers can walk to the off-corridor VBS stop M, making the consolidated
A→M→B route attractive. When the zone shifts toward B (or toward A), walking
to M no longer works for some/all zone origins and the model is expected to
shift its VBS selection accordingly.

Suggested sweep (per the source script): l=4, k=3.
"""

# ---------------------------------------------------------------------------
# Geometry constants
# ---------------------------------------------------------------------------

const T2_A_X_KM = -3.0
const T2_B_X_KM = 3.0
const T2_M_Y_KM = 0.8  # north offset of off-corridor VBS candidate M

const T2_ZM_FAR_X_KM        = -1.0  # zone centred between A and midpoint
const T2_ZM_CLOSE_X_KM      = 1.0   # zone shifted 1 km toward B
const T2_ZM_NEAR_A_X_KM     = -2.5  # zone shifted toward A
const T2_ZM_WALKABLE_A_X_KM = -2.9  # zone shifted further; all p_i walkable to A

const T2_WALK_THRESHOLD_KM = 1.0

const T2_LAMBDA_AB    = 30
const T2_LAMBDA_MB    = 10
const T2_WINDOW_HOURS = 3
const T2_WINDOW_SEC   = T2_WINDOW_HOURS * 3600

const T2_SCENARIO_DATE   = Date(2026, 1, 1)
const T2_WINDOW_START_DT = DateTime(T2_SCENARIO_DATE, Time(8, 0, 0))
const T2_WINDOW_END_DT   = T2_WINDOW_START_DT + Second(T2_WINDOW_SEC - 1)

const T2_N_VEHICLES       = 6
const T2_VEHICLE_CAPACITY = 30
const T2_VEHICLE_SPEED    = 25.0

const T2_SEED_BASE = 42

const T2_SUGGESTED_L = 4
const T2_SUGGESTED_K = 3

const T2_VARIANTS = [
    (case_name = "far_from_B",            zone_cx_km = T2_ZM_FAR_X_KM),
    (case_name = "close_to_B",            zone_cx_km = T2_ZM_CLOSE_X_KM),
    (case_name = "far_from_B_close_to_A", zone_cx_km = T2_ZM_NEAR_A_X_KM),
    (case_name = "walkable_to_A",         zone_cx_km = T2_ZM_WALKABLE_A_X_KM),
]

# ---------------------------------------------------------------------------
# Geometry construction
# ---------------------------------------------------------------------------

"""
    t2_build_stations(zone_cx_km) -> (all_stations, vbs_stations, zone_origins)

The entire middle cluster (M0, M, p1-p4) shifts together horizontally with
`zone_cx_km`; only the terminals A and B stay fixed.
"""
function t2_build_stations(zone_cx_km::Float64)
    vbs = [
        (id = 1, name = "A",  x_km = T2_A_X_KM,  y_km = 0.0,      role = "terminal"),
        (id = 2, name = "M0", x_km = zone_cx_km, y_km = 0.0,      role = "on_corridor"),
        (id = 3, name = "M",  x_km = zone_cx_km, y_km = T2_M_Y_KM, role = "off_corridor"),
        (id = 4, name = "B",  x_km = T2_B_X_KM,  y_km = 0.0,      role = "terminal"),
    ]
    zone = [
        (id = 4 + i, name = off.name,
         x_km = zone_cx_km + off.dx,
         y_km = T2_M_Y_KM + off.dy,
         role = "zone_origin")
        for (i, off) in enumerate(TC_ZONE_OFFSETS_KM)
    ]
    return [vbs; zone], vbs, zone
end

"""
    t2_validate_case(zone_origins, vbs_stations; threshold_km, check_terminal_A) -> Bool

No zone origin may be within `threshold_km` of B (destination bypass always
forbidden); A-proximity is skipped for the close-to-A variant, where walking
to A is the intended test design.
"""
function t2_validate_case(zone_origins, vbs_stations;
                           threshold_km::Float64 = T2_WALK_THRESHOLD_KM,
                           check_terminal_A::Bool = true)
    A = vbs_stations[1]
    B = vbs_stations[4]
    ok = true
    for p in zone_origins
        dA = tc_euclid_km(p.x_km, p.y_km, A.x_km, A.y_km)
        dB = tc_euclid_km(p.x_km, p.y_km, B.x_km, B.y_km)
        if check_terminal_A && dA <= threshold_km
            ok = false
        end
        if dB <= threshold_km
            ok = false
        end
    end
    ok || error("Test 2 geometry violates the no-terminal-bypass constraint.")
    return true
end

# ---------------------------------------------------------------------------
# Instance struct
# ---------------------------------------------------------------------------

struct T2Instance
    case_name::String
    seed_idx::Int
    seed::Int
    zone_cx_km::Float64
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

function _t2_note(case_name::String)
    if case_name == "walkable_to_A"
        "All p_i walkable to A (max 996 m at cx=-2.9). M0 is 0.1 km from A; " *
        "A→M→B detour Δ≈75 s. Both A and M are reachable; routing expected to collapse to M0+B."
    elseif case_name == "far_from_B"
        "Far case: all zone origins can walk to M (< 1 km). M is the natural VBS choice."
    elseif case_name == "close_to_B"
        "Close case: no zone origin can walk to M (> 1 km). Routing structure expected to change."
    else
        "Close-to-A case: M cluster shifted to x=-2.5 km (near A=-3 km). " *
        "Key change: A→M→B detour Δ≈63 s (vs ≈26 s at equidistant). " *
        "Some p_i also walkable to terminal A directly. " *
        "Expected: large detour cost makes M0 preferred over M; routing collapses to M0+B."
    end
end

const T2_HYPOTHESIS = (
    "Three-way comparison of M-zone placement. " *
    "far_from_B: zone above M0; all origins walk to M (<1 km); M is likely selected. " *
    "close_to_B: zone 1 km toward B; no origin can reach M (>1 km); routing changes. " *
    "far_from_B_close_to_A: zone 2.5 km toward A; all origins walk to A (<0.8 km); " *
    "M and M0 unreachable from zone; expected routing collapse to A+B only."
)

"""
    generate_test2_instance(case_name, zone_cx_km, seed_idx; seed_base=42) -> T2Instance
"""
function generate_test2_instance(case_name::String, zone_cx_km::Float64, seed_idx::Int;
                                  seed_base::Int = T2_SEED_BASE)::T2Instance
    seed = seed_base + seed_idx
    rng  = MersenneTwister(seed)

    all_stations, vbs_stations, zone_origins = t2_build_stations(zone_cx_km)

    near_A = zone_cx_km <= T2_ZM_NEAR_A_X_KM
    t2_validate_case(zone_origins, vbs_stations; check_terminal_A = !near_A)

    station_df = tc_build_station_df(all_stations)
    segment_df = tc_build_segment_df(all_stations; veh_speed = 8.0)

    A = vbs_stations[1]
    B = vbs_stations[4]
    order_rows = NamedTuple[]
    order_id = Ref(1)

    n_AB = tc_poisson_draw(T2_LAMBDA_AB * T2_WINDOW_HOURS, rng)
    for ts in tc_rand_timestamps(n_AB, rng, T2_WINDOW_START_DT, T2_WINDOW_SEC)
        tc_push_order!(order_rows, order_id, A.id, B.id, ts)
    end

    n_MB = tc_poisson_draw(T2_LAMBDA_MB * T2_WINDOW_HOURS, rng)
    for (i, ts) in enumerate(tc_rand_timestamps(n_MB, rng, T2_WINDOW_START_DT, T2_WINDOW_SEC))
        origin = zone_origins[mod1(i, length(zone_origins))]
        tc_push_order!(order_rows, order_id, origin.id, B.id, ts)
    end

    order_df = DataFrame(order_rows)
    sort!(order_df, :order_time)

    demand_counts = (A_to_B = n_AB, Mzone_to_B = n_MB, n_total = nrow(order_df))

    extra = Dict{String,Any}(
        "zone_center_x_km" => zone_cx_km,
        "zone_center_y_km" => 0.0,
        "displacement_from_far_km" => abs(zone_cx_km - T2_ZM_FAR_X_KM),
        "walk_threshold_km" => T2_WALK_THRESHOLD_KM,
    )

    return T2Instance(
        case_name, seed_idx, seed, zone_cx_km,
        station_df, segment_df, order_df,
        T2_N_VEHICLES, T2_VEHICLE_CAPACITY, T2_VEHICLE_SPEED,
        demand_counts, T2_SUGGESTED_L, T2_SUGGESTED_K,
        T2_HYPOTHESIS, _t2_note(case_name), extra,
    )
end

"""
    build_test2_cases(; n_seeds=5, variants=T2_VARIANTS) -> Vector{T2Instance}
"""
function build_test2_cases(; n_seeds::Int = 5, variants = T2_VARIANTS)::Vector{T2Instance}
    instances = T2Instance[]
    for v in variants
        for seed_idx in 1:n_seeds
            push!(instances, generate_test2_instance(v.case_name, v.zone_cx_km, seed_idx))
        end
    end
    return instances
end

# ---------------------------------------------------------------------------
# StationSelectionData conversion
# ---------------------------------------------------------------------------

function create_test2_problem_data(instance::T2Instance; kwargs...)::StationSelectionData
    return tc_problem_data(instance.stations, instance.orders, instance.segments; kwargs...)
end

create_test2_station_selection_data(instance::T2Instance; kwargs...) = create_test2_problem_data(instance; kwargs...)

# ---------------------------------------------------------------------------
# Diagnostic printer
# ---------------------------------------------------------------------------

function print_test2_summary(instances::Vector{T2Instance})
    println("\nTest 2 — Middle-Zone Proximity to B")
    println("  Hypothesis: ", T2_HYPOTHESIS)
    println("  " * "-"^66)
    @printf("  %-22s  %4s  %6s  %8s  %6s  %5s  %4s\n",
            "case", "seed", "cx(km)", "stations", "orders", "A→B", "M→B")
    println("  " * "-"^66)
    for inst in instances
        @printf("  %-22s  %4d  %6.1f  %8d  %6d  %5d  %4d\n",
                inst.case_name, inst.seed_idx, inst.zone_cx_km,
                nrow(inst.stations), nrow(inst.orders),
                inst.demand_counts.A_to_B, inst.demand_counts.Mzone_to_B)
    end
    println("  " * "-"^66)
    println("  Total instances : $(length(instances))")
end
