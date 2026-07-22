"""
Port of `scripts/test_case_generation/generate_test3_north_shift_cases.jl` —
Test 3, North-Shift Zone with Intermediate Stations.

The corridor backbone (A–M0–B) and fleet configuration are held fixed. Only
the zone center moves north, and intermediate VBS candidates are
auto-inserted between M0 and M so the model has a staircase of boarding
options.

Test 3 hypothesis: at height h (0.4 km), M and M0 are both directly
reachable, no intermediate needed. At height 2h (0.8 km), one intermediate
I1 is inserted; M0 is still walkable (971 m) and the model can trade a
shorter walk (via M) against a smaller detour (via I1). At height 4h
(1.6 km), three intermediates are inserted and M0 is no longer walkable at
the default 1.0 km threshold (~1761 m) — passengers must use intermediates
or M, and a recommended 1.8 km threshold keeps M0 reachable.

Intermediate station rule (rule-based, not hardcoded):
  n_segs  = ceil(zone_h_km / max_spacing_km)
  spacing = zone_h_km / n_segs
  I_k at (0, k × spacing), k = 1 … n_segs−1

Suggested sweep (per the source script): l=5, k=3 (build 5 of 9-10 stations;
model selects from {A, M0, I1, [I2,] M, B}).
"""

# ---------------------------------------------------------------------------
# Geometry constants
# ---------------------------------------------------------------------------

const T3_A_X_KM = -3.0
const T3_B_X_KM = 3.0
const T3_H_KM   = 0.4  # base north offset for the zone center

const T3_MAX_SPACING_KM = 0.5  # max allowed vertical gap between M0 and M

# Maximum zone height at which all zone origins can still walk to M0 within
# the default 1.0 km threshold. Binding constraint: p1/p2 at
# (±0.20, h+0.15 km) → dist to M0 = sqrt(0.04+(h+0.15)²) ≤ 1.0 → h ≤ ≈0.830 km.
const T3_MAX_H_FOR_M0_WALKABLE_KM = sqrt(0.96) - 0.15  # ≈ 0.830 km

# Recommended walking threshold for variants that exceed the default (e.g.
# north_shift_4h, at zone_h=1.6 km: max dist to M0 ≈ 1761 m).
const T3_WALK_THRESHOLD_4H_KM = 1.8

const T3_WALK_THRESHOLD_KM = 1.0  # default / diagnostic threshold

const T3_LAMBDA_AB    = 30
const T3_LAMBDA_MB    = 10
const T3_WINDOW_HOURS = 3
const T3_WINDOW_SEC   = T3_WINDOW_HOURS * 3600

const T3_SCENARIO_DATE   = Date(2026, 1, 1)
const T3_WINDOW_START_DT = DateTime(T3_SCENARIO_DATE, Time(8, 0, 0))
const T3_WINDOW_END_DT   = T3_WINDOW_START_DT + Second(T3_WINDOW_SEC - 1)

const T3_N_VEHICLES       = 6
const T3_VEHICLE_CAPACITY = 30
const T3_VEHICLE_SPEED    = 25.0

const T3_SEED_BASE = 42

const T3_SUGGESTED_L = 5
const T3_SUGGESTED_K = 3

const T3_VARIANTS = [
    (case_name = "north_shift_h",  zone_h_km = T3_H_KM),
    (case_name = "north_shift_2h", zone_h_km = round(2.0 * T3_H_KM, digits = 6)),
    (case_name = "north_shift_4h", zone_h_km = round(4.0 * T3_H_KM, digits = 6)),
]

# ---------------------------------------------------------------------------
# Geometry construction
# ---------------------------------------------------------------------------

"""
    t3_build_stations(zone_h_km; max_spacing_km=T3_MAX_SPACING_KM)
        -> (all_stations, vbs_stations, zone_origins, intermediates)

Station ID assignment (corridor order, bottom-up): A, M0, I1, I2, …, M, B,
then zone demand origins p1-p4.
"""
function t3_build_stations(zone_h_km::Float64; max_spacing_km::Float64 = T3_MAX_SPACING_KM)
    n_segs  = ceil(Int, zone_h_km / max_spacing_km)
    n_int   = n_segs - 1
    spacing = zone_h_km / n_segs

    id = 1
    vbs = NamedTuple[]

    push!(vbs, (id = id, name = "A",  x_km = T3_A_X_KM, y_km = 0.0, role = "terminal"));    id += 1
    push!(vbs, (id = id, name = "M0", x_km = 0.0,        y_km = 0.0, role = "on_corridor")); id += 1

    intermediates = NamedTuple[]
    for k in 1:n_int
        y = round(k * spacing, digits = 6)
        entry = (id = id, name = "I$k", x_km = 0.0, y_km = y, role = "intermediate")
        push!(vbs, entry)
        push!(intermediates, entry)
        id += 1
    end

    push!(vbs, (id = id, name = "M", x_km = 0.0, y_km = zone_h_km, role = "off_corridor")); id += 1
    push!(vbs, (id = id, name = "B", x_km = T3_B_X_KM, y_km = 0.0, role = "terminal")); id += 1

    zone = [
        (id = id + i - 1, name = off.name,
         x_km = 0.0 + off.dx, y_km = zone_h_km + off.dy, role = "zone_origin")
        for (i, off) in enumerate(TC_ZONE_OFFSETS_KM)
    ]

    return [vbs; zone], vbs, zone, intermediates
end

# ---------------------------------------------------------------------------
# Instance struct
# ---------------------------------------------------------------------------

struct T3Instance
    case_name::String
    seed_idx::Int
    seed::Int
    zone_h_km::Float64
    stations::DataFrame
    segments::DataFrame
    orders::DataFrame
    n_vehicles::Int
    vehicle_capacity::Int
    vehicle_speed::Float64
    demand_counts::NamedTuple
    n_intermediates::Int
    recommended_walk_threshold_km::Float64
    suggested_l::Int
    suggested_k::Int
    hypothesis::String
    note::String
    extra::Dict{String,Any}
end

const T3_HYPOTHESIS = (
    "Three-way comparison of zone north-shift. north_shift_h: no intermediate; " *
    "M and M0 both reachable. north_shift_2h: one intermediate I1; M0 still " *
    "reachable (971 m); model can choose I1 (shorter detour) vs M (shorter walk). " *
    "north_shift_4h: three intermediates; M0 no longer walkable (~1761 m >> 1000 m); " *
    "passengers must use intermediates or M. Walking limit for M0 reachability: " *
    "h ≤ $(round(T3_MAX_H_FOR_M0_WALKABLE_KM, digits=3)) km."
)

function _t3_note(zone_h_km::Float64, n_int::Int, spacing::Float64, intermediates)
    if n_int == 0
        "Zone at h=$(zone_h_km) km. Gap M0→M = $(zone_h_km) km ≤ max_spacing=$(T3_MAX_SPACING_KM) km: " *
        "no intermediate inserted. M and M0 both reachable by all zone origins within 1 km."
    elseif n_int == 1
        "Zone at h=$(zone_h_km) km. One intermediate I1 at y=$(round(spacing,digits=3)) km. " *
        "Zone origins walkable to M (≤250 m) and I1 (≤585 m). " *
        "I1 sits midway; model can trade shorter walk (via M) against smaller detour (via I1)."
    else
        "Zone at h=$(zone_h_km) km. $(n_int) intermediates: " *
        join(["$(s.name) at y=$(round(s.y_km,digits=3)) km" for s in intermediates], ", ") * ". " *
        "M is further north; model expected to select lower intermediates to shorten walk."
    end
end

"""
    generate_test3_instance(case_name, zone_h_km, seed_idx; seed_base=42) -> T3Instance
"""
function generate_test3_instance(case_name::String, zone_h_km::Float64, seed_idx::Int;
                                  seed_base::Int = T3_SEED_BASE)::T3Instance
    seed = seed_base + seed_idx
    rng  = MersenneTwister(seed)

    all_stations, vbs_stations, zone_origins, intermediates = t3_build_stations(zone_h_km)

    station_df = tc_build_station_df(all_stations)
    segment_df = tc_build_segment_df(all_stations; veh_speed = 8.0)

    A = vbs_stations[1]
    B = vbs_stations[end]
    order_rows = NamedTuple[]
    order_id = Ref(1)

    n_AB = tc_poisson_draw(T3_LAMBDA_AB * T3_WINDOW_HOURS, rng)
    for ts in tc_rand_timestamps(n_AB, rng, T3_WINDOW_START_DT, T3_WINDOW_SEC)
        tc_push_order!(order_rows, order_id, A.id, B.id, ts)
    end

    n_MB = tc_poisson_draw(T3_LAMBDA_MB * T3_WINDOW_HOURS, rng)
    for (i, ts) in enumerate(tc_rand_timestamps(n_MB, rng, T3_WINDOW_START_DT, T3_WINDOW_SEC))
        origin = zone_origins[mod1(i, length(zone_origins))]
        tc_push_order!(order_rows, order_id, origin.id, B.id, ts)
    end

    order_df = DataFrame(order_rows)
    sort!(order_df, :order_time)

    demand_counts = (A_to_B = n_AB, Mzone_to_B = n_MB, n_total = nrow(order_df))

    n_int   = length(intermediates)
    n_segs  = n_int + 1
    spacing = zone_h_km / n_segs

    max_m0_walk_m = 1000 * sqrt(0.04 + (zone_h_km + 0.15)^2)
    recommended_walk_threshold_km =
        max_m0_walk_m > T3_WALK_THRESHOLD_KM * 1000 ? T3_WALK_THRESHOLD_4H_KM : T3_WALK_THRESHOLD_KM

    extra = Dict{String,Any}(
        "n_segments" => n_segs,
        "spacing_km" => round(spacing, digits = 6),
        "max_spacing_km" => T3_MAX_SPACING_KM,
        "intermediate_positions_km" => [round(k * spacing, digits = 6) for k in 1:n_int],
        "walk_threshold_km" => T3_WALK_THRESHOLD_KM,
        "max_m0_walk_m" => round(max_m0_walk_m, digits = 1),
    )

    return T3Instance(
        case_name, seed_idx, seed, zone_h_km,
        station_df, segment_df, order_df,
        T3_N_VEHICLES, T3_VEHICLE_CAPACITY, T3_VEHICLE_SPEED,
        demand_counts, n_int, recommended_walk_threshold_km,
        T3_SUGGESTED_L, T3_SUGGESTED_K,
        T3_HYPOTHESIS, _t3_note(zone_h_km, n_int, spacing, intermediates), extra,
    )
end

"""
    build_test3_cases(; n_seeds=5, variants=T3_VARIANTS) -> Vector{T3Instance}
"""
function build_test3_cases(; n_seeds::Int = 5, variants = T3_VARIANTS)::Vector{T3Instance}
    instances = T3Instance[]
    for v in variants
        for seed_idx in 1:n_seeds
            push!(instances, generate_test3_instance(v.case_name, v.zone_h_km, seed_idx))
        end
    end
    return instances
end

# ---------------------------------------------------------------------------
# StationSelectionData conversion
# ---------------------------------------------------------------------------

function create_test3_problem_data(instance::T3Instance; kwargs...)::StationSelectionData
    return tc_problem_data(instance.stations, instance.orders, instance.segments; kwargs...)
end

create_test3_station_selection_data(instance::T3Instance; kwargs...) = create_test3_problem_data(instance; kwargs...)

# ---------------------------------------------------------------------------
# Diagnostic printer
# ---------------------------------------------------------------------------

function print_test3_summary(instances::Vector{T3Instance})
    println("\nTest 3 — North-Shift Zone with Intermediate Stations")
    println("  Hypothesis: ", T3_HYPOTHESIS)
    println("  " * "-"^70)
    @printf("  %-16s  %4s  %6s  %5s  %8s  %6s  %5s  %4s\n",
            "case", "seed", "h(km)", "n_int", "stations", "orders", "A→B", "M→B")
    println("  " * "-"^70)
    for inst in instances
        @printf("  %-16s  %4d  %6.2f  %5d  %8d  %6d  %5d  %4d\n",
                inst.case_name, inst.seed_idx, inst.zone_h_km, inst.n_intermediates,
                nrow(inst.stations), nrow(inst.orders),
                inst.demand_counts.A_to_B, inst.demand_counts.Mzone_to_B)
    end
    println("  " * "-"^70)
    println("  Total instances : $(length(instances))")
end
