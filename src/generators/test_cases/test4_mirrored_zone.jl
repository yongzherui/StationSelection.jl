"""
Port of `scripts/test_case_generation/generate_test4_mirrored_zone_cases.jl` —
Test 4, Mirrored Middle-Zone Demand (Symmetry Test).

Case `single_zone` (mirrored=false): all middle demand originates from one
zone above the corridor, Z_M⁺ = (0, h).
Case `mirrored_zones` (mirrored=true): the same total middle demand is split
evenly between an upper zone Z_M⁺ and a mirrored lower zone Z_M⁻ = (0, −h).

Test 4 hypothesis: symmetric demand makes the on-corridor route A→M0→B more
attractive/stable than in the single-zone (directionally biased) case,
because M0 achieves a balanced service role neither off-corridor candidate
can match individually.

Demand consistency guarantee (load-bearing for the hypothesis — do not
change the RNG draw order): both variants draw the same total `n_MB` from
the same seed/Poisson call; only the spatial assignment of origins differs.

Suggested sweep (per the source script): l=4,k=3 (single_zone); l=5,k=3
(mirrored_zones — adds M⁻).
"""

# ---------------------------------------------------------------------------
# Geometry constants
# ---------------------------------------------------------------------------

const T4_A_X_KM = -3.0
const T4_B_X_KM = 3.0
const T4_H_KM   = 0.8  # zone height; M⁺ at (0,+h), M⁻ at (0,−h)

const T4_WALK_THRESHOLD_KM = 1.0

const T4_LAMBDA_AB    = 30
const T4_LAMBDA_MB    = 10  # total middle demand; split 50/50 in mirrored case
const T4_WINDOW_HOURS = 3
const T4_WINDOW_SEC   = T4_WINDOW_HOURS * 3600

const T4_SCENARIO_DATE   = Date(2026, 1, 1)
const T4_WINDOW_START_DT = DateTime(T4_SCENARIO_DATE, Time(8, 0, 0))
const T4_WINDOW_END_DT   = T4_WINDOW_START_DT + Second(T4_WINDOW_SEC - 1)

const T4_N_VEHICLES       = 6
const T4_VEHICLE_CAPACITY = 30
const T4_VEHICLE_SPEED    = 25.0

const T4_SEED_BASE = 42

const T4_VARIANTS = [
    (case_name = "single_zone",    mirrored = false),
    (case_name = "mirrored_zones", mirrored = true),
]

# ---------------------------------------------------------------------------
# Geometry construction
# ---------------------------------------------------------------------------

"""
    t4_build_stations(mirrored) -> (all_stations, vbs_stations, upper_origins, lower_origins)

`single_zone` (mirrored=false): A, M0, M⁺, B, p1-p4 (8 stations).
`mirrored_zones` (mirrored=true): A, M0, M⁺, M⁻, B, p1-p4, p5-p8 (13 stations).
"""
function t4_build_stations(mirrored::Bool)
    id = 1
    vbs = NamedTuple[]

    push!(vbs, (id = id, name = "A",  x_km = T4_A_X_KM, y_km = 0.0,      role = "terminal"));     id += 1
    push!(vbs, (id = id, name = "M0", x_km = 0.0,        y_km = 0.0,      role = "on_corridor"));  id += 1
    push!(vbs, (id = id, name = "M+", x_km = 0.0,        y_km = +T4_H_KM, role = "off_corridor")); id += 1
    if mirrored
        push!(vbs, (id = id, name = "M-", x_km = 0.0, y_km = -T4_H_KM, role = "off_corridor")); id += 1
    end
    push!(vbs, (id = id, name = "B", x_km = T4_B_X_KM, y_km = 0.0, role = "terminal")); id += 1

    upper_origins = [
        (id = id + i - 1, name = off.name,
         x_km = 0.0 + off.dx, y_km = +T4_H_KM + off.dy, role = "zone_origin")
        for (i, off) in enumerate(TC_ZONE_OFFSETS_KM)
    ]
    id += length(upper_origins)

    lower_origins = if mirrored
        [
            (id = id + i - 1, name = "p$(4+i)",
             x_km = 0.0 + off.dx, y_km = -T4_H_KM + off.dy, role = "zone_origin")
            for (i, off) in enumerate(TC_ZONE_OFFSETS_KM)
        ]
    else
        NamedTuple[]
    end

    all_stations = [vbs; upper_origins; lower_origins]
    return all_stations, vbs, upper_origins, lower_origins
end

# ---------------------------------------------------------------------------
# Instance struct
# ---------------------------------------------------------------------------

struct T4Instance
    case_name::String
    seed_idx::Int
    seed::Int
    mirrored::Bool
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

const T4_HYPOTHESIS = (
    "single_zone: demand biased north (Z_M⁺ only); M⁺ is the natural off-corridor VBS. " *
    "mirrored_zones: demand symmetric around corridor; M⁺ and M⁻ each serve half the " *
    "middle demand, but M0 at the corridor origin is equidistant from both zones. " *
    "Hypothesis: symmetric demand makes A→M0→B more stable than in the single-zone case."
)

"""
    generate_test4_instance(case_name, mirrored, seed_idx; seed_base=42) -> T4Instance

Demand consistency guarantee: `n_MB` is drawn once regardless of variant,
then split by round-robin (single_zone: all to upper; mirrored: odd→upper,
even→lower interleaved) — this control flow must not be refactored in a way
that changes RNG draw order.
"""
function generate_test4_instance(case_name::String, mirrored::Bool, seed_idx::Int;
                                  seed_base::Int = T4_SEED_BASE)::T4Instance
    seed = seed_base + seed_idx
    rng  = MersenneTwister(seed)

    all_stations, vbs_stations, upper_origins, lower_origins = t4_build_stations(mirrored)

    station_df = tc_build_station_df(all_stations)
    segment_df = tc_build_segment_df(all_stations; veh_speed = 8.0)

    A = vbs_stations[1]
    B = vbs_stations[end]
    order_rows = NamedTuple[]
    order_id = Ref(1)

    # Stream 1: A -> B
    n_AB = tc_poisson_draw(T4_LAMBDA_AB * T4_WINDOW_HOURS, rng)
    for ts in tc_rand_timestamps(n_AB, rng, T4_WINDOW_START_DT, T4_WINDOW_SEC)
        tc_push_order!(order_rows, order_id, A.id, B.id, ts)
    end

    # Stream 2: middle-zone -> B. n_MB is drawn identically for both variants
    # (same seed, same Poisson call order).
    n_MB = tc_poisson_draw(T4_LAMBDA_MB * T4_WINDOW_HOURS, rng)
    n_upper = 0
    n_lower = 0

    for (i, ts) in enumerate(tc_rand_timestamps(n_MB, rng, T4_WINDOW_START_DT, T4_WINDOW_SEC))
        local origin
        if !mirrored
            origin = upper_origins[mod1(i, length(upper_origins))]
            n_upper += 1
        else
            if isodd(i)
                upper_idx = mod1(div(i + 1, 2), length(upper_origins))
                origin = upper_origins[upper_idx]
                n_upper += 1
            else
                lower_idx = mod1(div(i, 2), length(lower_origins))
                origin = lower_origins[lower_idx]
                n_lower += 1
            end
        end
        tc_push_order!(order_rows, order_id, origin.id, B.id, ts)
    end

    order_df = DataFrame(order_rows)
    sort!(order_df, :order_time)

    demand_counts = (A_to_B = n_AB, Mzone_to_B = n_MB, upper_to_B = n_upper,
                      lower_to_B = n_lower, n_total = nrow(order_df))

    note = if !mirrored
        "Single-zone case: all $(n_MB) middle orders from upper zone Z_M⁺=(0,$(T4_H_KM) km). " *
        "Demand is directionally biased north of the corridor. " *
        "Off-corridor VBS M⁺ is the natural pick-up point."
    else
        "Mirrored case: $(n_MB) middle orders split $(n_upper) upper / $(n_lower) lower. " *
        "Symmetric demand around the corridor: M⁺ and M⁻ are equally useful individually, " *
        "but M0 serves both zones without a detour penalty."
    end

    suggested_l = mirrored ? 5 : 4
    suggested_k = 3

    extra = Dict{String,Any}(
        "n_zones" => mirrored ? 2 : 1,
        "zone_h_km" => T4_H_KM,
        "walk_threshold_km" => T4_WALK_THRESHOLD_KM,
    )

    return T4Instance(
        case_name, seed_idx, seed, mirrored,
        station_df, segment_df, order_df,
        T4_N_VEHICLES, T4_VEHICLE_CAPACITY, T4_VEHICLE_SPEED,
        demand_counts, suggested_l, suggested_k,
        T4_HYPOTHESIS, note, extra,
    )
end

"""
    build_test4_cases(; n_seeds=5, variants=T4_VARIANTS) -> Vector{T4Instance}
"""
function build_test4_cases(; n_seeds::Int = 5, variants = T4_VARIANTS)::Vector{T4Instance}
    instances = T4Instance[]
    for v in variants
        for seed_idx in 1:n_seeds
            push!(instances, generate_test4_instance(v.case_name, v.mirrored, seed_idx))
        end
    end
    return instances
end

# ---------------------------------------------------------------------------
# StationSelectionData conversion
# ---------------------------------------------------------------------------

function create_test4_problem_data(instance::T4Instance; kwargs...)::StationSelectionData
    return tc_problem_data(instance.stations, instance.orders, instance.segments; kwargs...)
end

create_test4_station_selection_data(instance::T4Instance; kwargs...) = create_test4_problem_data(instance; kwargs...)

# ---------------------------------------------------------------------------
# Diagnostic printer
# ---------------------------------------------------------------------------

function print_test4_summary(instances::Vector{T4Instance})
    println("\nTest 4 — Mirrored Middle-Zone Demand (Symmetry Test)")
    println("  Hypothesis: ", T4_HYPOTHESIS)
    println("  " * "-"^74)
    @printf("  %-16s  %4s  %6s  %8s  %6s  %6s  %5s  %4s\n",
            "case", "seed", "n_zones", "stations", "orders", "M→B", "↑", "↓")
    println("  " * "-"^74)
    for inst in instances
        @printf("  %-16s  %4d  %6d  %8d  %6d  %6d  %5d  %4d\n",
                inst.case_name, inst.seed_idx, inst.mirrored ? 2 : 1,
                nrow(inst.stations), nrow(inst.orders), inst.demand_counts.Mzone_to_B,
                inst.demand_counts.upper_to_B, inst.demand_counts.lower_to_B)
    end
    println("  " * "-"^74)
    println("  Total instances : $(length(instances))")
end
