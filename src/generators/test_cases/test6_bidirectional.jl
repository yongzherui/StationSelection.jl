"""
Port of `scripts/test_case_generation/generate_test6_bidirectional_cases.jl` —
Test 6, Bidirectional Demand.

Geometry is identical across all demand configs — the standard middle-zone
corridor (A, M0, M, B + zone origins p1-p4). Four independent demand
streams are generated per instance: A→B, zone→B, B→A, zone→A. The total
corridor rate (λ_AB=30/hr) and zone rate (λ_MB=10/hr) are split between
forward (→B) and backward (→A) directions according to the demand config.

Test 6 hypothesis: in the unidirectional case (fwd100_bwd0), M is preferred
when zone demand justifies the off-corridor detour. As backward demand
grows, M0 (equidistant for both directions) becomes more attractive because
it minimises round-trip detour costs for both A→B and B→A streams
simultaneously.

NOTE (open item): unlike every other test script, the source has NO
"Suggested sweep" line (confirmed by reading to EOF). `l=4, k=3` here is an
INFERRED default (structural analog to Test 2's base corridor: A, M0, M, B),
not a value transcribed from the source — treat it as provisional.
"""

# ---------------------------------------------------------------------------
# Geometry constants — fixed across all demand configs
# ---------------------------------------------------------------------------

const T6_VEH_SPEED         = 8.0
const T6_WALK_THRESHOLD_KM = 1.0

const T6_A_X_KM  = -3.0
const T6_B_X_KM  = 3.0
const T6_M0_X_KM = 0.0
const T6_M0_Y_KM = 0.0
const T6_M_X_KM  = 0.0
const T6_H_KM    = 0.8
const T6_M_Y_KM  = T6_H_KM

const T6_LAMBDA_AB    = 30  # total corridor rate (A↔B), orders/hr
const T6_LAMBDA_MB    = 10  # total zone rate (zone↔corridor), orders/hr
const T6_WINDOW_HOURS = 3
const T6_WINDOW_SEC   = T6_WINDOW_HOURS * 3600

const T6_SCENARIO_DATE   = Date(2026, 1, 1)
const T6_WINDOW_START_DT = DateTime(T6_SCENARIO_DATE, Time(8, 0, 0))
const T6_WINDOW_END_DT   = T6_WINDOW_START_DT + Second(T6_WINDOW_SEC - 1)

const T6_N_VEHICLES       = 6
const T6_VEHICLE_CAPACITY = 30
const T6_VEHICLE_SPEED    = 25.0

const T6_SEED_BASE = 42

# Inferred, not transcribed from the source — see module docstring.
const T6_SUGGESTED_L = 4
const T6_SUGGESTED_K = 3

const T6_DEMAND_CONFIGS = [
    (label = "fwd100_bwd0", fwd_frac = 1.00, bwd_frac = 0.00),
    (label = "fwd75_bwd25", fwd_frac = 0.75, bwd_frac = 0.25),
    (label = "fwd50_bwd50", fwd_frac = 0.50, bwd_frac = 0.50),
]

# ---------------------------------------------------------------------------
# Geometry construction
# ---------------------------------------------------------------------------

function t6_build_stations()
    vbs = [
        (id = 1, name = "A",  x_km = T6_A_X_KM,  y_km = 0.0,       role = "terminal"),
        (id = 2, name = "M0", x_km = T6_M0_X_KM, y_km = T6_M0_Y_KM, role = "on_corridor"),
        (id = 3, name = "M",  x_km = T6_M_X_KM,  y_km = T6_M_Y_KM,  role = "off_corridor"),
        (id = 4, name = "B",  x_km = T6_B_X_KM,  y_km = 0.0,       role = "terminal"),
    ]
    zone = [
        (id = 4 + i, name = off.name,
         x_km = T6_M_X_KM + off.dx, y_km = T6_M_Y_KM + off.dy, role = "zone_origin")
        for (i, off) in enumerate(TC_ZONE_OFFSETS_KM)
    ]
    all_stations = [vbs; zone]
    return all_stations, vbs, zone, vbs[1], vbs[end]
end

# ---------------------------------------------------------------------------
# Instance struct
# ---------------------------------------------------------------------------

struct T6Instance
    demand_label::String
    seed_idx::Int
    seed::Int
    fwd_frac::Float64
    bwd_frac::Float64
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
    extra::Dict{String,Any}
end

const T6_HYPOTHESIS = (
    "fwd100_bwd0: unidirectional baseline (A→B, zone→B only). " *
    "fwd75_bwd25: mild return demand — 25% of corridor and zone " *
    "passengers travel B→A / zone→A. " *
    "fwd50_bwd50: symmetric — equal forward and backward demand. " *
    "Hypothesis: as backward demand grows, on-corridor M0 becomes " *
    "more attractive because it minimises round-trip detour costs."
)

"""
    generate_test6_instance(demand_cfg, seed_idx; seed_base=42) -> T6Instance

Generates four independent demand streams in this order (load-bearing for
reproducibility): A→B, zone→B, B→A, zone→A.
"""
function generate_test6_instance(demand_cfg::NamedTuple, seed_idx::Int;
                                  seed_base::Int = T6_SEED_BASE)::T6Instance
    fwd_frac = demand_cfg.fwd_frac
    bwd_frac = demand_cfg.bwd_frac
    dlabel   = demand_cfg.label

    lam_AB_fwd = T6_LAMBDA_AB * fwd_frac
    lam_MB_fwd = T6_LAMBDA_MB * fwd_frac
    lam_AB_bwd = T6_LAMBDA_AB * bwd_frac
    lam_MB_bwd = T6_LAMBDA_MB * bwd_frac

    seed = seed_base + seed_idx
    rng  = MersenneTwister(seed)

    all_stations, vbs_stations, zone_origins, A_st, B_st = t6_build_stations()

    station_df = tc_build_station_df(all_stations)
    segment_df = tc_build_segment_df(all_stations; veh_speed = T6_VEH_SPEED)

    order_rows = NamedTuple[]
    order_id = Ref(1)

    # Stream 1: A → B
    n_AB_fwd = tc_poisson_draw(lam_AB_fwd * T6_WINDOW_HOURS, rng)
    for ts in tc_rand_timestamps(n_AB_fwd, rng, T6_WINDOW_START_DT, T6_WINDOW_SEC)
        tc_push_order!(order_rows, order_id, A_st.id, B_st.id, ts)
    end

    # Stream 2: zone → B (round-robin over p1-p4)
    n_MB_fwd = tc_poisson_draw(lam_MB_fwd * T6_WINDOW_HOURS, rng)
    for (i, ts) in enumerate(tc_rand_timestamps(n_MB_fwd, rng, T6_WINDOW_START_DT, T6_WINDOW_SEC))
        tc_push_order!(order_rows, order_id, zone_origins[mod1(i, length(zone_origins))].id, B_st.id, ts)
    end

    # Stream 3: B → A
    n_AB_bwd = tc_poisson_draw(lam_AB_bwd * T6_WINDOW_HOURS, rng)
    for ts in tc_rand_timestamps(n_AB_bwd, rng, T6_WINDOW_START_DT, T6_WINDOW_SEC)
        tc_push_order!(order_rows, order_id, B_st.id, A_st.id, ts)
    end

    # Stream 4: zone → A (round-robin over p1-p4)
    n_MB_bwd = tc_poisson_draw(lam_MB_bwd * T6_WINDOW_HOURS, rng)
    for (i, ts) in enumerate(tc_rand_timestamps(n_MB_bwd, rng, T6_WINDOW_START_DT, T6_WINDOW_SEC))
        tc_push_order!(order_rows, order_id, zone_origins[mod1(i, length(zone_origins))].id, A_st.id, ts)
    end

    order_df = DataFrame(order_rows)
    sort!(order_df, :order_time)

    demand_counts = (A_to_B = n_AB_fwd, zone_to_B = n_MB_fwd,
                      B_to_A = n_AB_bwd, zone_to_A = n_MB_bwd,
                      n_total = nrow(order_df))

    extra = Dict{String,Any}(
        "lambda_AB_fwd" => lam_AB_fwd, "lambda_MB_fwd" => lam_MB_fwd,
        "lambda_AB_bwd" => lam_AB_bwd, "lambda_MB_bwd" => lam_MB_bwd,
        "walk_threshold_km" => T6_WALK_THRESHOLD_KM,
    )

    return T6Instance(
        dlabel, seed_idx, seed, fwd_frac, bwd_frac,
        station_df, segment_df, order_df,
        T6_N_VEHICLES, T6_VEHICLE_CAPACITY, T6_VEHICLE_SPEED,
        demand_counts, T6_SUGGESTED_L, T6_SUGGESTED_K,
        T6_HYPOTHESIS, extra,
    )
end

"""
    build_test6_cases(; n_seeds=5, demand_configs=T6_DEMAND_CONFIGS) -> Vector{T6Instance}
"""
function build_test6_cases(; n_seeds::Int = 5, demand_configs = T6_DEMAND_CONFIGS)::Vector{T6Instance}
    instances = T6Instance[]
    for dcfg in demand_configs
        for seed_idx in 1:n_seeds
            push!(instances, generate_test6_instance(dcfg, seed_idx))
        end
    end
    return instances
end

# ---------------------------------------------------------------------------
# StationSelectionData conversion
# ---------------------------------------------------------------------------

function create_test6_problem_data(instance::T6Instance; kwargs...)::StationSelectionData
    return tc_problem_data(instance.stations, instance.orders, instance.segments; kwargs...)
end

create_test6_station_selection_data(instance::T6Instance; kwargs...) = create_test6_problem_data(instance; kwargs...)

# ---------------------------------------------------------------------------
# Diagnostic printer
# ---------------------------------------------------------------------------

function print_test6_summary(instances::Vector{T6Instance})
    println("\nTest 6 — Bidirectional Demand")
    println("  Hypothesis: ", T6_HYPOTHESIS)
    for dcfg in T6_DEMAND_CONFIGS
        dc = filter(c -> c.demand_label == dcfg.label, instances)
        println("\n  [$(dcfg.label)  fwd=$(Int(round(dcfg.fwd_frac*100)))% / bwd=$(Int(round(dcfg.bwd_frac*100)))%]")
        println("  " * "-"^70)
        @printf("  %-4s  %6s  %6s  %6s  %6s  %6s\n", "seed", "total", "A→B", "z→B", "B→A", "z→A")
        println("  " * "-"^70)
        for c in dc
            @printf("  %4d  %6d  %6d  %6d  %6d  %6d\n",
                    c.seed_idx, c.demand_counts.n_total,
                    c.demand_counts.A_to_B, c.demand_counts.zone_to_B,
                    c.demand_counts.B_to_A, c.demand_counts.zone_to_A)
        end
        println("  " * "-"^70)
    end
    println("  Total instances: $(length(instances))")
end
