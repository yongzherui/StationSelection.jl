"""
    scripts/aggregate_od_route_method_grid.jl

Shared instance-grid and method-list definitions for the full method comparison
experiment on `AggregateODRouteModel` under
`NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)`:

  - Direct solve (enumerate-then-MIP), max_stops in {3, 4}
  - Plain column generation (no Benders), max_stops in {4, uncapped}
  - Benders decomposition on BendersY / BendersYZ / BendersYZH, each with
    {standard+no-reprice, standard+reprice, zero_completion, mw_cut (where
    applicable)} x max_stops in {4, uncapped}

Included by both `generate_method_compare_job_list.jl` (writes the job list)
and `run_method_compare_task.jl` (runs one (instance, method) task), so the
method-label -> solver-config mapping never drifts out of sync between the
two.

Instance families:
  - "grid"    synthetic Manhattan grid (generate_grid_instance / create_grid_problem_data)
  - "zhuzhou" real Zhuzhou station/order data (generate_zhuzhou_data, script-local
              generator included below -- supports n_scenarios, unlike the library's
              single-scenario generate_zhuzhou_instance)
"""

using StationSelection

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

# ── instance grid axes ──────────────────────────────────────────────────────

const N_STATIONS_LIST = [10, 15, 20, 30, 40, 50, 60]
const N_PAIRS_LIST     = [8, 16, 32]
const SEEDS             = [42, 123, 999]
const FAMILIES          = ["grid", "zhuzhou"]
const ENDPOINT_OVERLAP  = 2.0
const ZZ_N_SCENARIOS    = 3

_l_for(n::Int) = ceil(Int, n / 2)

# nx * ny == n_stations for every value in N_STATIONS_LIST, chosen close to square.
const _GRID_DIMS = Dict(
    10 => (2, 5), 15 => (3, 5), 20 => (4, 5), 30 => (5, 6),
    40 => (5, 8), 50 => (5, 10), 60 => (6, 10),
)
_grid_dims_for(n::Int) = get(() -> error(
    "no grid factorization registered for n_stations=$n; add one to _GRID_DIMS"
), _GRID_DIMS, n)

# ── method grid ──────────────────────────────────────────────────────────────

const MAX_STOPS_MODES = [:ms4, :uncapped]

struct MethodSpec
    label::String
    kind::Symbol                  # :direct | :cg | :benders
    decomposition::Any            # StationSelection.BendersY()/BendersYZ()/BendersYZH(), or nothing
    cut_derivation::Symbol        # :standard | :zero_completion | :restricted_mw_fixed_pi (benders only)
    reprice::Bool                 # benders only
    max_stops_mode::Symbol        # :ms4 | :ms3 | :uncapped
end

function _benders_variants(label_prefix::String, decomposition; include_mw::Bool)
    variants = [
        ("std_noreprice", :standard, false),
        ("std_reprice",   :standard, true),
        ("zerocomp",      :zero_completion, false),
    ]
    include_mw && push!(variants, ("mw", :restricted_mw_fixed_pi, false))

    specs = MethodSpec[]
    for (suffix, cut_derivation, reprice) in variants, ms_mode in MAX_STOPS_MODES
        push!(specs, MethodSpec(
            "$(label_prefix)_$(suffix)_$(ms_mode)", :benders, decomposition,
            cut_derivation, reprice, ms_mode,
        ))
    end
    return specs
end

const METHODS = MethodSpec[
    MethodSpec("direct_ms4",   :direct, nothing, :standard, false, :ms4),
    # direct_ms3 (max_stops=3): a cheaper fallback enumeration point kept alongside
    # direct_ms4 so Direct still has a comparison point at instance sizes where
    # ms4 enumeration blows past its time/memory budget (see CS_DIRECT_MAX_ROUTES /
    # CS_DIRECT_TIME_LIMIT in run_method_compare_task.jl).
    MethodSpec("direct_ms3",   :direct, nothing, :standard, false, :ms3),
    MethodSpec("cg_ms4",       :cg,     nothing, :standard, false, :ms4),
    # cg_uncapped (plain CG with max_stops=typemax(Int)) dropped: with y unfixed
    # (unlike Benders' inner CG, which solves a y-fixed RouteCoveringProblem --
    # see build_solver/covering.jl), the joint labeling search over all
    # n_stations candidates didn't converge even after raising the pricing
    # time limit and unrestricting max_visits_per_node -- ran 14+ min on a real
    # compute node at n=10 with no sign of finishing. Benders keeps both ms4
    # and uncapped variants since it isn't affected by this.
    _benders_variants("bendersY",   StationSelection.BendersY();   include_mw=true)...,
    _benders_variants("bendersYZ",  StationSelection.BendersYZ();  include_mw=true)...,
    _benders_variants("bendersYZH", StationSelection.BendersYZH(); include_mw=false)...,
]

method_by_label(label::AbstractString) = only(filter(m -> m.label == label, METHODS))

# ── instance construction ────────────────────────────────────────────────────

"""
    resolve_max_stops(mode, n_stations) -> Int

`:uncapped` must resolve to `typemax(Int)` -- the model's own "no limit"
sentinel (what `AggregateODRouteModel(...; max_stops=nothing)` resolves to
internally) -- NOT to a finite value equal to `n_stations`. Passing a finite
max_stops, even one equal to n_stations, flips `bounded_max_stops=true` in the
pricing labeling algorithm (pricing/data.jl), which tracks remaining-stops
budget in every label and is dramatically more expensive than the genuinely
uncapped path; on a 10-station grid instance this hung column generation past
100s of wall time where the true uncapped path converges in seconds.
"""
function resolve_max_stops(mode::Symbol, n_stations::Int)::Int
    mode == :ms4 && return 4
    mode == :ms3 && return 3
    mode == :uncapped && return typemax(Int)
    error("unknown max_stops_mode=$mode")
end

"""
    build_instance(family, n_stations, n_pairs, seed, data_dir) -> (data, max_walking_distance)

`max_walking_distance` is returned alongside `data` because the two families
resolve it differently: the synthetic grid measures distance in grid units
(so the cap is set to the grid diagonal -- effectively unrestricted), while
Zhuzhou measures walking time in seconds (matches the 600s convention used in
compare_benders_decompositions.jl / run_zhuzhou_instance.jl).

Walking cost is left at its raw, unscaled value here -- the route-weight vs
walk-weight tradeoff (`route_regularization_weight` vs `walk_cost_weight`) is
applied at the MODEL level (AggregateODRouteModel), not baked into the input
data, so it consistently affects every objective/dual/cut computation that
reads walking cost (see walk_cost_weight's docstring).
"""
function build_instance(family::AbstractString, n_stations::Int, n_pairs::Int, seed::Int, data_dir::AbstractString)
    if family == "grid"
        nx, ny = _grid_dims_for(n_stations)
        instance = generate_grid_instance(nx, ny, n_pairs; endpoint_overlap=ENDPOINT_OVERLAP, seed=seed)
        max_walk = Float64(nx + ny)
        data = create_grid_problem_data(instance; max_walking_distance=max_walk)
        return data, max_walk
    elseif family == "zhuzhou"
        data, meta = generate_zhuzhou_data(
            data_dir, n_stations, n_pairs;
            n_scenarios=ZZ_N_SCENARIOS, endpoint_overlap=ENDPOINT_OVERLAP, seed=seed,
        )
        print_zhuzhou_data_summary(data, meta)
        return data, 600.0
    else
        error("unknown family=$family (expected \"grid\" or \"zhuzhou\")")
    end
end
