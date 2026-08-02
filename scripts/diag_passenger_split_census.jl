# Census for the "last contributing pickup" split (bidirectional / meet-in-the-middle
# PFA pricing). Answers the three gating questions, on a seeded-RMP dual snapshot:
#
#   Q1  How much of a route lies AFTER its last contributing pickup, versus after the
#       pickup window W? The delivery suffix is what a backward DP would absorb; if it
#       is a small share of the route there is nothing to amortize. Also directly tests
#       the claim that the last-pickup split strictly dominates the W split.
#
#   Q2  Among live labels, how many share a (current station, live-clock support) pair?
#       That pair is the proposed backward-DP shard key. Mean labels per shard is the
#       make-or-break number: ~1 means the DP degenerates to one build per label (i.e.
#       today's per-label post_w_completion) and the design dies.
#
#   Q3  What fraction of labels ever exceed W? A control. Near zero confirms the W
#       split would never fire and that the last-pickup cut is the only viable one.
#
# Design note: notes/2026-08-01_pfa_last_pickup_split_design.md
using CSV, DataFrames, Gurobi, JuMP, Printf, Statistics, StationSelection
include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = parse(Int, get(ENV, "PFASP_N_PAIRS", "16"))
const RAW_MAX_STOPS = parse(Int, get(ENV, "PFASP_MAX_STOPS", "7"))
const MAX_STOPS = RAW_MAX_STOPS <= 0 ? typemax(Int) : RAW_MAX_STOPS
const SEARCH_TIME = parse(Float64, get(ENV, "PFASP_SEARCH_TIME", "300"))
const MAX_WAIT = parse(Float64, get(ENV, "PFASP_MAX_WAIT", "900"))
const GRB_ENV = Gurobi.Env()

function pricing_snapshot(n::Int)
    data, _ = generate_zhuzhou_data(DATA_DIR, n, N_PAIRS; n_scenarios=1, seed=42)
    model = AggregateODRouteModel(max(2, ceil(Int, n / 2));
        route_regularization_weight=10.0, walk_cost_weight=0.1,
        repositioning_time=20.0, max_walking_distance=600.0,
        max_wait_time=MAX_WAIT, detour_factor=2.0,
        max_stops=MAX_STOPS, max_visits_per_node=3)
    mapping = create_map(model, data)
    md = StationSelection.create_passenger_free_assignment_master_data(model, data, mapping)
    master = build_passenger_free_assignment_master(md, GRB_ENV; relax_integrality=true)
    set_silent(master.model)
    next_id = 1
    for column in passenger_free_assignment_two_stop_seed_columns(md; next_column_id=next_id)
        StationSelection.add_passenger_free_assignment_column!(master, column)
        next_id += 1
    end
    optimize!(master.model)
    termination_status(master.model) == JuMP.MOI.OPTIMAL || error("seed master did not solve")
    alpha, gamma_o, gamma_d = extract_passenger_free_assignment_duals(master)
    candidates = passenger_free_assignment_pricing_candidates(md, alpha, gamma_o, gamma_d, 1)
    pd = create_passenger_free_assignment_pricing_data(1, md.nodes, md.travel_cost, candidates;
        route_regularization_weight=md.route_regularization_weight,
        max_wait_time=md.max_wait_time, repositioning_time=md.repositioning_time,
        max_stops=md.max_stops, max_visits_per_node=md.max_visits_per_node)
    return pd
end

_ride_limit(pd, p, o, d) = begin
    lim = -Inf
    for opp in get(pd.assignments_by_origin, o, StationSelection.PassengerAssignmentOpportunity[])
        opp.passenger == p && opp.destination == d && (lim = max(lim, opp.ride_limit))
    end
    lim
end

"""
Split indices for one finished route, as *route positions* (1-based).

`q_pickup` is the position of the last stop whose opened pickup clock still
certifies an assignment the replayed route actually banks -- the split point of
the last-pickup decomposition. Everything after it is pure delivery, because by
construction no clock opened later contributes.

`q_w` is the last position reached within the pickup window `W` -- the split point
of the (weaker) post-W decomposition. `q_pickup <= q_w` always: no clock opens
after W, so the last *contributing* pickup cannot lie past it.
"""
function split_indices(route::Vector{Int}, pd)
    m = length(route)
    W = pd.max_wait_time
    elapsed = zeros(Float64, m)
    for i in 2:m
        elapsed[i] = elapsed[i - 1] +
            StationSelection._passenger_free_assignment_travel(pd, route[i - 1], route[i])
    end

    # Last position < d at which station s had a clock opened (a visit within W).
    function clock_open_before(s::Int, d::Int)
        best = 0
        for i in 1:(d - 1)
            route[i] == s && elapsed[i] <= W + 1e-9 && (best = i)
        end
        return best
    end

    banked = StationSelection._replay_passenger_free_assignment_route(route, pd)
    q_pickup = 1
    for (p, (o, dstation, _r)) in banked
        limit = _ride_limit(pd, p, o, dstation)
        isfinite(limit) || continue
        for d in 2:m
            route[d] == dstation || continue
            ci = clock_open_before(o, d)
            ci == 0 && continue
            if elapsed[d] - elapsed[ci] <= limit + 1e-9
                q_pickup = max(q_pickup, ci)
                break   # earliest certifying visit -> earliest admissible split
            end
        end
    end

    q_w = 1
    for i in 1:m
        elapsed[i] <= W + 1e-9 && (q_w = i)
    end
    return (m=m, q_pickup=q_pickup, q_w=q_w, duration=elapsed[m],
            n_banked=length(banked))
end

function main()
    length(ARGS) == 2 || error("usage: ... <n_stations> <output.csv>")
    n = parse(Int, ARGS[1]); output = ARGS[2]
    pd = pricing_snapshot(n)
    W = pd.max_wait_time
    @printf("CONFIG n=%d n_pairs=%d max_stops=%s W=%.1f opportunities=%d layers=%d\n",
        n, N_PAIRS, MAX_STOPS == typemax(Int) ? "inf" : string(MAX_STOPS),
        W, length(pd.opportunities), pd.n_layers)
    flush(stdout)

    # ── Q2 / Q3: census the live-label population ─────────────────────────────
    # Vector keys are hashed and compared by content; each observation builds a fresh
    # vector and never mutates one already stored, so this is safe as a Dict key.
    shard_counts = Dict{Tuple{Int, Vector{Int}}, Int}()
    support_hist = Dict{Int, Int}()
    n_labels = 0
    n_past_w = 0
    n_empty_support = 0

    observer = function (label)
        n_labels += 1
        label.time > W + 1e-9 && (n_past_w += 1)
        support = sort!(collect(keys(label.station_age)))
        isempty(support) && (n_empty_support += 1)
        key = (label.current, support)
        shard_counts[key] = get(shard_counts, key, 0) + 1
        support_hist[length(support)] = get(support_hist, length(support), 0) + 1
        return nothing
    end

    t0 = time()
    labels, exhausted, stats = StationSelection._enumerate_passenger_free_assignment_pricing_labels(
        pd; time_limit=SEARCH_TIME, reduced_cost_tol=1e-6,
        max_visits_per_node=pd.max_visits_per_node, label_observer=observer,
    )
    search_seconds = time() - t0
    @printf("SEARCH exhausted=%s seconds=%.1f generated=%d inserted=%d candidates=%d\n",
        string(exhausted), search_seconds, stats.labels_generated, n_labels, length(labels))
    flush(stdout)

    sizes = sort!(collect(values(shard_counts)); rev=true)
    n_shards = length(sizes)
    # Contrast: today's dominance buckets are keyed on `current` alone.
    current_only = length(unique(first(k) for k in keys(shard_counts)))

    # ── Q1: split geometry on the harvested candidate routes ──────────────────
    route_rows = NamedTuple[]
    for label in labels
        s = split_indices(label.route, pd)
        s.m >= 2 || continue
        push!(route_rows, (; n_stations=n, route_length=s.m, duration=s.duration,
            n_banked=s.n_banked, reduced_cost=label.reduced_cost,
            improving=label.reduced_cost < -1e-6,
            q_pickup=s.q_pickup, q_w=s.q_w,
            suffix_pickup=s.m - s.q_pickup, suffix_w=s.m - s.q_w,
            suffix_pickup_frac=(s.m - s.q_pickup) / s.m,
            suffix_w_frac=(s.m - s.q_w) / s.m,
            route=join(label.route, ';')))
    end
    routes = DataFrame(route_rows)
    CSV.write(replace(output, ".csv" => ".routes.csv"), routes)

    hist = DataFrame(sort!([(; support_size=k, n_labels=v) for (k, v) in support_hist];
                           by=r -> r.support_size))
    CSV.write(replace(output, ".csv" => ".support_hist.csv"), hist)

    improving = isempty(routes) ? routes : filter(r -> r.improving, routes)
    mean_or = (v, f) -> isempty(v) ? NaN : f(v)

    summary = (; n_stations=n, n_pairs=N_PAIRS,
        max_stops=(MAX_STOPS == typemax(Int) ? -1 : MAX_STOPS),
        max_wait_time=W, n_opportunities=length(pd.opportunities), n_layers=pd.n_layers,
        search_exhausted=exhausted, search_seconds=search_seconds,
        labels_generated=stats.labels_generated, labels_inserted=n_labels,
        max_live_labels=stats.max_live_labels,
        # Q3
        frac_labels_past_w=(n_labels == 0 ? NaN : n_past_w / n_labels),
        frac_labels_empty_support=(n_labels == 0 ? NaN : n_empty_support / n_labels),
        # Q2 -- the make-or-break number is mean/median labels per shard
        n_shards=n_shards, n_shards_current_only=current_only,
        mean_labels_per_shard=(n_shards == 0 ? NaN : n_labels / n_shards),
        median_labels_per_shard=mean_or(sizes, median),
        p90_labels_per_shard=(isempty(sizes) ? NaN : quantile(sizes, 0.9)),
        max_labels_per_shard=(isempty(sizes) ? 0 : first(sizes)),
        frac_singleton_shards=(n_shards == 0 ? NaN : count(==(1), sizes) / n_shards),
        mean_support_size=mean_or([k for (k, v) in support_hist for _ in 1:v], mean),
        # Q1
        n_routes=nrow(routes), n_routes_improving=nrow(improving),
        mean_route_length=mean_or(routes.route_length, mean),
        mean_suffix_pickup_frac=mean_or(routes.suffix_pickup_frac, mean),
        mean_suffix_w_frac=mean_or(routes.suffix_w_frac, mean),
        median_suffix_pickup=mean_or(routes.suffix_pickup, median),
        median_suffix_w=mean_or(routes.suffix_w, median),
        mean_suffix_pickup_frac_improving=mean_or(
            isempty(improving) ? Float64[] : improving.suffix_pickup_frac, mean),
        mean_suffix_w_frac_improving=mean_or(
            isempty(improving) ? Float64[] : improving.suffix_w_frac, mean),
        frac_routes_w_never_fires=(nrow(routes) == 0 ? NaN :
            count(==(0), routes.suffix_w) / nrow(routes)),
        mean_duration=mean_or(routes.duration, mean))
    CSV.write(output, DataFrame([summary]))

    @printf("Q1 suffix_after_last_pickup=%.1f%% of stops (post-W: %.1f%%), W never fires on %.1f%% of routes\n",
        100 * summary.mean_suffix_pickup_frac, 100 * summary.mean_suffix_w_frac,
        100 * summary.frac_routes_w_never_fires)
    @printf("Q2 shards=%d mean=%.2f median=%.1f p90=%.1f max=%d singletons=%.1f%% (current-only buckets: %d)\n",
        n_shards, summary.mean_labels_per_shard, summary.median_labels_per_shard,
        summary.p90_labels_per_shard, summary.max_labels_per_shard,
        100 * summary.frac_singleton_shards, current_only)
    @printf("Q3 labels past W = %.2f%%  mean live-clock support = %.2f\n",
        100 * summary.frac_labels_past_w, summary.mean_support_size)
    @printf("DONE n=%d labels=%d routes=%d %.1fs\n", n, n_labels, nrow(routes), search_seconds)
end
main()
