using Printf
using StationSelection

function fixture(n::Int)
    nodes = collect(1:n)
    travel = Dict((i, j) => Float64(abs(i - j) + ((3i + 5j) % 4) / 10)
                  for i in nodes for j in nodes if i != j)
    pairs = [(i, j) for i in nodes for j in nodes if i != j && (i + 2j) % 3 != 0]
    data = AggregateODRoutePricingData(
        1, nodes, travel, pairs, 0.15, 0.0, 5.0, 1.8, 6, 2, true,
    )
    duals = AggregateODRoutePricingDuals(
        Dict(pair => 3.0 + ((7pair[1] + pair[2]) % 11) / 3 for pair in pairs),
    )
    return data, duals
end

function main()
    length(ARGS) == 2 || error("usage: bench_nearest_open_pricer_mechanics.jl N signatures.txt")
    n = parse(Int, ARGS[1])
    signature_path = ARGS[2]
    data, duals = fixture(n)

    function run_once()
        columns, exhausted, stats = aggregate_od_route_pricing_by_label_setting(
            data, AggregateODRouteColumn[], duals;
            next_column_id=1, max_new_columns=100_000, n_candidates=100_000,
            time_limit=120.0, max_visits_per_node=2, profile=true,
        )
        signatures = sort([
            (Tuple(sort(c.od_pairs)), c.tau, c.metadata["reduced_cost"], c.metadata["route"])
            for c in columns
        ])
        return signatures, exhausted, stats
    end

    run_once() # compile every exercised method before measuring
    GC.gc()
    local result
    alloc = @allocated begin
        elapsed = @elapsed result = run_once()
    end
    signatures, exhausted, stats = result
    open(signature_path, "w") do io
        for signature in signatures
            println(io, repr(signature))
        end
    end
    @printf("RESULT wall=%.6f alloc=%d exhausted=%s columns=%d labels=%d rejected=%d removed=%d stale=%d max_live=%d dominance=%.6f\n",
        elapsed, alloc, exhausted, length(signatures), stats.labels_generated,
        stats.labels_rejected_by_dominance, stats.labels_removed_by_dominance,
        stats.stale_pops, stats.max_live_labels, stats.t_dominance_sec)
end

main()
