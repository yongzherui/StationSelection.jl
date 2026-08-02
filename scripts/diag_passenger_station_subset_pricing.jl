# Benchmark certified station-subset pricing on a seeded-RMP dual snapshot.
using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection
include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = parse(Int, get(ENV, "PFASS_N_PAIRS", "16"))
const TOTAL_TIME = parse(Float64, get(ENV, "PFASS_TOTAL_TIME", "3000"))
const ORACLE_TIME = parse(Float64, get(ENV, "PFASS_ORACLE_TIME", "300"))
const EARLY_TIME = parse(Float64, get(ENV, "PFASS_EARLY_TIME", "300"))
const INTEGRAL_REWARD = get(ENV, "PFASS_INTEGRAL_REWARD", "0") in ("1","true","yes")
const ROUTING_BOUND = get(ENV, "PFASS_ROUTING_BOUND", "1") in ("1","true","yes")
const USE_TRIPLE = get(ENV, "PFASS_TRIPLE", "0") in ("1","true","yes")
const TRIPLE_ALTS = parse(Int, get(ENV, "PFASS_TRIPLE_ALTS", "3"))
const PHASE = get(ENV, "PFASS_PHASE", "both")   # both | direct | bnb
const EXACT_PRUNE = get(ENV, "PFASS_EXACT_PRUNE", "1") in ("1","true","yes")
const EXACT_POSTW = get(ENV, "PFASS_EXACT_POSTW", "0") in ("1","true","yes")
const RAW_NODE_LIMIT = parse(Int, get(ENV, "PFASS_NODE_LIMIT", "0"))
const NODE_LIMIT = RAW_NODE_LIMIT <= 0 ? typemax(Int) : RAW_NODE_LIMIT
const RAW_MAX_STOPS = parse(Int,get(ENV,"PFASS_MAX_STOPS","4"))
const MAX_STOPS = RAW_MAX_STOPS <= 0 ? typemax(Int) : RAW_MAX_STOPS
const GRB_ENV = Gurobi.Env()
const OPTIMIZER = () -> Gurobi.Optimizer(GRB_ENV)

function pricing_snapshot(n::Int)
    data, _ = generate_zhuzhou_data(DATA_DIR, n, N_PAIRS; n_scenarios=1, seed=42)
    model = AggregateODRouteModel(max(2, ceil(Int,n/2));
        route_regularization_weight=10.0, walk_cost_weight=0.1,
        repositioning_time=20.0, max_walking_distance=600.0,
        max_wait_time=900.0, detour_factor=2.0,
        max_stops=MAX_STOPS, max_visits_per_node=3)
    mapping = create_map(model, data)
    md = StationSelection.create_passenger_free_assignment_master_data(model, data, mapping)
    master = build_passenger_free_assignment_master(md, GRB_ENV; relax_integrality=true)
    set_silent(master.model)
    next_id = 1
    for column in passenger_free_assignment_two_stop_seed_columns(md; next_column_id=next_id)
        StationSelection.add_passenger_free_assignment_column!(master,column)
        next_id += 1
    end
    optimize!(master.model)
    termination_status(master.model) == JuMP.MOI.OPTIMAL || error("seed master did not solve")
    alpha,gamma_o,gamma_d = extract_passenger_free_assignment_duals(master)
    candidates = passenger_free_assignment_pricing_candidates(md,alpha,gamma_o,gamma_d,1)
    pd = create_passenger_free_assignment_pricing_data(1,md.nodes,md.travel_cost,candidates;
        route_regularization_weight=md.route_regularization_weight,
        max_wait_time=md.max_wait_time, repositioning_time=md.repositioning_time,
        max_stops=md.max_stops, max_visits_per_node=md.max_visits_per_node)
    return pd, max(2,ceil(Int,n/2))
end

function row(n, mode, c)
    (; n, L=max(2,ceil(Int,n/2)), mode, optimal_value=c.optimal_value,
       reduced_cost=c.best_exact_result.reduced_cost,
       improving=c.optimal_value > 1e-6, globally_certified=c.globally_certified,
       global_upper_bound=c.final_global_upper_bound, absolute_gap=c.absolute_gap,
       nodes_created=c.nodes_created, nodes_processed=c.nodes_processed,
       pruned_cheap=c.nodes_pruned_cheap, pruned_lp=c.nodes_pruned_lp,
       fixed_priced=c.fixed_subsets_priced, heuristic_priced=c.heuristic_subsets_priced,
       unique_priced=c.unique_subsets_priced, lp_solves=c.reward_lp_solves,
       exact_seconds=c.total_exact_pricing_time, bound_seconds=c.total_bound_time,
       total_seconds=c.total_runtime_sec, labels=c.best_exact_result.labels_generated,
       station_set=join(collect(c.best_exact_result.station_set),';'),
       route=join(c.best_exact_result.route,';'))
end

function baseline_row(n, r)
    (; n, L=max(2,ceil(Int,n/2)), mode="full_network_exact",
       optimal_value=r.value, reduced_cost=r.reduced_cost, improving=r.value > 1e-6,
       globally_certified=r.certified, global_upper_bound=r.certified ? r.value : Inf,
       absolute_gap=r.certified ? 0.0 : Inf, nodes_created=0, nodes_processed=0,
       pruned_cheap=0, pruned_lp=0, fixed_priced=1, heuristic_priced=0,
       unique_priced=1, lp_solves=0, exact_seconds=r.runtime_sec, bound_seconds=0.0,
       total_seconds=r.runtime_sec, labels=r.labels_generated,
       station_set=join(collect(r.station_set),';'), route=join(r.route,';'))
end

function main()
    length(ARGS) == 2 || error("usage: ... <n> <output.csv>")
    n=parse(Int,ARGS[1]); output=ARGS[2]
    pd,L=pricing_snapshot(n)
    rows=NamedTuple[]
    variant = (INTEGRAL_REWARD ? "integral" : "lp") *
              (ROUTING_BOUND ? "_routing" : "_rewardonly") *
              (USE_TRIPLE ? "_triple" : "")
    PHASE in ("both","direct","bnb") || error("PFASS_PHASE must be both|direct|bnb")
    if PHASE in ("both","bnb")
        early=price_by_station_subset_branch_and_bound(pd,L; optimizer=OPTIMIZER,
            settings=StationSubsetPricingSettings(stop_on_first_improving_column=true,
                integral_reward_stations=INTEGRAL_REWARD,
                use_routing_reward_bound=ROUTING_BOUND,
                use_triple_routing_bounds=USE_TRIPLE, triple_alternatives_per_passenger=TRIPLE_ALTS,
                node_limit=NODE_LIMIT,
                time_limit=EARLY_TIME,exact_oracle_time_limit=min(ORACLE_TIME,EARLY_TIME),verbose=true))
        push!(rows,row(n,"early_column_$(variant)",early)); CSV.write(output,DataFrame(rows)); flush(stdout)
        @printf("EARLY n=%d L=%d profit=%.6f certified=%s ub=%.6f time=%.1f\n",
            n,L,early.optimal_value,string(early.globally_certified),early.final_global_upper_bound,early.total_runtime_sec)
    end
    if PHASE in ("both","direct")
        baseline=price_exact_on_stations(pd,BitSet(pd.nodes);time_limit=ORACLE_TIME,
            use_reduced_cost_pruning=EXACT_PRUNE, use_post_w_completion_bound=EXACT_POSTW)
        push!(rows,baseline_row(n,baseline)); CSV.write(output,DataFrame(rows)); flush(stdout)
        @printf("BASELINE n=%d profit=%.6f certified=%s labels=%d time=%.1f\n",
            n,baseline.value,string(baseline.certified),baseline.labels_generated,baseline.runtime_sec)
    end
    if PHASE in ("both","bnb")
        full=price_by_station_subset_branch_and_bound(pd,L; optimizer=OPTIMIZER,
            settings=StationSubsetPricingSettings(integral_reward_stations=INTEGRAL_REWARD,
                use_routing_reward_bound=ROUTING_BOUND,
                use_triple_routing_bounds=USE_TRIPLE, triple_alternatives_per_passenger=TRIPLE_ALTS,
                node_limit=NODE_LIMIT,
                time_limit=TOTAL_TIME,
                exact_oracle_time_limit=ORACLE_TIME,verbose=true))
        push!(rows,row(n,"certification_$(variant)",full)); CSV.write(output,DataFrame(rows)); flush(stdout)
        @printf("CERT n=%d L=%d profit=%.6f certified=%s ub=%.6f gap=%.6f nodes=%d subsets=%d time=%.1f\n",
            n,L,full.optimal_value,string(full.globally_certified),full.final_global_upper_bound,
            full.absolute_gap,full.nodes_processed,full.unique_subsets_priced,full.total_runtime_sec)
    end
end
main()
