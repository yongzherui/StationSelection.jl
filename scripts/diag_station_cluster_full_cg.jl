# End-to-end passenger CG A/B diagnostic: ordinary exact certification versus
# adaptive station-cluster certification with exact fallback.
using Gurobi, Printf, StationSelection
include(joinpath(@__DIR__,"generate_zhuzhou_instance.jl"))

const DATA_DIR=normpath(joinpath(@__DIR__,"..","..","Data","base_data"))
const RAW_MAX_STOPS=parse(Int,get(ENV,"PFA_DIAG_MAX_STOPS","4"))
const MAX_STOPS=RAW_MAX_STOPS <= 0 ? typemax(Int) : RAW_MAX_STOPS
const RAW_MAX_VISITS=parse(Int,get(ENV,"PFA_DIAG_MAX_VISITS","3"))
const MAX_VISITS=RAW_MAX_VISITS <= 0 ? typemax(Int) : RAW_MAX_VISITS
const TOTAL_TIME=parse(Float64,get(ENV,"PFA_DIAG_TOTAL_TIME","5400"))
const CERT_TIME=parse(Float64,get(ENV,"PFA_DIAG_CERT_TIME","900"))
const CLUSTER_TIME=parse(Float64,get(ENV,"PFA_DIAG_CLUSTER_TIME","300"))
const IP_TIME=parse(Float64,get(ENV,"PFA_DIAG_IP_TIME","900"))
const N_CANDIDATES=parse(Int,get(ENV,"PFA_DIAG_N_CANDIDATES","100"))
const MAX_NEW_COLUMNS=parse(Int,get(ENV,"PFA_DIAG_MAX_NEW_COLUMNS","20"))

length(ARGS)==4 || error("usage: ... <n> <seed> <baseline|cluster> <output.txt>")
n=parse(Int,ARGS[1]); seed=parse(Int,ARGS[2]); mode=ARGS[3]; output=ARGS[4]
mode in ("baseline","cluster") || error("mode must be baseline or cluster")

data,_=generate_zhuzhou_data(DATA_DIR,n,16;n_scenarios=3,seed=seed)
model=AggregateODRouteModel(max(2,ceil(Int,n/2));
    route_regularization_weight=10.0,walk_cost_weight=0.1,
    repositioning_time=20.0,max_walking_distance=600.0,max_wait_time=900.0,
    detour_factor=2.0,max_stops=MAX_STOPS,max_visits_per_node=MAX_VISITS)
env=Gurobi.Env()
t0=time()
result=run_passenger_free_assignment_column_generation(model,data;
    optimizer_env=env,max_cg_iters=1000,n_candidates=N_CANDIDATES,max_new_columns=MAX_NEW_COLUMNS,
    pricing_time_limit_sec=120.0,certification_time_limit_sec=CERT_TIME,
    ip_time_limit_sec=IP_TIME,total_time_limit_sec=TOTAL_TIME,
    parallel_scenarios=true,station_simple_warm_start=false,
    use_adaptive_cluster_certification=(mode=="cluster"),
    cluster_initial_num_clusters=max(2,ceil(Int,n/3)),
    cluster_max_num_clusters=n,cluster_time_limit_sec=CLUSTER_TIME,
    verify_reduced_costs=true,verbose=true)
wall=time()-t0
cluster_rows=get(result.final_result.metadata,"cluster_certificate_rows",NamedTuple[])
n_attempts=length(cluster_rows)
n_scenario_cert=count(r->r.certified,cluster_rows)
cert_rounds=Set((r.round,r.iteration) for r in cluster_rows if r.certified)
all_rounds=Set((r.round,r.iteration) for r in cluster_rows)
fully_certified_rounds=count(key->all(r->r.certified,
    filter(r->(r.round,r.iteration)==key,cluster_rows)),all_rounds)
cluster_seconds=sum((r.seconds for r in cluster_rows);init=0.0)
max_clusters=maximum((r.num_clusters_after for r in cluster_rows);init=0)

open(output,"w") do io
    @printf(io,"SUMMARY n=%d seed=%d scenarios=3 max_stops=%s max_visits=%s n_candidates=%d max_new_columns=%d mode=%s status=%s stop=%s lp_certified=%s cg_iters=%d rounds=%d columns=%d labels=%d pricing_seconds=%.6f certification_seconds=%.6f total_seconds=%.6f wall=%.6f\n",
        n,seed,MAX_STOPS==typemax(Int) ? "uncapped" : string(MAX_STOPS),
        MAX_VISITS==typemax(Int) ? "uncapped" : string(MAX_VISITS),N_CANDIDATES,MAX_NEW_COLUMNS,mode,string(result.status),string(result.cg_stop_reason),
        string(result.lp_bound_certified),result.n_cg_iters,result.n_rounds,
        result.n_columns,result.total_labels_generated,result.total_pricing_seconds,
        result.certification_seconds,result.total_seconds,wall)
    @printf(io,"CLUSTER attempts=%d scenario_certificates=%d certification_rounds=%d fully_certified_rounds=%d cluster_seconds=%.6f max_clusters=%d\n",
        n_attempts,n_scenario_cert,length(cert_rounds),fully_certified_rounds,
        cluster_seconds,max_clusters)
    for r in cluster_rows
        @printf(io,"CLUSTER_ROW round=%d iteration=%d scenario=%d K_before=%d K_after=%d lb=%.9f certified=%s stop=%s labels=%d seconds=%.6f route=%s\n",
            r.round,r.iteration,r.scenario,r.num_clusters_before,r.num_clusters_after,
            r.lower_bound_reduced_cost,string(r.certified),r.stop_reason,
            r.labels_generated,r.seconds,r.cluster_route)
    end
end
println(read(output,String))
