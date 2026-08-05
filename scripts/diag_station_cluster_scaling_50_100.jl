# Cluster-certified full-CG scaling with recoverable early-termination telemetry.
using Dates, Gurobi, JuMP, Printf, StationSelection
include(joinpath(@__DIR__,"generate_zhuzhou_instance.jl"))
const MOI=JuMP.MOI
const DATA_DIR=normpath(joinpath(@__DIR__,"..","..","Data","base_data"))

length(ARGS)==3 || error("usage: ... <n> <seed> <output>")
n=parse(Int,ARGS[1]); seed=parse(Int,ARGS[2]); output=ARGS[3]
mkpath(dirname(output))
open(output,"w") do io
    println(io,"RUN_START n=$n seed=$seed scenarios=3 max_stops=4 utc=$(Dates.now(Dates.UTC))")
end

try
    data,_=generate_zhuzhou_data(DATA_DIR,n,16;n_scenarios=3,seed=seed)
    model=AggregateODRouteModel(max(2,ceil(Int,n/2));
        route_regularization_weight=10.0,walk_cost_weight=0.1,
        repositioning_time=20.0,max_walking_distance=600.0,max_wait_time=900.0,
        detour_factor=2.0,max_stops=4,max_visits_per_node=3)
    env=Gurobi.Env(); t0=time()
    result=run_passenger_free_assignment_column_generation(model,data;
        optimizer_env=env,max_cg_iters=2000,n_candidates=20,max_new_columns=20,
        pricing_time_limit_sec=120.0,certification_time_limit_sec=3600.0,
        cluster_time_limit_sec=3600.0,ip_time_limit_sec=1800.0,
        total_time_limit_sec=18000.0,parallel_scenarios=true,
        station_simple_warm_start=false,use_adaptive_cluster_certification=true,
        cluster_initial_num_clusters=max(2,ceil(Int,n/3)),
        cluster_max_num_clusters=n,verify_reduced_costs=true,verbose=true)
    wall=time()-t0; m=result.final_result.model
    has_incumbent=primal_status(m)==MOI.FEASIBLE_POINT
    incumbent=has_incumbent ? objective_value(m) : NaN
    bound=try objective_bound(m) catch; NaN end
    gap=(has_incumbent && isfinite(bound) && abs(incumbent)>1e-9) ?
        100*(incumbent-bound)/abs(incumbent) : NaN
    rows=get(result.final_result.metadata,"cluster_certificate_rows",NamedTuple[])
    open(output,"a") do io
        @printf(io,"SUMMARY n=%d seed=%d status=%s cg_stop_reason=%s lp_bound=%.9f lp_certified=%s mip_termination=%s has_incumbent=%s incumbent_objective=%.9f objective_bound=%.9f mip_gap_pct=%.6f cg_iters=%d rounds=%d columns=%d labels=%d pricing_seconds=%.6f certification_seconds=%.6f total_seconds=%.6f wall=%.6f\n",
            n,seed,string(result.status),string(result.cg_stop_reason),result.lp_bound,
            string(result.lp_bound_certified),string(result.mip_termination),string(has_incumbent),
            incumbent,bound,gap,result.n_cg_iters,result.n_rounds,result.n_columns,
            result.total_labels_generated,result.total_pricing_seconds,
            result.certification_seconds,result.total_seconds,wall)
        println(io,"FINAL_Y_SUPPORT count=$(length(result.open_stations)) stations=$(join(result.open_stations,','))")
        println(io,"UNSERVED count=$(length(result.unserved_passengers)) passengers=$(join(result.unserved_passengers,','))")
        @printf(io,"CLUSTER attempts=%d certificates=%d seconds=%.6f max_K=%d\n",
            length(rows),count(r->r.certified,rows),sum((r.seconds for r in rows);init=0.0),
            maximum((r.num_clusters_after for r in rows);init=0))
        for r in rows
            @printf(io,"CLUSTER_ROW round=%d iteration=%d scenario=%d K_before=%d K_after=%d lb=%.9f certified=%s stop=%s labels=%d seconds=%.6f\n",
                r.round,r.iteration,r.scenario,r.num_clusters_before,r.num_clusters_after,
                r.lower_bound_reduced_cost,string(r.certified),r.stop_reason,
                r.labels_generated,r.seconds)
        end
        println(io,"TERMINATION_REASON cg=$(result.cg_stop_reason) mip=$(result.mip_termination)")
    end
    println(read(output,String))
catch err
    open(output,"a") do io
        println(io,"ERROR type=$(typeof(err)) message=$(sprint(showerror,err))")
        println(io,"TERMINATION_REASON exception")
    end
    showerror(stderr,err,catch_backtrace()); println(stderr)
end
