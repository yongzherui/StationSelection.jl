"""
    scripts/analyze_pfa_station_filter_scaling.jl <OUTROOT>

Aggregate the sbatch_pfa_station_filter_scaling.sh grid.

Answers three things:
  1. Effectiveness: joint-LP filter opportunity / endpoint-station / label
     reduction vs n and scenario count.
  2. Correctness: certified optimum (LP bound + MIP) identical between
     `nofilter` and `joint_lp` for every matched (n, scen, seed).
  3. Warm-start integration: per-iteration reduction split by which pricer was
     active (`station_simple` during the warm-start phase vs `revisit` after),
     confirming the filter fires during elementary pricing too.

Reads OUTROOT/<mode>_n<N>_sc<S>_s<SEED>/results/*.csv and .../iters/*_iterations.csv.
"""

using CSV, DataFrames, Statistics, Printf

function _load_all(outroot::String, subdir::String)
    frames = DataFrame[]
    for d in readdir(outroot; join=true)
        isdir(d) || continue
        sd = joinpath(d, subdir)
        isdir(sd) || continue
        for f in readdir(sd; join=true)
            endswith(f, ".csv") || continue
            try
                push!(frames, DataFrame(CSV.File(f)))
            catch err
                @warn "skip $f" exception=err
            end
        end
    end
    isempty(frames) && return DataFrame()
    return reduce(vcat, frames; cols=:union)
end

function main()
    length(ARGS) >= 1 || error("usage: analyze_pfa_station_filter_scaling.jl <OUTROOT>")
    outroot = abspath(ARGS[1])

    results = _load_all(outroot, "results")
    isempty(results) && error("no results CSVs under $outroot")

    # ---- 1. effectiveness (joint_lp rows only) --------------------------------
    println("=== joint-LP filter effectiveness (per case) ===")
    jl = filter(r -> r.station_rc_filter_mode == "joint_lp", results)
    sort!(jl, [:n_stations, :n_scenarios, :seed])
    eff = select(jl,
        :n_stations => :n, :n_scenarios => :scen, :seed,
        :cg_stop_reason,
        :mean_station_filter_positive_rho_reduction => :opp_reduction,
        :mean_station_filter_positive_rho_station_reduction => :stn_reduction,
        :mean_station_filter_excluded_opportunities => :excl_opps,
        :mean_station_filter_excluded_stations => :excl_stns,
        :total_labels_generated => :labels,
        :certification_seconds => :cert_s,
        :total_pricing_seconds => :pricing_s,
    )
    show(eff; allrows=true, allcols=true); println()

    println("\n=== effectiveness aggregated by (n, scen) ===")
    g = combine(groupby(jl, [:n_stations, :n_scenarios]),
        :mean_station_filter_positive_rho_reduction => (x -> mean(skipmissing(x))) => :mean_opp_reduction,
        :mean_station_filter_positive_rho_station_reduction => (x -> mean(skipmissing(x))) => :mean_stn_reduction,
        :mean_station_filter_excluded_opportunities => (x -> mean(skipmissing(x))) => :mean_excl_opps,
        :mean_station_filter_excluded_stations => (x -> mean(skipmissing(x))) => :mean_excl_stns,
    )
    show(sort(g, [:n_stations, :n_scenarios]); allrows=true, allcols=true); println()

    # ---- 2. correctness: joint_lp vs nofilter ---------------------------------
    println("\n=== correctness: joint_lp vs nofilter (matched n,scen,seed) ===")
    key = [:n_stations, :n_scenarios, :seed]
    nf = filter(r -> r.station_rc_filter_mode == "none", results)
    cols = vcat(key, [:lp_bound, :mip_objective, :lp_bound_certified,
                      :total_labels_generated, :total_pricing_seconds,
                      :certification_seconds, :wall_time_sec])
    m = innerjoin(select(jl, cols...), select(nf, cols...);
                  on=key, makeunique=true)
    if isempty(m)
        println("(no matched pairs yet)")
    else
        m.lp_diff = abs.(coalesce.(m.lp_bound, NaN) .- coalesce.(m.lp_bound_1, NaN))
        m.mip_diff = abs.(coalesce.(m.mip_objective, NaN) .- coalesce.(m.mip_objective_1, NaN))
        m.label_delta = coalesce.(m.total_labels_generated, 0) .- coalesce.(m.total_labels_generated_1, 0)
        m.pricing_delta = coalesce.(m.total_pricing_seconds, NaN) .- coalesce.(m.total_pricing_seconds_1, NaN)
        show(select(m, :n_stations => :n, :n_scenarios => :scen, :seed,
                    :lp_diff, :mip_diff,
                    :total_labels_generated_1 => :labels_nofilter,
                    :total_labels_generated => :labels_jointlp,
                    :label_delta, :pricing_delta);
             allrows=true, allcols=true); println()
        @printf("\nmax |LP diff| = %.3e   max |MIP diff| = %.3e\n",
            maximum(skipmissing(m.lp_diff)), maximum(skipmissing(m.mip_diff)))
    end

    # ---- 3. warm-start integration: reduction split by pricer -----------------
    println("\n=== reduction by active pricer (joint_lp iters) ===")
    iters = _load_all(outroot, "iters")
    if isempty(iters) || !("pricer" in names(iters)) ||
            !("station_filter_positive_rho_reduction" in names(iters))
        println("(no per-iteration data with filter columns yet)")
        return
    end
    ji = filter(r -> !ismissing(r.station_filter_positive_rho_reduction), iters)
    if isempty(ji)
        println("(no filter-active iterations logged)")
        return
    end
    byp = combine(groupby(ji, :pricer),
        nrow => :n_iters,
        :station_filter_positive_rho_reduction => (x -> mean(skipmissing(x))) => :mean_opp_reduction,
        :station_filter_excluded_opportunities => (x -> mean(skipmissing(x))) => :mean_excl_opps,
        :station_filter_excluded_stations => (x -> mean(skipmissing(x))) => :mean_excl_stns,
        :station_filter_positive_rho_reduction => (x -> maximum(skipmissing(x); init=0.0)) => :max_opp_reduction,
    )
    show(byp; allrows=true, allcols=true); println()
    println("\n(if a `station_simple` row appears with excl_opps>0, the filter is " *
            "firing during the warm-start elementary phase.)")
end

main()
