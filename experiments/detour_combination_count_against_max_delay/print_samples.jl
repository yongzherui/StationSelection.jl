"""
Print sample detour combinations with travel times.

Usage:
    julia --project=. experiments/detour_combination_count_against_max_delay/print_samples.jl --config <config_file>

Options:
    --max-delay 0.0         # max delay in seconds
    --sample-size 10        # number of samples per mode
    --seed 1                # RNG seed
    --mode both             # same_source | same_dest | both
"""

using ArgParse
using StationSelection
using DataFrames: nrow
using Random
using TOML
using Printf

const PROJECT_ROOT = dirname(dirname(dirname(@__FILE__)))

function parse_commandline()
    s = ArgParseSettings(
        description = "Print sample detour combinations",
        prog = "print_samples.jl"
    )

    @add_arg_table! s begin
        "--config", "-c"
            help = "Path to configuration TOML file"
            arg_type = String
            default = joinpath(PROJECT_ROOT, "example/config.toml")
        "--max-delay"
            help = "Max delay in seconds"
            arg_type = Float64
            default = 0.0
            dest_name = "max_delay"
        "--sample-size"
            help = "Number of samples per mode"
            arg_type = Int
            default = 10
            dest_name = "sample_size"
        "--seed"
            help = "RNG seed"
            arg_type = Int
            default = 1
        "--mode"
            help = "same_source | same_dest | both"
            arg_type = String
            default = "both"
    end

    return parse_args(s)
end

function format_seconds(seconds::Real)
    s_int = round(Int, seconds)
    if s_int < 60
        return @sprintf("%ds", s_int)
    elseif s_int < 3600
        m = s_int ÷ 60
        s = s_int % 60
        return @sprintf("%dm%02ds", m, s)
    else
        h = s_int ÷ 3600
        m = (s_int % 3600) ÷ 60
        s = s_int % 60
        return @sprintf("%dh%02dm%02ds", h, m, s)
    end
end

function load_data(config_path::String)
    config = TOML.parsefile(config_path)

    station_file = joinpath(PROJECT_ROOT, config["paths"]["station_file"])
    order_file = joinpath(PROJECT_ROOT, config["paths"]["order_file"])
    segment_file = joinpath(PROJECT_ROOT, config["paths"]["segment_file"])

    stations = read_candidate_stations(station_file)
    requests = read_customer_requests(
        order_file;
        start_time=config["scenario"]["start_time"],
        end_time=config["scenario"]["end_time"]
    )

    walking_costs = compute_station_pairwise_costs(stations)
    routing_costs = read_routing_costs_from_segments(segment_file, stations)

    scenarios = [(config["scenario"]["start_time"], config["scenario"]["end_time"])]

    data = create_station_selection_data(
        stations,
        requests,
        walking_costs;
        routing_costs=routing_costs,
        scenarios=scenarios
    )

    return data, config, nrow(requests)
end

function print_triplet(data::StationSelectionData, j::Int, k::Int, l::Int; label="")
    t_jk = get_routing_cost(data, j, k)
    t_kl = get_routing_cost(data, k, l)
    t_jl = get_routing_cost(data, j, l)

    detour = t_jk + t_kl
    slack = detour - t_jl
    ratio = t_jl == 0 ? 0.0 : detour / t_jl

    header = isempty(label) ? "" : "[$label] "
    println("  $(header)stations: j=$j, k=$k, l=$l")
    println("    t(j->k)=$(format_seconds(t_jk))  t(k->l)=$(format_seconds(t_kl))  t(j->l)=$(format_seconds(t_jl))")
    println("    detour=$(format_seconds(detour))  slack=$(format_seconds(slack))  ratio=$(round(ratio, digits=4))")
end

function sample_combinations!(rng::AbstractRNG, combos, sample_size::Int)
    n = length(combos)
    if n == 0
        return Tuple[]
    end
    if n <= sample_size
        return combos
    end
    idx = rand(rng, 1:n, sample_size)
    return combos[idx]
end

function main()
    args = parse_commandline()

    println("=" ^ 60)
    println("Sample Detour Combinations")
    println("=" ^ 60)

    println("\n[1] Loading data from: $(args["config"]) ")
    data, config, request_count = load_data(args["config"])

    println("  - Stations: $(data.n_stations)")
    println("  - Requests: $(request_count)")

    mc = config["model"]
    k = mc["k"]
    l = mc["l"]
    routing_weight = mc["routing_weight"]
    time_window = get(mc, "time_window", nothing)
    routing_delay = get(mc, "routing_delay", nothing)
    if isnothing(time_window) || isnothing(routing_delay)
        error("Config model section must include time_window and routing_delay for detour experiments.")
    end

    max_delay = args["max_delay"]
    model = TwoStageSingleDetourModel(k, l, routing_weight, time_window, max_delay)

    rng = MersenneTwister(args["seed"])
    sample_size = args["sample_size"]
    mode = lowercase(args["mode"])

    if mode == "same_source" || mode == "both"
        combos = find_same_source_detour_combinations(model, data)
        println("\n[2] same_source combinations")
        println("  - total: $(length(combos)) (max_delay=$(max_delay)s)")
        for (j, k, l) in sample_combinations!(rng, combos, sample_size)
            print_triplet(data, j, k, l; label="same_source")
        end
    end

    if mode == "same_dest" || mode == "both"
        combos = find_same_dest_detour_combinations(model, data)
        println("\n[3] same_dest combinations")
        println("  - total: $(length(combos)) (max_delay=$(max_delay)s)")
        n = length(combos)
        if n == 0
            return
        end
        picks = sample_size < n ? rand(rng, 1:n, sample_size) : collect(1:n)
        for idx in picks
            (j, k, l, time_delta) = combos[idx]
            print_triplet(data, j, k, l; label="same_dest, Δt=$(time_delta)")
        end
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
