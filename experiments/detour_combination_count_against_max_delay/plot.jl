"""
Plot detour combination proportions vs max delay.

Usage:
    julia --project=. experiments/detour_combination_count_against_max_delay/plot.jl \
        --input experiments/detour_combination_count_against_max_delay/detour_counts.csv \
        --output experiments/detour_combination_count_against_max_delay/detour_proportions.png
"""

using ArgParse
using CSV
using DataFrames
using Plots

function parse_commandline()
    s = ArgParseSettings(
        description = "Plot detour combination proportions vs max delay",
        prog = "plot.jl"
    )

    @add_arg_table! s begin
        "--input", "-i"
            help = "Input CSV path from detour count experiment"
            arg_type = String
            default = joinpath(@__DIR__, "detour_counts.csv")
        "--output", "-o"
            help = "Output image path"
            arg_type = String
            default = joinpath(@__DIR__, "detour_proportions.png")
        "--title"
            help = "Plot title"
            arg_type = String
            default = "Detour combinations as proportion of total triplets"
    end

    return parse_args(s)
end

function main()
    args = parse_commandline()

    df = CSV.read(args["input"], DataFrame)
    if !all(["max_delay", "same_source_proportion", "same_dest_proportion"] .∈ Ref(names(df)))
        error("Input CSV missing required columns: max_delay, same_source_proportion, same_dest_proportion")
    end

    sort!(df, :max_delay)

    plt = plot(
        df.max_delay,
        df.same_source_proportion;
        label="same_source",
        lw=2,
        xlabel="Max delay (s)",
        ylabel="Proportion of total triplets",
        title=args["title"],
        legend=:topleft
    )

    plot!(
        plt,
        df.max_delay,
        df.same_dest_proportion;
        label="same_dest",
        lw=2
    )

    savefig(plt, args["output"])
    println("✓ Wrote plot: $(args["output"]) ")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
