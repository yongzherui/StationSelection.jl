"""
Plot variable/constraint growth vs station limit.

Usage:
    julia --project=. experiments/variable_and_constraint_growth/plot.jl \
        --input experiments/variable_and_constraint_growth/variable_constraint_growth.csv \
        --output experiments/variable_and_constraint_growth/variable_constraint_growth.png
"""

using ArgParse
using CSV
using DataFrames
using Plots

function parse_commandline()
    s = ArgParseSettings(
        description = "Plot variable/constraint growth",
        prog = "plot.jl"
    )

    @add_arg_table! s begin
        "--input", "-i"
            help = "Input CSV path"
            arg_type = String
            default = joinpath(@__DIR__, "variable_constraint_growth.csv")
        "--output", "-o"
            help = "Output image path"
            arg_type = String
            default = joinpath(@__DIR__, "variable_constraint_growth.png")
        "--title"
            help = "Plot title"
            arg_type = String
            default = "Variable/Constraint Growth vs Station Limit"
    end

    return parse_args(s)
end

function main()
    args = parse_commandline()

    df = CSV.read(args["input"], DataFrame)
    required = ["n_stations", "total_variables", "total_constraints"]
    if !all(required .∈ Ref(names(df)))
        error("Input CSV missing required columns: $(required)")
    end

    sort!(df, :n_stations)

    plt = plot(
        df.n_stations,
        df.total_variables;
        label="variables",
        lw=2,
        xlabel="Number of stations",
        ylabel="Count",
        title=args["title"],
        legend=:topleft
    )

    scatter!(
        plt,
        df.n_stations,
        df.total_variables;
        label="",
        ms=5
    )

    plot!(
        plt,
        df.n_stations,
        df.total_constraints;
        label="constraints",
        lw=2
    )

    scatter!(
        plt,
        df.n_stations,
        df.total_constraints;
        label="",
        ms=5
    )

    savefig(plt, args["output"])
    println("✓ Wrote plot: $(args["output"]) ")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
