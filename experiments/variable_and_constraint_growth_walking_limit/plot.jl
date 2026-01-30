"""
Plot variable/constraint growth vs walking distance limit.

Usage:
    julia --project=. experiments/variable_and_constraint_growth_walking_limit/plot.jl \
        --input experiments/variable_and_constraint_growth_walking_limit/walking_limit_growth.csv \
        --output experiments/variable_and_constraint_growth_walking_limit/walking_limit_growth.png
"""

using ArgParse
using CSV
using DataFrames
using Plots
using Logging

function parse_commandline()
    s = ArgParseSettings(
        description = "Plot variable/constraint growth vs walking distance limit",
        prog = "plot.jl"
    )

    @add_arg_table! s begin
        "--input", "-i"
            help = "Input CSV path"
            arg_type = String
            default = joinpath(@__DIR__, "walking_limit_growth.csv")
        "--output", "-o"
            help = "Output image path"
            arg_type = String
            default = joinpath(@__DIR__, "walking_limit_growth.png")
        "--title"
            help = "Plot title"
            arg_type = String
            default = "Variable/Constraint Growth vs Walking Distance Limit"
        "--reference-line"
            help = "X position for reference vertical line (m)"
            arg_type = Float64
            default = 400.0
    end

    return parse_args(s)
end

function main()
    global_logger(SimpleLogger(stderr, Logging.Error))

    args = parse_commandline()

    df = CSV.read(args["input"], DataFrame)
    required = ["walking_distance", "total_variables", "total_constraints"]
    if !all(required .∈ Ref(names(df)))
        error("Input CSV missing required columns: $(required)")
    end

    sort!(df, :walking_distance)

    # Create the main plot
    plt = plot(
        df.walking_distance,
        df.total_variables;
        label="Total Variables",
        lw=2,
        color=:blue,
        xlabel="Maximum Walking Distance (m)",
        ylabel="Count",
        title=args["title"],
        legend=:topleft,
        size=(900, 600),
        margin=5Plots.mm
    )

    scatter!(
        plt,
        df.walking_distance,
        df.total_variables;
        label="",
        ms=4,
        color=:blue
    )

    plot!(
        plt,
        df.walking_distance,
        df.total_constraints;
        label="Total Constraints",
        lw=2,
        color=:orange
    )

    scatter!(
        plt,
        df.walking_distance,
        df.total_constraints;
        label="",
        ms=4,
        color=:orange
    )

    # Add reference line at specified walking distance (default 400m)
    ref_line = args["reference-line"]
    vline!(plt, [ref_line];
        label="$(Int(ref_line))m reference",
        color=:red,
        linestyle=:dash,
        lw=2
    )

    savefig(plt, args["output"])
    println("✓ Wrote plot: $(args["output"])")

    # Detailed breakdown plots for variable/constraint subtypes
    var_cols = filter(name -> startswith(String(name), "var_"), names(df))
    con_cols = filter(name -> startswith(String(name), "con_"), names(df))

    if !isempty(var_cols)
        output_vars = replace(args["output"], ".png" => "_vars.png")
        plt_vars = plot(
            xlabel="Maximum Walking Distance (m)",
            ylabel="Count",
            title="Variable Breakdown vs Walking Distance Limit",
            legend=:topleft,
            size=(900, 600),
            margin=5Plots.mm
        )

        palette = Plots.palette(:tab10)
        for (i, col) in enumerate(sort(var_cols))
            label = replace(String(col), "var_" => "")
            color = palette[mod1(i, length(palette))]
            plot!(plt_vars, df.walking_distance, df[!, col]; label=label, lw=2, color=color)
            scatter!(plt_vars, df.walking_distance, df[!, col]; label="", ms=3, color=color)
        end

        vline!(plt_vars, [ref_line];
            label="$(Int(ref_line))m reference",
            color=:red,
            linestyle=:dash,
            lw=2
        )

        savefig(plt_vars, output_vars)
        println("✓ Wrote variable breakdown plot: $output_vars")
    end

    if !isempty(con_cols)
        output_cons = replace(args["output"], ".png" => "_constraints.png")
        plt_cons = plot(
            xlabel="Maximum Walking Distance (m)",
            ylabel="Count",
            title="Constraint Breakdown vs Walking Distance Limit",
            legend=:topleft,
            size=(900, 600),
            margin=5Plots.mm
        )

        palette = Plots.palette(:tab10)
        for (i, col) in enumerate(sort(con_cols))
            label = replace(String(col), "con_" => "")
            color = palette[mod1(i, length(palette))]
            plot!(plt_cons, df.walking_distance, df[!, col]; label=label, lw=2, color=color)
            scatter!(plt_cons, df.walking_distance, df[!, col]; label="", ms=3, color=color)
        end

        vline!(plt_cons, [ref_line];
            label="$(Int(ref_line))m reference",
            color=:red,
            linestyle=:dash,
            lw=2
        )

        savefig(plt_cons, output_cons)
        println("✓ Wrote constraint breakdown plot: $output_cons")
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
