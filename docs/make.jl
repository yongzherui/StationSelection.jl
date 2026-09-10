# Docs build. Run it through SLURM, never on a login node -- loading StationSelection loads
# Gurobi: `sbatch scripts/sbatch_build_docs.sh`. That script explains why Documenter comes
# from a stacked `@station-docs-tools` environment instead of a docs/Project.toml (short
# version: a fresh resolve re-runs Gurobi's build step, which hangs).
#
# `prettyurls=false` is deliberate: it emits `page.html` rather than `page/index.html`,
# which is what makes `docs/build/index.html` browsable straight off the filesystem
# (rsync it down, or open it through the ORCD OnDemand portal). Turn it on only if these
# docs are ever served from a web root.
#
# `checkdocs=:exports` with `warnonly=true` is also deliberate: the build must not fail on
# an undocumented export, but it should LIST them. That list is the point -- it is the only
# mechanical inventory of which public names carry no documentation at all.
using Documenter
using StationSelection

makedocs(
    sitename="StationSelection.jl",
    modules=[StationSelection],
    format=Documenter.HTML(
        prettyurls=false,
        # Several docstrings in this package are pages in their own right, so the default
        # 200 KiB page cap trips on them. Warn, don't fail: an over-large page is a
        # readability signal to act on, not a reason to have no docs.
        size_threshold=nothing,
        size_threshold_warn=200 * 1024,
    ),
    pages=[
        "Home" => "index.md",
        "Problems" => "problems.md",
        "Formulations" => "formulations.md",
        "Model construction" => "construction.md",
        "Solvers" => [
            "Overview" => "solvers/index.md",
            "DirectMIPSolver" => "solvers/direct.md",
            "CGSolver" => "solvers/cg.md",
            "BendersSolver" => "solvers/benders.md",
        ],
        "Pricing & label setting" => "pricing.md",
        "Certification" => "certification.md",
        "Results & solve status" => "results.md",
        "Data & mappings" => "data.md",
        "Model building blocks" => "building_blocks.md",
        "Instance generation" => "generators.md",
        "Analysis & output" => "analysis.md",
    ],
    checkdocs=:exports,
    warnonly=true,
    # No "edit on GitHub" links, for two reasons. There is no public repository to link to
    # (this package is a submodule of a private project), and resolving those links makes
    # Documenter call `Pkg.dependencies()` over the package's whole graph -- which FAILS
    # here: the Manifest still carries an `MbedTLS_jll` entry, dropped from the stdlib in
    # Julia 1.12, and Pkg errors with "could not find source path" (job 22476156). Nothing
    # else in the package touches `Pkg.dependencies()`, which is why the test suite has
    # never noticed. `warnonly` does not cover it -- it is a Pkg error, not a doc warning.
    remotes=nothing,
)
