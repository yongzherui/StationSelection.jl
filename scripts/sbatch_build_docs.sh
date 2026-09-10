#!/bin/bash
#SBATCH --job-name=station-docs
#SBATCH --partition=mit_preemptable
# Same reasoning as sbatch_run_tests.sh: short job, preemptable starts in minutes. A
# preempted job shows as CANCELLED/PREEMPTED, not a build failure -- check the state before
# reading missing HTML as a broken build.
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=01:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/station-docs-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/station-docs-%j.err

set -euo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"

# Absolute path: under sbatch, BASH_SOURCE points at SLURM's copy of this script in
# /var/spool/slurmd/, where scripts/lib/ does not exist.
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"

cd "$PROJECT_ROOT"

# Documenter lives in a SHARED named environment, not in a docs/Project.toml, and the build
# then runs in the PACKAGE's own environment with that shared one stacked behind it.
#
# The obvious layout -- docs/Project.toml listing Documenter plus a `Pkg.develop`ed
# StationSelection -- does not work here. A fresh environment re-resolves the whole
# dependency graph, which re-runs Gurobi's build step under a new environment hash, and that
# build HANGS: measured 2026-09-10 (job 22475077) sitting on `Building Gurobi` for 10+
# minutes with an empty build.log and no CPU, killed rather than waited out. Gurobi is
# already built in the package environment, so the fix is to never leave it: `@` is the
# package (already instantiated, already built), `@station-docs-tools` supplies Documenter,
# and no resolve touches Gurobi.
#
# Keep it this way. Reintroducing a docs/Project.toml reintroduces the hang.
julia --startup-file=no --project=@station-docs-tools -e '
    using Pkg
    haskey(Pkg.project().dependencies, "Documenter") || Pkg.add("Documenter")
'

JULIA_LOAD_PATH="@:@station-docs-tools:@stdlib" \
    julia --startup-file=no --project="$PROJECT_ROOT" "$PROJECT_ROOT/docs/make.jl"

echo "docs written to $PROJECT_ROOT/docs/build/index.html"
