#!/bin/bash
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=01:00:00

# direct_enumeration_guide vs plain lifted BendersYZ comparison on the real sample_09
# fixture, ONE (n_stations, guided) config per job -- submit separately per config so
# a slow config can't starve a fast one of shared wall-clock budget. Reads DEG_N_STATIONS
# / DEG_GUIDED from the environment (set via `sbatch --export=...` at submission time),
# and passes an optional $1 outdir through to the Julia script.
#
# Usage (submit one job per config):
#   sbatch --job-name=deg_n15_guided \
#          --output=.../sample09-deg-n15-guided-%j.out --error=.../sample09-deg-n15-guided-%j.err \
#          --export=ALL,DEG_N_STATIONS=15,DEG_GUIDED=true \
#          scripts/sbatch_sample09_direct_enumeration_guide_compare.sh [outdir]

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTDIR="${1:-}"

echo "===== sample09 direct_enumeration_guide compare (BendersYZ, restricted_mw_fixed_pi) ====="
echo "Job ID: $SLURM_JOB_ID"
echo "Node: $SLURM_NODELIST"
echo "DEG_N_STATIONS=${DEG_N_STATIONS:-unset}  DEG_GUIDED=${DEG_GUIDED:-unset}"
echo "Start time: $(date)"
echo ""

module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6
julia --version
echo ""

cd "$PROJECT_ROOT"
echo "Working directory: $(pwd)"

julia --startup-file=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/sample09_direct_enumeration_guide_compare.jl" "$OUTDIR"
RUN_EXIT=$?

echo ""
echo "===== Run complete ====="
echo "Exit code: $RUN_EXIT"
echo "End time: $(date)"

exit $RUN_EXIT
