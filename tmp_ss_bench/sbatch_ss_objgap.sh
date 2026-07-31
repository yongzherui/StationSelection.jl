#!/bin/bash
#SBATCH --job-name=ss_objgap
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=12G
#SBATCH --time=02:00:00

set -uo pipefail

# One array task per station count. Station counts come from PFASS_NVALS (space
# separated, default "10 15 20"); submit --array over its indices.
#   PFASS_NVALS="25 30 40" sbatch --array=0-2 --export=ALL sbatch_ss_objgap.sh
read -ra NVALS <<< "${PFASS_NVALS:-10 15 20}"
TASK="${SLURM_ARRAY_TASK_ID:?submit with --array=0-<#n minus 1>}"
N="${NVALS[$TASK]}"

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"

module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6
julia --version

JULIA_VERSION=$(julia --startup-file=no -e 'print(VERSION)')
if [ -n "${SLURM_TMPDIR:-}" ]; then
    export JULIA_DEPOT_PATH="$SLURM_TMPDIR/julia_depot_v${JULIA_VERSION}"
else
    export JULIA_DEPOT_PATH="/tmp/$USER/julia_depot_v${JULIA_VERSION}_${SLURM_ARRAY_JOB_ID}_${TASK}"
fi
mkdir -p "$JULIA_DEPOT_PATH"
rsync -a --exclude='compiled/' --exclude='logs/' ~/.julia/ "$JULIA_DEPOT_PATH/"

cd "$PROJECT_ROOT"
# p=16, 3 scenarios, ms=5, seed 42 (defaults). Full CG both ways, objective gap.
echo "===== station-simple vs revisit CG objective gap: n=${N} ====="
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/diag_passenger_station_simple_vs_revisit_objective.jl" \
    "$N" "$PROJECT_ROOT/tmp_ss_bench/objgap_out"
