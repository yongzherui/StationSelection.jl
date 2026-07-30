#!/bin/bash
#SBATCH --job-name=ss_pricing_grid
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=8G
#SBATCH --time=04:00:00

set -uo pipefail

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
    export JULIA_DEPOT_PATH="/tmp/$USER/julia_depot_v${JULIA_VERSION}_${SLURM_JOB_ID}"
fi
mkdir -p "$JULIA_DEPOT_PATH"
rsync -a --exclude='compiled/' --exclude='logs/' ~/.julia/ "$JULIA_DEPOT_PATH/"

cd "$PROJECT_ROOT"
# n = 10..20 at p=16, 3 scenarios (benchmark defaults N_PAIRS=16, N_SCENARIOS=3).
# max_stops=5 exhausts within the time limit at every n in this range.
# Cap each search at 300s so no single case can eat the whole wall budget.
export PFA_TIME_LIMIT=300
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/bench_passenger_free_assignment_labels.jl" \
    --cases 10:5,12:5,14:5,16:5,18:5,20:5
