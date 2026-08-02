#!/bin/bash
#SBATCH --job-name=ss_triple_tests
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=8G
#SBATCH --time=00:40:00

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
WORKDIR="$PROJECT_ROOT/tmp_ss_bench"

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
julia --startup-file=no --project="$PROJECT_ROOT" "$WORKDIR/run_triple_cut_tests.jl"
