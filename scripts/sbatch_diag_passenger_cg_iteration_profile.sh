#!/bin/bash
# Per-iteration CG profile (time-per-iteration as the pool grows), with pruning ON
# (the default). Covers instance size and scenario count:
#   task 0: n=30 scen=1   task 1: n=40 scen=1   task 2: n=30 scen=3
# All N_PAIRS=16, max_stops=7.
#SBATCH --job-name=pfa_cg_iterprof
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=40G
#SBATCH --time=01:15:00
#SBATCH --array=0-2

set -euo pipefail
NS=(30 40 30)
SCEN=(1 1 3)
TASK="${SLURM_ARRAY_TASK_ID:?submit with --array=0-2}"
N="${NS[$TASK]}"
S="${SCEN[$TASK]}"
PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTDIR="${PFACG_OUTDIR:?set PFACG_OUTDIR}"
mkdir -p "$OUTDIR"

JULIA_VERSION="${CS_JULIA_VERSION:-1.12.6}"
module load gcc/12.2.0
module load community-modules
module load StdEnv
module load "${CS_GUROBI_MODULE:-gurobi/12.0.3}"
module load "julia/${JULIA_VERSION}"

COPY_DEPOT="${CS_COPY_DEPOT:-1}"
if [ "$COPY_DEPOT" = "0" ]; then
    export JULIA_DEPOT_PATH="${JULIA_DEPOT_PATH:-$HOME/.julia}"
else
    if [ -n "${SLURM_TMPDIR:-}" ]; then
        export JULIA_DEPOT_PATH="$SLURM_TMPDIR/julia_depot_v${JULIA_VERSION}"
    else
        export JULIA_DEPOT_PATH="/tmp/$USER/julia_depot_v${JULIA_VERSION}_${SLURM_ARRAY_JOB_ID}_${TASK}"
    fi
    mkdir -p "$JULIA_DEPOT_PATH"
    rsync -a --exclude='compiled/' --exclude='logs/' ~/.julia/ "$JULIA_DEPOT_PATH/"
fi

export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
export PFACG_N_PAIRS=16
export PFACG_N_SCENARIOS="$S"
export PFACG_MAX_STOPS=7
export PFACG_MAX_ITERS=400
export PFACG_TOTAL_TIME=2400

cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
  "$PROJECT_ROOT/scripts/diag_passenger_cg_iteration_profile.jl" \
  "$N" "$OUTDIR/n${N}_scen${S}_ms7_iterprofile.csv"
