#!/bin/bash
# Census gating the "last contributing pickup" split (bidirectional PFA pricing).
#
# Swept over max_stops, NOT just the usual 7: the entire case for a bidirectional
# split rests on route depth, so measuring only at a cap that forces short routes
# would answer the question for the wrong regime. `ms=0` means uncapped.
#
#   task 0: n=15 ms=7    task 1: n=15 ms=10   task 2: n=15 ms=uncapped
#   task 3: n=20 ms=7    task 4: n=20 ms=10   task 5: n=20 ms=uncapped
#   task 6: n=30 ms=7    task 7: n=30 ms=10
#
# (n=30 uncapped is omitted: it will not exhaust in the search budget, and a
# non-exhausted search biases the Q1 route sample toward whatever best-first found
# early. `search_exhausted` is recorded so this is visible per row.)
#
# All N_PAIRS=16. See notes/2026-08-01_pfa_last_pickup_split_design.md
#SBATCH --job-name=pfa_split_census
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=48G
#SBATCH --time=01:00:00
#SBATCH --array=0-7

set -euo pipefail
NS=(15 15 15 20 20 20 30 30)
MS=(7  10  0  7  10  0  7  10)
TASK="${SLURM_ARRAY_TASK_ID:?submit with --array=0-7}"
N="${NS[$TASK]}"
M="${MS[$TASK]}"
MSLABEL=$([ "$M" = "0" ] && echo "inf" || echo "$M")
PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTDIR="${PFASP_OUTDIR:?set PFASP_OUTDIR}"
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
export PFASP_N_PAIRS=16
export PFASP_MAX_STOPS="$M"
export PFASP_SEARCH_TIME=600

cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
  "$PROJECT_ROOT/scripts/diag_passenger_split_census.jl" \
  "$N" "$OUTDIR/n${N}_ms${MSLABEL}_split_census.csv"
