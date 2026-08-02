#!/bin/bash
# max_stops convergence sweep at fixed n=20, with the direct/exact pricer and the
# station-subset B&B run as SEPARATE jobs so runtime is attributed cleanly to each
# method. 10 tasks = cap in {3,4,5,6,7} x phase in {bnb, direct}.
# K = min(L=10, cap) = cap (sound regime).
#SBATCH --job-name=pfa_ms_sweep
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=40G
#SBATCH --time=01:30:00
#SBATCH --array=0-9

set -euo pipefail
N=20
TASK="${SLURM_ARRAY_TASK_ID:?submit with --array=0-9}"
CAP="$((3 + TASK % 5))"
PHASE=$([[ $((TASK / 5)) -eq 0 ]] && echo bnb || echo direct)
PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTDIR="${PFASS_OUTDIR:?set PFASS_OUTDIR}"
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
    echo "Using existing depot: $JULIA_DEPOT_PATH"
else
    if [ -n "${SLURM_TMPDIR:-}" ]; then
        export JULIA_DEPOT_PATH="$SLURM_TMPDIR/julia_depot_v${JULIA_VERSION}"
    else
        export JULIA_DEPOT_PATH="/tmp/$USER/julia_depot_v${JULIA_VERSION}_${SLURM_ARRAY_JOB_ID}_${TASK}"
    fi
    mkdir -p "$JULIA_DEPOT_PATH"
    rsync -a --exclude='compiled/' --exclude='logs/' ~/.julia/ "$JULIA_DEPOT_PATH/"
    echo "Depot ready: $JULIA_DEPOT_PATH"
fi

export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
export PFASS_MAX_STOPS="$CAP"
export PFASS_PHASE="$PHASE"
export PFASS_INTEGRAL_REWARD=0
export PFASS_ROUTING_BOUND=1
export PFASS_EARLY_TIME=300
export PFASS_ORACLE_TIME=1800
export PFASS_TOTAL_TIME=1800

cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
  "$PROJECT_ROOT/scripts/diag_passenger_station_subset_pricing.jl" \
  "$N" "$OUTDIR/n${N}_ms${CAP}_${PHASE}.csv"
