#!/bin/bash
# Station-subset B&B certification vs full-network exact pricer, capped at 7 stops.
# n=30 (L=15) and n=40 (L=20): K = min(L, max_stops) = 7 = max_stops, so distinct
# stations <= route_length <= 7 makes K a valid distinct-station cap -> the B&B
# certification is provably equivalent to the full-network exact price. The
# baseline (full_network_exact) also runs at max_stops=7, so it is apples-to-apples.
#SBATCH --job-name=pfa_subset_ms7
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=40G
#SBATCH --time=05:00:00
#SBATCH --array=0-1

set -euo pipefail
NVALS=(30 40)
TASK="${SLURM_ARRAY_TASK_ID:?submit with --array=0-1}"
N="${NVALS[$TASK]}"
PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTDIR="${PFASS_OUTDIR:?set PFASS_OUTDIR}"
mkdir -p "$OUTDIR"

JULIA_VERSION="${CS_JULIA_VERSION:-1.12.6}"
module load gcc/12.2.0
module load community-modules
module load StdEnv
module load "${CS_GUROBI_MODULE:-gurobi/12.0.3}"
module load "julia/${JULIA_VERSION}"

# Concurrent array tasks: copy the depot per task (excluding compiled cache) so
# sibling tasks never collide on the precompilation lock.
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
export PFASS_MAX_STOPS=7
export PFASS_INTEGRAL_REWARD=0
export PFASS_ROUTING_BOUND=1
# Bound each phase independently; a timed-out run still writes its incumbent,
# open-node upper bound, and globally_certified=false.
export PFASS_EARLY_TIME=900
export PFASS_ORACLE_TIME=1800
export PFASS_TOTAL_TIME=10800

cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
  "$PROJECT_ROOT/scripts/diag_passenger_station_subset_pricing.jl" \
  "$N" "$OUTDIR/n${N}_ms7.csv"
