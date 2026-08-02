#!/bin/bash
# Fair-time baseline: give the full-network exact pricer the SAME 3h budget the
# station-subset B&B gets (job 19412214), so a B&B "win" can't be an artifact of
# the pricer's shorter 1800s cap. Only the full_network_exact row matters here;
# the certification phase is neutered (TOTAL_TIME=1) since the B&B is measured in
# the sibling job. Cap 7, n=30 (L=15) / n=40 (L=20): K = 7 = max_stops (sound).
#SBATCH --job-name=pfa_ms7_fairexact
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=40G
#SBATCH --time=04:00:00
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
export PFASS_EARLY_TIME=300
# The one that matters: exact pricer gets the same 3h the B&B got.
export PFASS_ORACLE_TIME=10800
# Neuter the certification phase in this job (B&B is measured in job 19412214).
export PFASS_TOTAL_TIME=1

cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
  "$PROJECT_ROOT/scripts/diag_passenger_station_subset_pricing.jl" \
  "$N" "$OUTDIR/n${N}_ms7_fairexact.csv"
