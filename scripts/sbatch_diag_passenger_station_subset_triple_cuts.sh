#!/bin/bash
# Experiment 2: do k=3 (triple) routing cuts tighten the reward+routing LP bound
# that is the certification bottleneck at cap 7? Two measurements at cap=7:
#   - root LP bound (node_limit=1): triple OFF vs ON at n=20 and n=30
#   - full certification (node_limit=0, 30min): does triple ON certify n=20 cap7?
# All phase=bnb (triples only affect the B&B). TRIPLE_ALTS=5 gives the cuts a fair
# shot without blowing up the a-priori triple enumeration.
#SBATCH --job-name=pfa_triple
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=40G
#SBATCH --time=01:00:00
#SBATCH --array=0-5

set -euo pipefail
# task: N TRIPLE NODELIMIT TOTAL  label
NS=(20 20 30 30 20 20)
TRIPLES=(0 1 0 1 1 0)
NODELIMITS=(1 1 1 1 0 0)
TOTALS=(60 60 60 60 1800 1800)
TASK="${SLURM_ARRAY_TASK_ID:?submit with --array=0-5}"
N="${NS[$TASK]}"
TRIPLE="${TRIPLES[$TASK]}"
NODELIMIT="${NODELIMITS[$TASK]}"
TOTAL="${TOTALS[$TASK]}"
TAG=$([[ "$TRIPLE" == 1 ]] && echo on || echo off)
KIND=$([[ "$NODELIMIT" == 1 ]] && echo root || echo fullcert)

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
export PFASS_MAX_STOPS=7
export PFASS_PHASE=bnb
export PFASS_INTEGRAL_REWARD=0
export PFASS_ROUTING_BOUND=1
export PFASS_TRIPLE="$TRIPLE"
export PFASS_TRIPLE_ALTS=5
export PFASS_NODE_LIMIT="$NODELIMIT"
export PFASS_EARLY_TIME=300
export PFASS_ORACLE_TIME=300
export PFASS_TOTAL_TIME="$TOTAL"

cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
  "$PROJECT_ROOT/scripts/diag_passenger_station_subset_pricing.jl" \
  "$N" "$OUTDIR/n${N}_ms7_${KIND}_triple${TAG}.csv"
