#!/bin/bash
# Feature A benchmark: does enabling the admissible reduced-cost completion bound
# (and optionally the exact post-wait completion) speed up the direct/exact pricer
# without changing the certified optimum? n=20, phase=direct, cap in {5,6,7}, three
# variants: off (baseline), prune, prune+postw. Compare against exp-1 baselines
# (cap5 5.5s, cap6 18.6s, cap7 47.6s).
#SBATCH --job-name=pfa_prune_bench
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=40G
#SBATCH --time=01:30:00
#SBATCH --array=0-8

set -euo pipefail
N=20
TASK="${SLURM_ARRAY_TASK_ID:?submit with --array=0-8}"
CAP="$((5 + TASK % 3))"
VAR="$((TASK / 3))"           # 0=off 1=prune 2=prune+postw
case "$VAR" in
  0) PRUNE=0; POSTW=0; VTAG=off ;;
  1) PRUNE=1; POSTW=0; VTAG=prune ;;
  2) PRUNE=1; POSTW=1; VTAG=prunepostw ;;
esac
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
export PFASS_MAX_STOPS="$CAP"
export PFASS_PHASE=direct
export PFASS_INTEGRAL_REWARD=0
export PFASS_ROUTING_BOUND=1
export PFASS_EXACT_PRUNE="$PRUNE"
export PFASS_EXACT_POSTW="$POSTW"
export PFASS_ORACLE_TIME=1800

cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
  "$PROJECT_ROOT/scripts/diag_passenger_station_subset_pricing.jl" \
  "$N" "$OUTDIR/n${N}_ms${CAP}_${VTAG}.csv"
