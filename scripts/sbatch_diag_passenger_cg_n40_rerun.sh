#!/bin/bash
# Recover the n=40 column of the CG scaling grid: those cells OOM'd at 24G (n=40 ms7
# label search is memory-hungry). Rerun at 64G, and bound the final MIP to 120s so it
# can't overrun the wall before the summary writes (the study cares about LP-bound
# certification, not the integer solution). scen15 already produced a valid
# (not-certified) result, so only rerun scen in {1,3,5,10}.
#SBATCH --job-name=pfa_cg_n40
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=64G
#SBATCH --time=04:00:00
#SBATCH --array=0-3

set -euo pipefail
N=40
SCEN=(1 3 5 10)
TASK="${SLURM_ARRAY_TASK_ID:?submit with --array=0-3}"
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
export PFACG_MAX_ITERS=2000
export PFACG_TOTAL_TIME=10800
export PFACG_CERT_TIME=3600
export PFACG_PRICING_TIME=120
export PFACG_IP_TIME=120

cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
  "$PROJECT_ROOT/scripts/diag_passenger_cg_iteration_profile.jl" \
  "$N" "$OUTDIR/n${N}_scen${S}_ms7_iterprofile.csv"
