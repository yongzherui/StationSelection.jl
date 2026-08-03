#!/bin/bash
# Does a smaller build budget l/n open a reduced-cost gap that the joint-LP
# station filter can exploit? At l=n/2 (job 19523673) closed stations sat at the
# p-median threshold with rc(y_j)~0, so nothing was ever eliminated. This sweeps
# the NEW budgets l=n/4 and l=n/3 (l=n/2 already measured, all zero).
#
# joint_lp only (we want the slack/need + exclusion diagnostics; correctness of
# the filter is already established). Grid: ldiv in {4,3} x n in {20,25,30} x
# scen in {1,3} x seed in {42,43} = 24 tasks. ms4, warm start ON, p=16.
# Each task -> its own OUTDIR/ldiv<D>_n<N>_sc<S>_s<SEED>/ (l recorded in the CSV).
#SBATCH --job-name=pfa_ldiv
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=16G
#SBATCH --time=01:10:00
#SBATCH --array=0-23

set -euo pipefail

LDIVS=(4 3)
NS=(20 25 30)
SCEN=(1 3)
SEEDS=(42 43)

TASK="${SLURM_ARRAY_TASK_ID:?submit with --array=0-23}"
SEED_I=$(( TASK % 2 ))
SCEN_I=$(( (TASK / 2) % 2 ))
N_I=$(( (TASK / 4) % 3 ))
LDIV_I=$(( (TASK / 12) % 2 ))

SEED="${SEEDS[$SEED_I]}"
S="${SCEN[$SCEN_I]}"
N="${NS[$N_I]}"
LDIV="${LDIVS[$LDIV_I]}"

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTROOT="${PFALD_OUTDIR:?set PFALD_OUTDIR}"
OUTDIR="$OUTROOT/ldiv${LDIV}_n${N}_sc${S}_s${SEED}"
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
export PFA_N_PAIRS=16
export PFA_N_SCENARIOS="$S"
export PFA_SEEDS="$SEED"
export PFA_L_DIV="$LDIV"
export PFA_MAX_STOPS=4
export PFA_MAX_CG_ITERS=2000
export PFA_CASE_TIME=1800
export PFA_CERT_TIME=1200
export PFA_PRICING_TIME=60
export PFA_IP_TIME=300
export PFA_STATION_RC_FILTER="${PFALD_FILTER:-joint_lp}"

echo "task=$TASK ldiv=$LDIV n=$N scen=$S seed=$SEED filter=$PFA_STATION_RC_FILTER outdir=$OUTDIR"
cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
  "$PROJECT_ROOT/scripts/passenger_free_assignment_cg_scaling.jl" \
  "$OUTDIR" "$N"
