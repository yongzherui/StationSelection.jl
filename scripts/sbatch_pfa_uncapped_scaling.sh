#!/bin/bash
# Does TRULY UNCAPPED max_stops (typemax(Int)) stay fast at n=30, or does the
# n=20 "1.7x faster than a finite cap" result break down as n grows?
#
# PFA_MAX_STOPS=0 -> typemax(Int), the genuinely-unbounded path (NOT a large
# finite cap, which sets bounded_max_stops=true and is materially slower). A/B
# partner is the ms4 nofilter grid from job 19523673, same (n,scen,seed).
#
# Grid: n in {20,25,30} x scen in {1,3} x seed in {42,43} = 12 tasks. l=n/2,
# nofilter, warm start ON, p=16. Each task -> its own OUTDIR/unc_n<N>_sc<S>_s<SEED>/.
#SBATCH --job-name=pfa_uncapped
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=24G
#SBATCH --time=01:30:00
#SBATCH --array=0-11

set -euo pipefail

NS=(20 25 30)
SCEN=(1 3)
SEEDS=(42 43)

TASK="${SLURM_ARRAY_TASK_ID:?submit with --array=0-11}"
SEED_I=$(( TASK % 2 ))
SCEN_I=$(( (TASK / 2) % 2 ))
N_I=$(( (TASK / 4) % 3 ))

SEED="${SEEDS[$SEED_I]}"
S="${SCEN[$SCEN_I]}"
N="${NS[$N_I]}"

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTROOT="${PFAUNC_OUTDIR:?set PFAUNC_OUTDIR}"
OUTDIR="$OUTROOT/unc_n${N}_sc${S}_s${SEED}"
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
export PFA_MAX_STOPS=0          # 0 -> typemax(Int) == truly uncapped
export PFA_MAX_CG_ITERS=2000
export PFA_CASE_TIME=2400
export PFA_CERT_TIME=1800
export PFA_PRICING_TIME=120
export PFA_IP_TIME=300
export PFA_STATION_RC_FILTER=0

echo "task=$TASK n=$N scen=$S seed=$SEED UNCAPPED outdir=$OUTDIR"
cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
  "$PROJECT_ROOT/scripts/passenger_free_assignment_cg_scaling.jl" \
  "$OUTDIR" "$N"
