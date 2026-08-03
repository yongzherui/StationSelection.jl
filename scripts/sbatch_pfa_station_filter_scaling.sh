#!/bin/bash
# Station-elimination (joint-LP) filter effectiveness at larger n.
#
# Question: how much does the joint-LP station reduced-cost filter shrink the
# pricing graph (positive opportunities / endpoint stations / generated labels)
# as n grows, and does it preserve the certified optimum? Also: does it fire
# during the station-simple warm-start phase (per-iteration rows carry `pricer`).
#
# Grid: mode in {nofilter, joint_lp} x n in {20,25,30} x scen in {1,3} x seed in {42,43}.
# One case per array task (24 total). ms4, warm start ON (library default), p=16.
# Each task writes into its own OUTDIR/case_<task>/ so nothing collides; the
# per-case results/<case>.csv and iters/<case>_iterations.csv are the deliverables.
#SBATCH --job-name=pfa_stnfilter
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=16G
#SBATCH --time=01:10:00
#SBATCH --array=0-23

set -euo pipefail

MODEVAL=(0 joint_lp)          # value passed to PFA_STATION_RC_FILTER
MODELBL=(nofilter joint_lp)   # label used in the output dir name
NS=(20 25 30)
SCEN=(1 3)
SEEDS=(42 43)

TASK="${SLURM_ARRAY_TASK_ID:?submit with --array=0-23}"
SEED_I=$(( TASK % 2 ))
SCEN_I=$(( (TASK / 2) % 2 ))
N_I=$(( (TASK / 4) % 3 ))
MODE_I=$(( (TASK / 12) % 2 ))

SEED="${SEEDS[$SEED_I]}"
S="${SCEN[$SCEN_I]}"
N="${NS[$N_I]}"
MODE="${MODEVAL[$MODE_I]}"
MLBL="${MODELBL[$MODE_I]}"

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTROOT="${PFASF_OUTDIR:?set PFASF_OUTDIR}"
OUTDIR="$OUTROOT/${MLBL}_n${N}_sc${S}_s${SEED}"
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
export PFA_MAX_STOPS=4
export PFA_MAX_CG_ITERS=2000
export PFA_CASE_TIME=1800
export PFA_CERT_TIME=1200
export PFA_PRICING_TIME=60
export PFA_IP_TIME=300
export PFA_STATION_RC_FILTER="$MODE"

echo "task=$TASK mode=$MLBL n=$N scen=$S seed=$SEED filter=$MODE outdir=$OUTDIR"
cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
  "$PROJECT_ROOT/scripts/passenger_free_assignment_cg_scaling.jl" \
  "$OUTDIR" "$N"
