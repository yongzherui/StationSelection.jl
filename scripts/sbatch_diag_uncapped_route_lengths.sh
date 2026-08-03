#!/bin/bash
# Length distribution of the SELECTED routes in the uncapped optimum, for 4
# cases: three where uncapped certified AND beat ms4 (long routes provably in the
# optimum) plus one control where uncapped==ms4 (expect short routes).
#   0: n25 sc1 s42  (uncapped beat ms4 by -1124)
#   1: n20 sc3 s42  (-282)
#   2: n30 sc3 s43  (-214)
#   3: n30 sc1 s42  (control, mipΔ=0)
#SBATCH --job-name=pfa_routelen
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=24G
#SBATCH --time=01:00:00
#SBATCH --array=0-3

set -euo pipefail
NS=(25 20 30 30)
SC=(1 3 3 1)
SD=(42 42 43 42)
T="${SLURM_ARRAY_TASK_ID:?}"
N="${NS[$T]}"; S="${SC[$T]}"; SEED="${SD[$T]}"

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTDIR="${PFARL_OUTDIR:?set PFARL_OUTDIR}"; mkdir -p "$OUTDIR"

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
        export JULIA_DEPOT_PATH="/tmp/$USER/julia_depot_v${JULIA_VERSION}_${SLURM_ARRAY_JOB_ID}_${T}"
    fi
    mkdir -p "$JULIA_DEPOT_PATH"
    rsync -a --exclude='compiled/' --exclude='logs/' ~/.julia/ "$JULIA_DEPOT_PATH/"
fi

export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
export PFA_N_PAIRS=16
export PFA_N_SCENARIOS="$S"
export PFA_SEEDS="$SEED"
export PFA_MAX_STOPS=0
export PFA_CASE_TIME=2400
export PFA_CERT_TIME=1800
export PFA_PRICING_TIME=120
export PFA_IP_TIME=300

cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
  "$PROJECT_ROOT/scripts/diag_uncapped_route_lengths.jl" "$N" \
  > "$OUTDIR/routelen_n${N}_sc${S}_s${SEED}.txt" 2>&1
echo "wrote $OUTDIR/routelen_n${N}_sc${S}_s${SEED}.txt"
