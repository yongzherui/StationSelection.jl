#!/bin/bash
# CG scaling study on Zhuzhou data: how does full column generation scale, and can
# it certify? Widened grid: n_stations in {10,15,20,25,30,40} x n_scenarios in
# {1,3,5,10,15}, p=16, max_stops=7, pruning ON (default). Long budget so the hard
# corners (n=40, scen=15) get a real chance to certify: 3h CG total, 1h certification
# pass. Each run writes a per-iteration CSV and a one-row .summary.csv.
#SBATCH --job-name=pfa_cg_scale
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=24G
#SBATCH --time=03:30:00
#SBATCH --array=0-29

set -euo pipefail
NS=(10 15 20 25 30 40)
SCEN=(1 3 5 10 15)
TASK="${SLURM_ARRAY_TASK_ID:?submit with --array=0-29}"
N="${NS[$((TASK % 6))]}"
S="${SCEN[$((TASK / 6))]}"
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

cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
  "$PROJECT_ROOT/scripts/diag_passenger_cg_iteration_profile.jl" \
  "$N" "$OUTDIR/n${N}_scen${S}_ms7_iterprofile.csv"
