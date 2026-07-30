#!/bin/bash
# Passenger free-assignment CG scaling grid, LARGE n: 40/50/60 x p, 3 scenarios.
#
# Companion to sbatch_pfa_scaling_grid.sh (n=10..30). Split into its own script
# because at these sizes almost every cell is expected to hit its CG budget
# rather than certify, and the settings are tuned for "terminate cleanly and
# report progress" instead of "finish".
#
# The SLURM wall is deliberately well above the CG budget:
#   3h CG (PFA_CASE_TIME) + 15min final MIP (PFA_IP_TIME) + instance build + IO
# If SLURM kills the process instead, `run_one` never reaches its CSV write and
# the cell vanishes from the study -- so the graceful in-Julia timeout must
# always win the race. The n=30 cells in the first grid confirmed that path
# works: they stopped at 10804s and still wrote a full row with
# certification_exhausted=false.
#
#SBATCH --job-name=pfa_grid_lg
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=05:00:00
#SBATCH --array=0-11
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/pfa-gridlg-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/pfa-gridlg-%A_%a.err

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
STUDY="${PFA_STUDY:-$PROJECT_ROOT/experiments/2026-07-30_pfa_scaling_grid}"

# n=60 is within the 84 stations the Zhuzhou station file provides, and n=40
# already admits 1305 distinct valid OD pairs, so every p below is comfortably
# satisfiable at every n here.
N_LIST=(40 50 60)
P_LIST=(8 16 24 32)

N_COUNT=${#N_LIST[@]}
IDX=${SLURM_ARRAY_TASK_ID:-0}
N_STATIONS=${N_LIST[$((IDX % N_COUNT))]}
N_PAIRS=${P_LIST[$((IDX / N_COUNT))]}

OUTDIR="$STUDY/n${N_STATIONS}_p${N_PAIRS}"
mkdir -p "$OUTDIR"

module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6

cd "$PROJECT_ROOT"
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"

export PFA_N_PAIRS="$N_PAIRS"
export PFA_N_SCENARIOS=3
export PFA_SEEDS=42
export PFA_MAX_STOPS=0
export PFA_MAX_VISITS=3
export PFA_STATION_BUDGET_CAP=0
export PFA_CASE_TIME=10800
export PFA_CERT_TIME=1800
export PFA_PRICING_TIME=120
export PFA_IP_TIME=900

echo "=== cell n=$N_STATIONS p=$N_PAIRS scenarios=3 (array task $IDX) ==="
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/passenger_free_assignment_cg_scaling.jl" "$OUTDIR" "$N_STATIONS"
