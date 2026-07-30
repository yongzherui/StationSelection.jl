#!/bin/bash
# Passenger free-assignment CG scaling grid: n_stations x n_pairs at 3 scenarios.
#
# One array task per (n, p) cell, so every cell gets its OWN 3-hour CG budget
# rather than sharing one -- otherwise a single hard cell starves every cell
# after it in the loop.
#
#SBATCH --job-name=pfa_grid
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=24G
#SBATCH --time=03:45:00
#SBATCH --array=0-19
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/pfa-grid-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/pfa-grid-%A_%a.err

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
STUDY="${PFA_STUDY:-$PROJECT_ROOT/experiments/2026-07-30_pfa_scaling_grid}"

# Grid. Every p here is well inside the number of distinct valid OD pairs the
# order history supports at the smallest n (n=10 has 90), so no cell silently
# gets fewer pairs than requested.
N_LIST=(10 15 20 25 30)
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

# Three scenarios price concurrently (_price_passenger_scenarios), so 3 threads
# are useful and the 4th absorbs GC. More would idle.
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"

export PFA_N_PAIRS="$N_PAIRS"
export PFA_N_SCENARIOS=3
export PFA_SEEDS=42
# Unbounded stops: the regime the recent pricer work targeted, and the honest
# setting for a scaling curve -- an artificial stop cap would flatten it.
export PFA_MAX_STOPS=0
export PFA_MAX_VISITS=3
# Station-budget cap stays off: measured inert on bound quality and slower to
# price (notes/2026-07-30_passenger_pricing_label_search_optimizations.md).
export PFA_STATION_BUDGET_CAP=0
# 3h of CG per cell. Certification is what upgrades a run to :optimality_proven,
# so it gets a real share rather than the 900s default.
export PFA_CASE_TIME=10800
export PFA_CERT_TIME=1800
export PFA_PRICING_TIME=120
export PFA_IP_TIME=900

echo "=== cell n=$N_STATIONS p=$N_PAIRS scenarios=3 (array task $IDX) ==="
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/passenger_free_assignment_cg_scaling.jl" "$OUTDIR" "$N_STATIONS"
