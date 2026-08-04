#!/bin/bash
# Small-instance route/rho census for learning station-subset heuristics.
# Four cells: scenarios {1,3} x seeds {42,43}. Pricing retains up to 10,000
# improving routes individually so the route-level correlations are not a
# 20-column batch artifact.
#SBATCH --job-name=pfa_n15_rho
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=24G
#SBATCH --time=01:00:00
#SBATCH --array=0-3

set -euo pipefail
SC=(1 1 3 3)
SD=(42 43 42 43)
T="${SLURM_ARRAY_TASK_ID:?}"
S="${SC[$T]}"
SEED="${SD[$T]}"

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTDIR="${PFARHO_OUTDIR:?set PFARHO_OUTDIR}"
mkdir -p "$OUTDIR"

JULIA_VERSION="${CS_JULIA_VERSION:-1.12.6}"
module load gcc/12.2.0
module load community-modules
module load StdEnv
module load "${CS_GUROBI_MODULE:-gurobi/12.0.3}"
module load "julia/${JULIA_VERSION}"

export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
export PFA_N_PAIRS=16
export PFA_N_SCENARIOS="$S"
export PFA_SEEDS="$SEED"
export PFA_MAX_STOPS=4
export PFA_MAX_VISITS=3
export PFA_N_CANDIDATES="${PFARHO_N_CANDIDATES:-10000}"
export PFA_MAX_NEW_COLUMNS="${PFARHO_MAX_NEW_COLUMNS:-10000}"
export PFA_EXHAUSTIVE_EACH_ITER="${PFARHO_EXHAUSTIVE_EACH_ITER:-0}"
export PFA_THETA_RHO_CORE_SIZE="${PFARHO_THETA_RHO_CORE_SIZE:-0}"
export PFA_THETA_RHO_OUTSIDERS="${PFARHO_THETA_RHO_OUTSIDERS:-1}"
export PFA_CASE_TIME=2400
export PFA_CERT_TIME=1200
export PFA_PRICING_TIME=300
export PFA_IP_TIME=300
export PFA_OUTDIR="$OUTDIR"

cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
  "$PROJECT_ROOT/scripts/diag_y_support_churn.jl" 15 \
  > "$OUTDIR/n15_sc${S}_s${SEED}.txt" 2>&1
