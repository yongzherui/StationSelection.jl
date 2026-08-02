#!/bin/bash
# Validate feature A on the real hard instance (not toy fixtures): n=30, max_stops=7,
# direct/exact label-setting pricer with reduced-cost pruning ON. Must still certify
# 9260.4; compare labels against the pruning-OFF baseline (6,698,960 labels / 9238s
# from job 19413551 n30_ms7_fairexact). Labels are machine-independent, the clean metric.
#SBATCH --job-name=pfa_n30_prunechk
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=40G
#SBATCH --time=03:30:00

set -euo pipefail
N=30
PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTDIR="${PFASS_OUTDIR:?set PFASS_OUTDIR}"
mkdir -p "$OUTDIR"

JULIA_VERSION="${CS_JULIA_VERSION:-1.12.6}"
module load gcc/12.2.0
module load community-modules
module load StdEnv
module load "${CS_GUROBI_MODULE:-gurobi/12.0.3}"
module load "julia/${JULIA_VERSION}"

# Single standalone job -> shared depot is fine (no concurrent siblings to race).
export JULIA_DEPOT_PATH="${JULIA_DEPOT_PATH:-$HOME/.julia}"

export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
export PFASS_MAX_STOPS=7
export PFASS_PHASE=direct
export PFASS_INTEGRAL_REWARD=0
export PFASS_ROUTING_BOUND=1
export PFASS_EXACT_PRUNE=1
export PFASS_EXACT_POSTW=0
export PFASS_ORACLE_TIME=10800

cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
  "$PROJECT_ROOT/scripts/diag_passenger_station_subset_pricing.jl" \
  "$N" "$OUTDIR/n30_ms7_direct_pruneon.csv"
