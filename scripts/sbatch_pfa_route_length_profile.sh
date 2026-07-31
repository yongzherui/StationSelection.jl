#!/bin/bash
# Route-length profile of the PFA column pool, on the cheap cells of the
# 2026-07-30 scaling grid (same model/weights/seed/budgets, so the pool is the
# same one the grid produced).
#
#SBATCH --job-name=pfa_rlen
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=24G
#SBATCH --time=02:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/pfa-rlen-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/pfa-rlen-%j.err

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTDIR="${PFA_RLEN_OUT:-$PROJECT_ROOT/experiments/2026-07-30_pfa_scaling_grid/route_lengths}"
mkdir -p "$OUTDIR"

module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6

cd "$PROJECT_ROOT"
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"

# Cheap cells first so the profile exists even if the job is cut short.
# Grid wall times: 6.5 8.1 10.5 17.8 | 8.2 56 158 | 15.8 553 | 1734
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/pfa_route_length_profile.jl" "$OUTDIR" \
    10:8 10:16 10:24 10:32 15:8 15:16 20:8 15:24 20:16 30:8
