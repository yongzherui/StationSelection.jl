#!/bin/bash
# Two-stop seeding A/B: correctness gate + coverage-phase savings.
#
#SBATCH --job-name=pfa_seedab
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=04:30:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/pfa-seedab-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/pfa-seedab-%j.err

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTDIR="${PFA_AB_OUT:-$PROJECT_ROOT/experiments/2026-07-30_pfa_scaling_grid/seed_ab}"
mkdir -p "$OUTDIR"

module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6

cd "$PROJECT_ROOT"
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
export PFA_CASE_TIME="${PFA_CASE_TIME:-1800}"

stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/pfa_seed_ab.jl" "$OUTDIR" \
    ${PFA_AB_CELLS:-10:8 10:16 15:8 20:8 10:24 15:16}
