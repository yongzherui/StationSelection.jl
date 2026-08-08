#!/bin/bash
#SBATCH --job-name=pfa_cg_scaling
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --mem=48G
#SBATCH --time=04:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/pfa-cg-scaling-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/pfa-cg-scaling-%j.err

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTDIR="${PFA_OUTDIR:-$PROJECT_ROOT/experiments/passenger_free_assignment_cg_scaling}"

source "$(dirname "${BASH_SOURCE[0]}")/lib/slurm_modules.sh"

cd "$PROJECT_ROOT"
# One Julia thread per allocated CPU: the per-scenario label searches run
# concurrently (see _price_passenger_scenarios). Each concurrent search holds its
# own label pool, hence the larger memory request.
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/passenger_free_assignment_cg_scaling.jl" "$OUTDIR" "$@"
