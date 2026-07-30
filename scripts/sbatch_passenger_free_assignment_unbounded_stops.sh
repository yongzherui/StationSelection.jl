#!/bin/bash
#SBATCH --job-name=pfa_unbounded
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=06:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/pfa-unbounded-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/pfa-unbounded-%j.err

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTDIR="${PFAU_OUTDIR:-$PROJECT_ROOT/experiments/pfa_unbounded_stops}"

module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6

cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/passenger_free_assignment_unbounded_stops.jl" "$OUTDIR" "$@"
