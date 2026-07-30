#!/bin/bash
#SBATCH --job-name=diag_pass_vs_agg
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=01:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/diag-pass-vs-agg-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/diag-pass-vs-agg-%j.err

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"

module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/diag_passenger_cg_vs_aggregate_direct.jl" "$@"
