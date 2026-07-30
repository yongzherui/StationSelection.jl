#!/bin/bash
#SBATCH --job-name=diag_qbar_duality
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=00:20:00

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"

module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/diag_certified_qbar_duality.jl"
