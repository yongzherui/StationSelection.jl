#!/bin/bash
#SBATCH --job-name=diag_mw_lb_debug
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=00:30:00

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"

module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6

cd "$PROJECT_ROOT"
export CS_DEBUG_LIFTED_LB=1
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/zhuzhou_p16_scaling_route100x_task.jl" \
    "$PROJECT_ROOT/experiments/zhuzhou_p16_scaling_route100x_debug" 15 123 bendersYZ_mw_ms4_lifted_lb
