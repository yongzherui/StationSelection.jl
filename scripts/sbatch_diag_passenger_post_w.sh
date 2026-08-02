#!/bin/bash
#SBATCH --job-name=pfa_postw
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=12G
#SBATCH --time=02:00:00

set -uo pipefail
NVALS=(15 20)
N="${NVALS[${SLURM_ARRAY_TASK_ID:?submit with --array=0-1}]}"
PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"

module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6

export PFALAG_ROUNDS=1
export PFAPOSTW_SAMPLES=20
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/diag_passenger_lagrangian_gap.jl" "$N"
