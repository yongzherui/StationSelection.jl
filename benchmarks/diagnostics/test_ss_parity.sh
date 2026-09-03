#!/bin/bash
#SBATCH --job-name=ss_parity
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=16G
#SBATCH --time=01:30:00
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
TASK=ssparity
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
export CS_COPY_DEPOT=0
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" -e 'using Pkg; Pkg.test()'
