#!/bin/bash
#SBATCH --job-name=rc_audit
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=3
#SBATCH --mem=8G
#SBATCH --time=01:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/rc-audit-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/rc-audit-%j.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/converged_min_reduced_cost_audit.jl
