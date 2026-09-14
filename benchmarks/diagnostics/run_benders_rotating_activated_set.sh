#!/bin/bash
# Rotate r "most hopeful" unbuilt stations into the priced set each Benders iteration.
# Strict completion only -- no row generation, no LPO. See benders_rotating_activated_set.jl.
#SBATCH --job-name=benders_ra
#SBATCH --partition=mit_preemptable
#SBATCH --requeue
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=3
#SBATCH --mem=16G
#SBATCH --time=01:30:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-ra-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-ra-%A_%a.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
# Real array now, so SLURM sets these; the aliases only matter for a bare `sbatch` of one
# arm. Note CS_COPY_DEPOT=0 is passed at submit time DELIBERATELY even though this is an
# array: the per-job depot copy exists to stop concurrent tasks racing on the precompile
# cache, and there is nothing to write here -- the package is unchanged since the last
# successful load, so every task is a pure reader of a warm shared cache. That skips 12
# rsyncs and 12 x ~7min of recompilation. Drop the flag if src/ has changed.
export SLURM_ARRAY_JOB_ID="${SLURM_ARRAY_JOB_ID:-$SLURM_JOB_ID}"
export SLURM_ARRAY_TASK_ID="${SLURM_ARRAY_TASK_ID:-0}"
TASK="${SLURM_ARRAY_TASK_ID}"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/benders_rotating_activated_set.jl
