#!/bin/bash
# First end-to-end Benders solve of the joint routing+assignment model, plus its
# monolithic exactness references. See benders_direct_arm_and_flatness.jl's docstring.
#SBATCH --job-name=benders_da
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=64G
#SBATCH --time=03:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-da-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-da-%j.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
# slurm_array_task_env.sh names its per-job depot from the ARRAY vars, which a plain
# (non-array) sbatch never sets -- and `set -u` makes that fatal. Alias them to this job.
export SLURM_ARRAY_JOB_ID="${SLURM_ARRAY_JOB_ID:-$SLURM_JOB_ID}"
export SLURM_ARRAY_TASK_ID="${SLURM_ARRAY_TASK_ID:-0}"
TASK="${SLURM_ARRAY_TASK_ID}"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/benders_direct_arm_and_flatness.jl
