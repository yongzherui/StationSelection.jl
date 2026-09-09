#!/bin/bash
# Instance-geometry probe: what separates the seeds the relaxed-cluster loop certifies
# from the ones it cannot. Pure data generation + k-medoids, no solve, so it is cheap --
# the wall is dominated by the per-job depot copy and precompile, not the measurement.
#SBATCH --job-name=inst_geom
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=3
#SBATCH --mem=8G
#SBATCH --time=00:40:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/inst-geom-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/inst-geom-%j.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
TASK="${SLURM_JOB_ID}"
# slurm_array_task_env.sh names the per-job depot from the ARRAY vars, which a plain
# (non-array) sbatch never sets -- and `set -u` makes that fatal. Alias them to this job.
export SLURM_ARRAY_JOB_ID="${SLURM_ARRAY_JOB_ID:-$SLURM_JOB_ID}"
export SLURM_ARRAY_TASK_ID="${SLURM_ARRAY_TASK_ID:-0}"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/instance_geometry_vs_certification.jl
