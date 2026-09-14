#!/bin/bash
# Is the detour term worth crediting? One instance per array task. See
# benders_detour_credit_potential.jl -- it prints a VIABLE / NOT VIABLE verdict.
#
# Usage: sbatch --array=1-4 benchmarks/diagnostics/run_benders_detour_credit_potential.sh
#SBATCH --job-name=benders_dc
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=3
#SBATCH --mem=14G
#SBATCH --time=02:00:00
#SBATCH --array=1-4
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-dc-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-dc-%A_%a.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
export SLURM_ARRAY_JOB_ID="${SLURM_ARRAY_JOB_ID:-$SLURM_JOB_ID}"
export SLURM_ARRAY_TASK_ID="${SLURM_ARRAY_TASK_ID:-1}"
TASK="${SLURM_ARRAY_TASK_ID}"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"

# The gold arm writes one LP row per column of the enumerated universe, so n and max_stops
# are what keep the reference computable at all.
case "$TASK" in
  1) DC_N=10 DC_P=8 DC_S=1 DC_SEED=42 DC_MAX_STOPS=4 ;;
  2) DC_N=10 DC_P=8 DC_S=1 DC_SEED=43 DC_MAX_STOPS=4 ;;
  3) DC_N=10 DC_P=8 DC_S=3 DC_SEED=42 DC_MAX_STOPS=4 ;;
  4) DC_N=8  DC_P=6 DC_S=1 DC_SEED=42 DC_MAX_STOPS=3 ;;
  *) echo "no task $TASK" >&2; exit 1 ;;
esac
export DC_N DC_P DC_S DC_SEED DC_MAX_STOPS
export DC_ANCHORS="${DC_ANCHORS:-8}"
echo "task $TASK: n=$DC_N p=$DC_P s=$DC_S seed=$DC_SEED max_stops=$DC_MAX_STOPS"

stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/benders_detour_credit_potential.jl
