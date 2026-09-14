#!/bin/bash
# What do the real CG iterations do to the dual, and how close is that to the best dual that
# exists? Four-way comparison + per-iteration trajectory + gold binding-row anatomy.
# One instance per array task. See benders_flatness_bound.jl.
#
# Usage: sbatch --array=1-3 benchmarks/diagnostics/run_benders_flatness_bound.sh
#SBATCH --job-name=benders_fl
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=3
#SBATCH --mem=16G
#SBATCH --time=02:00:00
#SBATCH --array=1-3
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-fl-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-fl-%A_%a.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
export SLURM_ARRAY_JOB_ID="${SLURM_ARRAY_JOB_ID:-$SLURM_JOB_ID}"
export SLURM_ARRAY_TASK_ID="${SLURM_ARRAY_TASK_ID:-1}"
TASK="${SLURM_ARRAY_TASK_ID}"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"

# The gold arm writes one LP row per column of the ENUMERATED universe and the value function
# enumerates the master's whole feasible set, so n and max_stops are what keep this a proof.
case "$TASK" in
  1) FL_N=10 FL_P=8 FL_S=1 FL_SEED=42 FL_MAX_STOPS=4 ;;
  2) FL_N=10 FL_P=8 FL_S=1 FL_SEED=43 FL_MAX_STOPS=4 ;;
  3) FL_N=8  FL_P=6 FL_S=1 FL_SEED=42 FL_MAX_STOPS=3 ;;
  *) echo "no task $TASK" >&2; exit 1 ;;
esac
export FL_N FL_P FL_S FL_SEED FL_MAX_STOPS
export FL_ANCHORS="${FL_ANCHORS:-8}"
echo "task $TASK: n=$FL_N p=$FL_P s=$FL_S seed=$FL_SEED max_stops=$FL_MAX_STOPS"

stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/benders_flatness_bound.jl
