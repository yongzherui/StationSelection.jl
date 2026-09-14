#!/bin/bash
# Cross-oracle agreement smoke check, one INSTANCE per array task. See
# benders_oracle_smoke.jl: every task solves its instance with all five arms and fails if
# their objectives disagree, which is the cheap validity test for the Pareto cuts.
#
# Usage: sbatch --array=1-4 benchmarks/diagnostics/run_benders_oracle_smoke.sh
#SBATCH --job-name=benders_sm
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=3
#SBATCH --mem=12G
#SBATCH --time=01:00:00
#SBATCH --array=1-4
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-sm-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-sm-%A_%a.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
export SLURM_ARRAY_JOB_ID="${SLURM_ARRAY_JOB_ID:-$SLURM_JOB_ID}"
export SLURM_ARRAY_TASK_ID="${SLURM_ARRAY_TASK_ID:-1}"
TASK="${SLURM_ARRAY_TASK_ID}"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"

# (N, S) per task. Small on purpose: this is the gate that runs after every edit, not the
# experiment. Task 4 is the multi-scenario shape, where the subproblems run in parallel
# threads and each builds its own auxiliary LP -- the arrangement most likely to expose a
# shared-state mistake in the completion.
case "$TASK" in
  1) SM_N=8  SM_S=1 ;;
  2) SM_N=10 SM_S=1 ;;
  3) SM_N=12 SM_S=1 ;;
  4) SM_N=10 SM_S=3 ;;
  *) echo "no task $TASK" >&2; exit 1 ;;
esac
export SM_N SM_S
echo "task $TASK: n=$SM_N s=$SM_S"

stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/benders_oracle_smoke.jl
