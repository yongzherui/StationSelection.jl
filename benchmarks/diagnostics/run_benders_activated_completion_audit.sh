#!/bin/bash
# Is the activated oracle's dual completion dual-FEASIBLE, how much cut strength does it
# cost, and does the locally Pareto-optimal completion buy that strength back? One INSTANCE
# per array task -- every task enumerates its own value function and runs all four arms
# against it, so the arms are always compared on the same anchors and the same Q.
#
# Usage: sbatch --array=1-6 benchmarks/diagnostics/run_benders_activated_completion_audit.sh
#SBATCH --job-name=benders_aca
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=3
#SBATCH --mem=16G
#SBATCH --time=04:00:00
#SBATCH --array=1-6
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-aca-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-aca-%A_%a.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
export SLURM_ARRAY_JOB_ID="${SLURM_ARRAY_JOB_ID:-$SLURM_JOB_ID}"
export SLURM_ARRAY_TASK_ID="${SLURM_ARRAY_TASK_ID:-1}"
TASK="${SLURM_ARRAY_TASK_ID}"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"

# (seed, S) per task. n stays at 10: the whole script rests on enumerating the master's
# feasible set and the complete column universe, and both are exponential -- 10/k=5 is 86
# station sets and ~16k columns, which is the size at which "exactly right" is affordable.
# Seeds vary DEMAND only on this instance family (the stations are the deterministic top-N by
# popularity), so this grid varies the demand pattern and the scenario count, not geography.
case "$TASK" in
  1) ACA_SEED=42 ACA_S=1 ;;
  2) ACA_SEED=43 ACA_S=1 ;;
  3) ACA_SEED=44 ACA_S=1 ;;
  4) ACA_SEED=42 ACA_S=3 ;;
  5) ACA_SEED=43 ACA_S=3 ;;
  6) ACA_SEED=44 ACA_S=3 ;;
  *) echo "no task $TASK" >&2; exit 1 ;;
esac
export ACA_SEED ACA_S
export ACA_N="${ACA_N:-10}"
export ACA_P="${ACA_P:-8}"
export ACA_MAX_STOPS="${ACA_MAX_STOPS:-4}"
export ACA_ANCHORS="${ACA_ANCHORS:-8}"
export ACA_ARMS="${ACA_ARMS:-column_generation_activated,lpo_baseline,lpo_pareto,column_generation}"
export ACA_CORE_POINT="${ACA_CORE_POINT:-relative_interior}"
echo "task $TASK: n=$ACA_N p=$ACA_P s=$ACA_S seed=$ACA_SEED arms=$ACA_ARMS"

stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/benders_activated_completion_audit.jl
