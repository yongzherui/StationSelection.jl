#!/bin/bash
# How much cut strength does the conservative `rho <= 0` completion give up against a true
# Magnanti-Wong dual over the enumerated full route family? One tiny INSTANCE per array task.
# TESTING ONLY -- the gold arm enumerates R, which production must never do.
#
# Usage: sbatch --array=1-4 benchmarks/diagnostics/run_benders_lpo_gold_standard.sh
#SBATCH --job-name=benders_gs
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=3
#SBATCH --mem=12G
#SBATCH --time=02:00:00
#SBATCH --array=1-4
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-gs-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-gs-%A_%a.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
export SLURM_ARRAY_JOB_ID="${SLURM_ARRAY_JOB_ID:-$SLURM_JOB_ID}"
export SLURM_ARRAY_TASK_ID="${SLURM_ARRAY_TASK_ID:-1}"
TASK="${SLURM_ARRAY_TASK_ID}"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"

# Deliberately tiny: the gold arm writes one LP row per column of the ENUMERATED universe, so
# `max_stops` and `n` are what keep it writable at all.
case "$TASK" in
  1) GS_N=8  GS_P=6 GS_SEED=42 GS_MAX_STOPS=3 ;;
  2) GS_N=8  GS_P=6 GS_SEED=43 GS_MAX_STOPS=3 ;;
  3) GS_N=10 GS_P=8 GS_SEED=42 GS_MAX_STOPS=3 ;;
  4) GS_N=10 GS_P=8 GS_SEED=42 GS_MAX_STOPS=4 ;;
  *) echo "no task $TASK" >&2; exit 1 ;;
esac
export GS_N GS_P GS_SEED GS_MAX_STOPS
export GS_S="${GS_S:-1}"
export GS_ANCHORS="${GS_ANCHORS:-6}"
echo "task $TASK: n=$GS_N p=$GS_P s=$GS_S seed=$GS_SEED max_stops=$GS_MAX_STOPS"

stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/benders_lpo_gold_standard.jl
