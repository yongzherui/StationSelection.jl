#!/bin/bash
# One table row per node/process. Submit with a pinned STUDY11_RUN_ID and matching array.
# Example: STUDY11_RUN_ID=2026-09-09_n30 sbatch --array=1-10 submit_benchmark.sh n30.tsv
#SBATCH --job-name=study11_master_lp_integrality
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=3
# Study 9 measured peak RSS 13.1G across 107 tasks (n=20 through n=50, every arm), median
# 8-10G and essentially FLAT in n -- the footprint is dominated by fixed overhead (Julia +
# Gurobi + the loaded package) rather than by the instance or the column pool. The
# per-iteration MIP snapshot adds a second model, but it is the same pool-sized rebuild
# `recover_integer_solution` already does once per run and it is dropped immediately, so it
# does not move the peak. 16G matches Studies 9 and 10.
#SBATCH --mem=16G
# 14400 s CG budget + the 300 s final recovery MIP + the per-iteration MIP snapshots +
# start-up. Study 9's n=30 relaxed_k60 runs certified in 155-5195 s of CG, and the
# snapshots add at most `iterations x ip_time_limit_sec` (25 x 120 s ~ 50 min) on top, so
# 5 h is generous headroom rather than a sized reservation -- the budget is the real cap.
#SBATCH --time=05:00:00
#SBATCH --output=slurm_logs/%x-%A_%a.out
#SBATCH --error=slurm_logs/%x-%A_%a.err

set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
STUDY_DIR="${SLURM_SUBMIT_DIR:?submit from the Study 11 directory}"
PROJECT_ROOT="$(cd "$STUDY_DIR/../.." && pwd)"
TASK="${SLURM_ARRAY_TASK_ID:?submit via sbatch --array}"
TABLE="${1:?usage: submit_benchmark.sh <smoke.tsv|n30.tsv|n30_exact_control.tsv|n30_maxstops.tsv>}"
case "$TABLE" in
    smoke.tsv|n30.tsv|n30_exact_control.tsv|n30_maxstops.tsv) ;;
    *) echo "invalid job table: $TABLE" >&2; exit 2 ;;
esac
RUN_ID="${STUDY11_RUN_ID:?set STUDY11_RUN_ID in the submitting shell}"
export STUDY11_OUTPUT_DIR="${STUDY11_OUTPUT_DIR:-$PROJECT_ROOT/benchmarks/experiments/${RUN_ID}_study11_master_lp_integrality}"
mkdir -p "$STUDY11_OUTPUT_DIR"

source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"

JOBS_FILE="$STUDY_DIR/config/$TABLE"
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")
[ -n "$JOB_LINE" ] || { echo "no task $TASK in $JOBS_FILE" >&2; exit 2; }
cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" "$STUDY_DIR/run_benchmark.jl" "$JOB_LINE"
