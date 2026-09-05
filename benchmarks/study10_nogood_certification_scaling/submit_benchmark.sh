#!/bin/bash
# Study 10 -- does no-good-cut relaxed-cluster certification still fire at n=20 and n=30?
#
# Usage (STUDY10_RUN_DATE pins every task to one run directory -- see the block below,
# and note it must be set in the SUBMITTING shell, not inside this script):
#   mkdir -p slurm_logs                       # -o/-e need it before the first submit
#   export STUDY10_RUN_DATE=$(date +%F)
#   STUDY10_RUN_DATE=$STUDY10_RUN_DATE sbatch --array=21-40 submit_benchmark.sh   # n=30, 6 h
#   STUDY10_RUN_DATE=$STUDY10_RUN_DATE sbatch --array=1-20 --time=04:45:00 --mem=24G \
#       submit_benchmark.sh                                                        # n=20, 4 h
#
# Re-running a few failed tasks later: pass the ORIGINAL date so the rows land beside the
# rest of the study rather than in a fresh directory.
#   STUDY10_RUN_DATE=2026-09-04 sbatch --array=7,19 submit_benchmark.sh
#
# The SBATCH directives below are sized for the HEAVIER n=30 half, because a job that
# outruns its walltime writes no row at all while one that finishes early costs only queue
# priority. Override --time/--mem on the command line for the n=20 half, as above.
#
# jobs.tsv is ordered size-slowest, then arm, then seed, so:
#   1-20  n=20   (1-5 baseline, 6-10 K=8,  11-15 K=12, 16-20 K=16)
#   21-40 n=30   (21-25 baseline, 26-30 K=12, 31-35 K=18, 36-40 K=24)
# Any single arm is a contiguous 5-job range and can be submitted or re-run alone.
#
#SBATCH --job-name=study10_nogood_certification_scaling
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
# One CPU per scenario (s=3) so `parallel_scenario_pricing` makes a round's wall the max
# over scenarios rather than their sum; Gurobi stays pinned to 1 thread so scenario pricing
# is the only parallelism. 32G rather than Studies 7/8's 24G: n=30's label populations sit
# between n=20 (comfortable at 24G) and n=40 (measured to OOM at 24G).
#SBATCH --cpus-per-task=3
#SBATCH --mem=32G
# 6 h total CG budget + a 300 s recovery MIP + package load and instance build.
#SBATCH --time=06:45:00
#SBATCH --output=slurm_logs/%x-%A_%a.out
#SBATCH --error=slurm_logs/%x-%A_%a.err

set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"

STUDY_DIR="${SLURM_SUBMIT_DIR:?SLURM_SUBMIT_DIR not set -- submit from the Study 10 directory}"
PROJECT_ROOT="$(cd "$STUDY_DIR/../.." && pwd)"
TASK="${SLURM_ARRAY_TASK_ID:?SLURM_ARRAY_TASK_ID not set -- submit via sbatch --array=1-40}"

# Pin every task of the array to ONE run directory.
#
# `benchmark_output_dir` defaults to `<Dates.today()>_<slug>` evaluated when the task runs.
# With a 4-7 h walltime on a preemptable queue, tasks that start after midnight would write
# into a second directory, and `analyze.jl` -- which picks the newest match -- would then
# silently analyze half the study.
#
# This block CANNOT fix that on its own: it runs on the compute node at task start, so
# anything it computes from `date` is already per-task. The date has to come from the
# submitting shell and ride in on sbatch's default `--export=ALL`, i.e. the usage lines at
# the top of this file. The per-task fallback below is only so a bare `sbatch` still
# produces a working (if possibly split) run rather than failing; it warns when it fires.
if [ -z "${STUDY10_RUN_DATE:-}" ]; then
    echo "WARNING: STUDY10_RUN_DATE unset -- this task stamps its own date, so an array" >&2
    echo "         spanning midnight will split across two run directories. Submit as:" >&2
    echo "         STUDY10_RUN_DATE=\$(date +%F) sbatch --array=1-40 submit_benchmark.sh" >&2
    STUDY10_RUN_DATE="$(date +%F)"
fi
export STUDY10_OUTPUT_DIR="${STUDY10_OUTPUT_DIR:-$PROJECT_ROOT/benchmarks/experiments/${STUDY10_RUN_DATE}_study10_nogood_certification_scaling}"
mkdir -p "$STUDY10_OUTPUT_DIR"

source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"

JOBS_FILE="$STUDY_DIR/config/jobs.tsv"
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")   # row 0 is the header

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$STUDY_DIR/run_benchmark.jl" "$JOB_LINE"
