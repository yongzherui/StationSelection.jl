#!/bin/bash
# One table row per node/process. Submit with a pinned STUDY9_RUN_ID and matching array.
# Example: STUDY9_RUN_ID=2026-09-08_rc_scale sbatch --array=1-30 submit_benchmark.sh validation.tsv
#SBATCH --job-name=study9_relaxed_cluster_scale
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=3
# MEASURED across 107 tasks (n=20 through n=50, every arm): peak RSS 13.1G, median 8-10G,
# and essentially FLAT in n -- n=30 peaked at 12.4G against n=50's 13.1G, because the
# footprint is dominated by fixed overhead (Julia + Gurobi + the loaded package) rather than
# by the instance or the column pool. 16G covers every observed run with ~20% headroom.
#
# `--mem` is a RESERVATION, so asking 5x what you use is worth fixing on shared hardware --
# but do not expect it to shorten your queue wait. MEASURED 2026-09-08: pending jobs here
# report reason `(Priority)`, not `(Resources)`, on a partition with 53k CPUs and ~2000
# running jobs, so the wait is fair-share (`sshare -U` gave FairShare 0.024 after a heavy
# day), not a resource shortage. Right-size memory for hygiene, not for throughput.
#
# The one live exception is a large-n `exact` arm: the 2026-08-01 full-CG grid OOM'd at n=40
# with 24G on that pricer. Study 9's exact arms stop at n=25 (peak 12.1G), so pass an
# explicit larger `--mem` if that pricer is ever run big again.
#SBATCH --mem=16G
#SBATCH --time=06:30:00
#SBATCH --output=slurm_logs/%x-%A_%a.out
#SBATCH --error=slurm_logs/%x-%A_%a.err

set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
STUDY_DIR="${SLURM_SUBMIT_DIR:?submit from the Study 9 directory}"
PROJECT_ROOT="$(cd "$STUDY_DIR/../.." && pwd)"
TASK="${SLURM_ARRAY_TASK_ID:?submit via sbatch --array}"
TABLE="${1:?usage: submit_benchmark.sh <validation.tsv|nNN.tsv>}"
case "$TABLE" in
    smoke.tsv|smoke_twotier.tsv|n30_twotier.tsv|n40_twotier.tsv|n40_anytime.tsv|n40_reach.tsv|n40_reach2.tsv|n40_reach3.tsv|n40_reach4.tsv|n40_reach3b.tsv|n30_twotier_m14.tsv|n50_twotier.tsv|validation.tsv|n20.tsv|n25.tsv|n30.tsv|n35.tsv|n40.tsv|n45.tsv|n50.tsv|n55.tsv|n60.tsv|n65.tsv|n70.tsv|n75.tsv|n80.tsv|n84.tsv) ;;
    *) echo "invalid job table: $TABLE" >&2; exit 2 ;;
esac
RUN_ID="${STUDY9_RUN_ID:?set STUDY9_RUN_ID in the submitting shell}"
export STUDY9_OUTPUT_DIR="${STUDY9_OUTPUT_DIR:-$PROJECT_ROOT/benchmarks/experiments/${RUN_ID}_study9_relaxed_cluster_scalability}"
mkdir -p "$STUDY9_OUTPUT_DIR"

source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"

JOBS_FILE="$STUDY_DIR/config/$TABLE"
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")
[ -n "$JOB_LINE" ] || { echo "no task $TASK in $JOBS_FILE" >&2; exit 2; }
cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" "$STUDY_DIR/run_benchmark.jl" "$JOB_LINE"
