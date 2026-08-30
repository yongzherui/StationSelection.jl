#!/bin/bash
# Study 5 -- single-threaded exact-CG scaling, one job per instance.
# kind of scaling sweep (see ../../scripts/sbatch_zhuzhou_instance.sh) -- 16G/4h per
# task, matched to the memory-censoring protocol in
# notes/2026-08-05_free_assignment_cg_direct_ms5_comparison.md so OOM outcomes stay
# comparable to that prior sweep.
#
# Usage: sbatch --array=1-<n_jobs> submit_benchmark.sh <substudy>
#   substudy = stations, passengers, or scenarios
#   <n_jobs> = number of data rows in config/jobs.tsv (i.e. lines - 1 for the header).
#   -o/-e need slurm_logs/ to exist before first submit: mkdir -p slurm_logs
#
#SBATCH --job-name=study5_scaling_exact_cg
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
# Overridden per submission: the parallel arm needs --cpus-per-task=<n_scenarios>.
#SBATCH --cpus-per-task=1
# 24G for BOTH arms, deliberately. The parallel arm holds n_scenarios label-search
# frontiers live at once where serial holds one, and the archived serial run already peaked
# at 6.1-10.8G of 16G at s=12. Giving the arms *different* memory would confound the
# measurement: Julia sizes its GC heap against the cgroup limit, so GC time -- which lands
# in the wall clock this study compares -- would differ for reasons unrelated to threading.
#SBATCH --mem=24G
#SBATCH --time=06:30:00
#SBATCH --output=slurm_logs/%x-%A_%a.out
#SBATCH --error=slurm_logs/%x-%A_%a.err

set -euo pipefail
# Match Julia's threads to whatever the allocation actually granted. The serial arm is
# submitted with 1 CPU and the parallel arm with n_scenarios, so this is 1 and n_scenarios
# respectively -- and it can never silently disagree with the allocation. run_benchmark.jl
# additionally hard-fails if a parallel row gets fewer threads than its table asked for.
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"

STUDY_DIR="${SLURM_SUBMIT_DIR:?SLURM_SUBMIT_DIR not set -- submit from the Study 5 directory}"
PROJECT_ROOT="$(cd "$STUDY_DIR/../.." && pwd)"
TASK="${SLURM_ARRAY_TASK_ID:?SLURM_ARRAY_TASK_ID not set -- submit via sbatch --array=1-<n_jobs>}"

source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"

SUBSTUDY="${1:?usage: submit_benchmark.sh <stations|passengers|scenarios>}"
case "$SUBSTUDY" in
    stations|passengers|scenarios) ;;
    *) echo "invalid Study 5 sub-study: $SUBSTUDY" >&2; exit 2 ;;
esac
JOBS_FILE="$STUDY_DIR/config/${SUBSTUDY}_jobs.tsv"
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")   # row 0 is the header; task N -> data row N+1

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$STUDY_DIR/run_benchmark.jl" "$JOB_LINE"
