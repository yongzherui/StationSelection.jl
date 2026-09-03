#!/bin/bash
# Study 9 -- does the relaxed-cluster relaxation certify, and at which cluster count?
#
# Usage: sbatch --array=1-30 submit_benchmark.sh
#   -o/-e need slurm_logs/ to exist before the first submit: mkdir -p slurm_logs
#
# Arms are contiguous 5-job ranges in jobs.tsv order (baseline, K=3, 6, 9, 12, 15), so a
# cheap bracketing pass is `--array=1-5,26-30` (baseline + the K=n ceiling control), which
# bounds what the middle arms can possibly achieve before they are run.
#
#SBATCH --job-name=study9_relaxed_cluster_certification
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
# One CPU per scenario (s=3), so every scenario's certification and pricing search gets a
# thread and a round's wall is the max over scenarios rather than their sum; Gurobi stays
# pinned to 1 thread so scenario pricing is the only parallelism. Memory and walltime are
# sized for n=15, not inherited from Studies 7/8's n=20 grid -- the 1800 s total budget
# plus a 300 s recovery MIP fits comfortably in 45 minutes.
#SBATCH --cpus-per-task=3
#SBATCH --mem=8G
#SBATCH --time=00:45:00
#SBATCH --output=slurm_logs/%x-%A_%a.out
#SBATCH --error=slurm_logs/%x-%A_%a.err

set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"

STUDY_DIR="${SLURM_SUBMIT_DIR:?SLURM_SUBMIT_DIR not set -- submit from the Study 9 directory}"
PROJECT_ROOT="$(cd "$STUDY_DIR/../.." && pwd)"
TASK="${SLURM_ARRAY_TASK_ID:?SLURM_ARRAY_TASK_ID not set -- submit via sbatch --array=1-<n_jobs>}"

source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"

JOBS_FILE="$STUDY_DIR/config/jobs.tsv"
JOB_LINE=$(sed -n "$((TASK + 1))p" "$JOBS_FILE")   # row 0 is the header

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$STUDY_DIR/run_benchmark.jl" "$JOB_LINE"
