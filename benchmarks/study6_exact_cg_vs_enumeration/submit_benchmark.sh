#!/bin/bash
# Study 6 -- Base exact CG versus exhaustive enumeration, one method per task.
#SBATCH --job-name=study6_cg_vs_enum
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=16G
#SBATCH --time=02:00:00
#SBATCH --output=slurm_logs/%x-%A_%a.out
#SBATCH --error=slurm_logs/%x-%A_%a.err

set -euo pipefail
export JULIA_NUM_THREADS=1

STUDY_DIR="${SLURM_SUBMIT_DIR:?SLURM_SUBMIT_DIR not set -- submit from the Study 6 directory}"
PROJECT_ROOT="$(cd "$STUDY_DIR/../.." && pwd)"
TASK="${SLURM_ARRAY_TASK_ID:?SLURM_ARRAY_TASK_ID not set -- submit via sbatch --array=1-60}"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"

JOB_LINE=$(sed -n "$((TASK + 1))p" "$STUDY_DIR/config/jobs.tsv")
cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" "$STUDY_DIR/run_benchmark.jl" "$JOB_LINE"
