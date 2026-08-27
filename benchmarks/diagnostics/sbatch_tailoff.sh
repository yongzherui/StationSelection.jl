#!/bin/bash
#SBATCH --job-name=study6_tailoff
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --mem=16G
#SBATCH --time=00:40:00
#SBATCH --output=slurm_logs/%x-%j.out
#SBATCH --error=slurm_logs/%x-%j.err

set -euo pipefail
export JULIA_NUM_THREADS=1

DIAG_DIR="${SLURM_SUBMIT_DIR:?submit from benchmarks/diagnostics}"
PROJECT_ROOT="$(cd "$DIAG_DIR/../.." && pwd)"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$DIAG_DIR/${DIAG_SCRIPT:-study6_tailoff_repro.jl}" "$@"
