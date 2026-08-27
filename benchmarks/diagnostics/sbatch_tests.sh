#!/bin/bash
#SBATCH --job-name=ss_tests
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=01:30:00
#SBATCH --output=slurm_logs/%x-%j.out
#SBATCH --error=slurm_logs/%x-%j.err
set -euo pipefail
DIAG_DIR="${SLURM_SUBMIT_DIR:?}"
PROJECT_ROOT="$(cd "$DIAG_DIR/../.." && pwd)"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" -e 'using Pkg; Pkg.test()'
