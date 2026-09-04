#!/bin/bash
#SBATCH --job-name=diagnose
#SBATCH --partition=mit_preemptable
# mit_preemptable, not mit_normal: mit_normal queues behind whatever else this
# account is running (a short job was estimated 29 h out during a large array),
# while preemptable nodes start in minutes and preemption is rare in practice.
# A preempted job shows as CANCELLED/PREEMPTED with truncated output -- check the
# sacct state before reading missing results as a failure.
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=02:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/diagnose-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/diagnose-%j.err

# Generic SLURM launcher for any scripts/diagnose.jl mode. One script replaces
# the per-diagnostic sbatch_diag_*.sh files this repo used to accumulate.
#
# Usage:
#   sbatch scripts/sbatch_diagnose.sh <mode> [mode args...]
#   sbatch --time=12:00:00 --mem=32G scripts/sbatch_diagnose.sh station_cluster cg 60 42 cluster out.txt
#
# Mode-specific env vars (see scripts/modes/<mode>.jl docstrings) can be set on
# the sbatch command line, e.g.:
#   PFACG_MAX_STOPS=6 sbatch scripts/sbatch_diagnose.sh cg_iteration_profile 30 out.csv

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"

if [ "$#" -lt 1 ]; then
    echo "ERROR: Usage: sbatch_diagnose.sh <mode> [mode args...]"
    exit 1
fi

source "$(dirname "${BASH_SOURCE[0]}")/lib/slurm_modules.sh"

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/diagnose.jl" "$@"
