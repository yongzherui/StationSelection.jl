#!/bin/bash
#SBATCH --job-name=bench_pfa_labels
#SBATCH --partition=mit_preemptable
# mit_preemptable, not mit_normal: mit_normal queues behind whatever else this
# account is running (a short job was estimated 29 h out during a large array),
# while preemptable nodes start in minutes and preemption is rare in practice.
# A preempted job shows as CANCELLED/PREEMPTED with truncated output -- check the
# sacct state before reading missing results as a failure.
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=01:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/bench-pfa-labels-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/bench-pfa-labels-%j.err

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"

source "$(dirname "${BASH_SOURCE[0]}")/lib/slurm_modules.sh"

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/bench_passenger_free_assignment_labels.jl" "$@"
