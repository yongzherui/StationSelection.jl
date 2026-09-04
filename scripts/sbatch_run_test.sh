#!/bin/bash
#SBATCH --job-name=run_test
#SBATCH --partition=mit_preemptable
# mit_preemptable, not mit_normal: these are short jobs and mit_normal queues
# behind whatever else this account is running (a full suite was estimated 29 h
# out during a large array). Preemptable nodes start in minutes. The tradeoff is
# that a job can be killed mid-run -- for a ~90 s test suite that is a cheap
# resubmit, and a preempted job shows as CANCELLED/PREEMPTED rather than a real
# test failure, so check the state before reading the absence of output as a bug.
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=01:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/run-test-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/run-test-%j.err

# Generic SLURM launcher for scripts/run_test.jl -- one focused test/ file (or
# several) run in isolation, skipping the full test/runtests.jl suite.
#
# Usage:
#   sbatch scripts/sbatch_run_test.sh <path-relative-to-test/> [more paths...]
#   sbatch scripts/sbatch_run_test.sh opt/test_passenger_mcf_relaxation.jl

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"

if [ "$#" -lt 1 ]; then
    echo "ERROR: Usage: sbatch_run_test.sh <path-relative-to-test/> [more paths...]"
    exit 1
fi

# Absolute path: under sbatch, BASH_SOURCE points at SLURM's copy of this script in
# /var/spool/slurmd/, where scripts/lib/ does not exist.
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"

cd "$PROJECT_ROOT"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/run_test.jl" "$@"
