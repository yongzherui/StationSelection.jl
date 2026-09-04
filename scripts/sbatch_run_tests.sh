#!/bin/bash
#SBATCH --job-name=station-tests
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
#SBATCH --mem=16G
#SBATCH --time=04:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/station-tests-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/station-tests-%j.err

set -euo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"

# Absolute path: under sbatch, BASH_SOURCE points at SLURM's copy of this script in
# /var/spool/slurmd/, where scripts/lib/ does not exist.
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" -e 'using Pkg; Pkg.test()'
