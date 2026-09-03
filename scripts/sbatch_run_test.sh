#!/bin/bash
#SBATCH --job-name=run_test
#SBATCH --partition=mit_normal
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
