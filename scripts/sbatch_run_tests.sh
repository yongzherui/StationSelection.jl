#!/bin/bash
#SBATCH --job-name=station-tests
#SBATCH --partition=mit_normal
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
