#!/bin/bash
#SBATCH --job-name=proto_domidx
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=6G
#SBATCH --time=00:20:00

set -uo pipefail
module load julia/1.12.6
julia --version
PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
# Needs DataStructures (already a project dep, already precompiled in the shared
# depot -> read-only load, no recompile, safe alongside concurrent jobs). No Gurobi.
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/prototype_dominance_index.jl"
