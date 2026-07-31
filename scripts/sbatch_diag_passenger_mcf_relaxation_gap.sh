#!/bin/bash
#SBATCH --job-name=diag_pfa_mcf
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=02:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/diag-pfa-mcf-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/diag-pfa-mcf-%A_%a.err

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTDIR="${OUTDIR:-$PROJECT_ROOT/tmp_ss_bench/mcf_gap}"

module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6

# One station count per array task, so the sweep runs concurrently.
N_STATIONS_LIST=(10 15 20)
N_STATIONS="${N_STATIONS_LIST[${SLURM_ARRAY_TASK_ID:-0}]}"

mkdir -p "$OUTDIR"
cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/diag_passenger_mcf_relaxation_gap.jl" "$N_STATIONS" "$OUTDIR"
