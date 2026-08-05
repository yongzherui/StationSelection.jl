#!/usr/bin/env bash
#SBATCH --job-name=free_cg_dir_ms5
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=01:30:00
#SBATCH --array=1-72
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/experiments/free_assignment_cg_direct_ms5/slurm_logs/%x_%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/experiments/free_assignment_cg_direct_ms5/slurm_logs/%x_%A_%a.err

set -euo pipefail
ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
EXP="$ROOT/experiments/free_assignment_cg_direct_ms5"
DATA="$ROOT/../Data/base_data"
line=$(sed -n "$((SLURM_ARRAY_TASK_ID + 1))p" "$EXP/jobs.tsv")
IFS=$'\t' read -r n p seed q method <<< "$line"
module load julia/1.12.6
cd "$ROOT"
julia --project=. scripts/diag_free_assignment_cg_direct_ms5_task.jl \
    "$EXP/results" "$DATA" "$n" "$p" "$seed" "$q" "$method"
