#!/bin/bash
# One n per array task. Submit: sbatch --array=1-2 run_benders_lpo.sh
#   task 1 -> n=10, task 2 -> n=15, task 3 -> n=20 (see the case below).
#
# n=20 is included but expected to be dominated by the plain activated arm, which at n=20
# reached iteration 588 with 1,764 cuts and a 25% gap still open. LP_TOTAL_LIMIT caps that
# per oracle so the task reports a non-converged row instead of hitting the wall.
#SBATCH --job-name=benders_lpo
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=64G
#SBATCH --time=03:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-lpo-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-lpo-%A_%a.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
TASK="${SLURM_ARRAY_TASK_ID:?submit via sbatch --array}"
case "$TASK" in
    1) export LP_N=10 ;;
    2) export LP_N=15 ;;
    3) export LP_N=20 ;;
    *) echo "task $TASK out of range 1-3" >&2; exit 2 ;;
esac
export LP_S="${LP_S:-3}"
export LP_MAX_STOPS="${LP_MAX_STOPS:-4}"
export LP_TOTAL_LIMIT="${LP_TOTAL_LIMIT:-900.0}"
export CS_COPY_DEPOT="${CS_COPY_DEPOT:-1}"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"
echo "=== task $TASK -> n=$LP_N s=$LP_S max_stops=$LP_MAX_STOPS ==="
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/benders_lpo_compare.jl
