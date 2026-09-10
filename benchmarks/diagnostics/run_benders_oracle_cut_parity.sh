#!/bin/bash
# One SEED per array task. Submit: sbatch --array=1-10 run_benders_oracle_cut_parity.sh
#   task i -> seed (CP_SEED_BASE + i - 1), default base 42, so 1-10 covers seeds 42..51.
#
# Each task runs BOTH oracles on its own instance (that pairing is the experimental design --
# see benders_oracle_cut_parity.jl), plus a determinism re-solve. Per-task depot: concurrent
# tasks sharing ~/.julia raced on the precompile cache and killed a task with SIGBUS in
# gc_mark_outrefs (array 22421655).
#SBATCH --job-name=benders_cp
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=64G
#SBATCH --time=02:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-cp-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-cp-%A_%a.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
TASK="${SLURM_ARRAY_TASK_ID:?submit via sbatch --array}"
export CP_SEEDS=$(( ${CP_SEED_BASE:-42} + TASK - 1 ))
export CP_N="${CP_N:-20}"
export CS_COPY_DEPOT="${CS_COPY_DEPOT:-1}"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"
echo "=== task $TASK -> CP_N=$CP_N CP_SEEDS=$CP_SEEDS ==="
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/benders_oracle_cut_parity.jl
