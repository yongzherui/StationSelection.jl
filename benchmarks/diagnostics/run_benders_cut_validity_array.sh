#!/bin/bash
# One SEED per array task. The seeds in benders_cut_validity_seeds.jl are fully independent
# -- each runs its own Benders + DirectMIPSolver + CG + cut audit with no shared state -- so
# running them sequentially in one job wastes ~10x the wall clock and makes the whole batch
# all-or-nothing against preemption on mit_preemptable. Measured: 3.5 min/seed sequential
# (~37 min for 10) versus ~2 min startup + ~1.5 min work in parallel.
#
# Submit: sbatch --array=1-10 run_benders_cut_validity_array.sh
#   task i -> seed (SEED_BASE + i - 1), default base 42, so 1-10 covers seeds 42..51.
# Env: CV_N (default 20), CV_SEED_BASE, plus the script's own CV_* knobs.
#SBATCH --job-name=benders_cva
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=64G
#SBATCH --time=01:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-cva-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-cva-%A_%a.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
TASK="${SLURM_ARRAY_TASK_ID:?submit via sbatch --array}"
SEED_BASE="${CV_SEED_BASE:-42}"
export CV_SEEDS=$((SEED_BASE + TASK - 1))
export CV_N="${CV_N:-20}"
# PER-TASK DEPOT BY DEFAULT. Sharing one depot across concurrent tasks killed task 5 of array
# 22421655 with `Bus error (signal 7)` in gc_mark_outrefs 47 s in (MaxRSS 2 GB of 64 GB, so
# not OOM): four tasks entered precompilation at once and three rewrote StationSelection.ji,
# which invalidates the mmap of any sibling holding the old file, and the next GC touch faults.
#
# Julia's pidfile lock does NOT prevent this. It serialises WRITERS; it does nothing for a
# READER whose mapping is replaced underneath it. Nor can the script's per-seed try/catch
# contain it -- SIGBUS kills the process rather than raising a Julia exception.
#
# Paying ~6 min of precompile per task is the correct trade. If you want the fast path, warm
# the shared depot with ONE job first (any script that loads StationSelection), confirm it
# wrote its .ji, and only then submit with CS_COPY_DEPOT=0 -- the race needs a stale cache to
# exist at all. Editing src/ right before submitting is what makes the cache stale.
export CS_COPY_DEPOT="${CS_COPY_DEPOT:-1}"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"
echo "=== array task $TASK -> CV_N=$CV_N CV_SEEDS=$CV_SEEDS ==="
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/benders_cut_validity_seeds.jl
