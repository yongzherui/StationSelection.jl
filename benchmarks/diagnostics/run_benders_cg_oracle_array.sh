#!/bin/bash
# One (arm, n) CELL per array task. Submit: sbatch --array=1-5 run_benders_cg_oracle_array.sh
#
#   task 1-3 -> parity    n = 10, 15, 20   (both oracles + DirectMIPSolver + CGSolver)
#   task 4-5 -> baseline_ms n = 10, 15       (CG oracle at max_stops=10 + CGSolver)
#
# WHY THE CELL AND NOT THE SOLVE. There are really 16 independent solves here (3 parity
# sizes x 4 solves + 2 baseline_ms sizes x 2), but Julia startup with a per-task depot is
# ~350 s while the solves are 2-130 s. Splitting a cell into 4 tasks turns 350+300 into
# 350+130 -- roughly three minutes of wall for four times the compute, which is a bad trade
# on a shared cluster. Below ~5 min of work per task you are paying startup to save less
# than startup.
#
# The second reason is that the parity COMPARISON is the deliverable: keeping the four
# solves in one task lets it assert cg == enum == direct == cg_lp and fail loudly. Split per
# solve and that check has to move into a separate aggregation pass over per-task rows.
#
# Long pole is task 3 (parity n=20: DirectMIP alone was 127 s). Split that one further if it
# becomes the binding constraint.
#SBATCH --job-name=benders_ora
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=64G
#SBATCH --time=03:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-ora-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-ora-%A_%a.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
TASK="${SLURM_ARRAY_TASK_ID:?submit via sbatch --array}"
case "$TASK" in
    1) export OR_NS=10 OR_FULL_NS="" ;;
    2) export OR_NS=15 OR_FULL_NS="" ;;
    3) export OR_NS=20 OR_FULL_NS="" ;;
    4) export OR_NS=""  OR_FULL_NS=10 ;;
    5) export OR_NS=""  OR_FULL_NS=15 ;;
    *) echo "task $TASK out of range 1-5" >&2; exit 2 ;;
esac
# Per-task depot: concurrent tasks sharing ~/.julia raced on the precompile cache and killed
# a task with SIGBUS in gc_mark_outrefs (array 22421655). See
# run_benders_cut_validity_array.sh for the full mechanism.
export CS_COPY_DEPOT="${CS_COPY_DEPOT:-1}"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"
echo "=== task $TASK -> OR_NS='$OR_NS' OR_FULL_NS='$OR_FULL_NS' ==="
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/benders_cg_oracle.jl
