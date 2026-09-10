#!/bin/bash
# ONE (instance, oracle) per array task, each on its OWN exclusive node.
#
# The arms used to run sequentially inside one task per `n`. Three problems with that:
# a non-converging arm burned the wall the later arms needed; one expected non-convergence
# made the task exit non-zero, hiding real failures; and array tasks shared nodes (tasks 1
# and 2 of job 22435620 both landed on node4309), so the wall times the whole comparison
# rests on were contaminated by co-tenancy.
#
# Grid = LPO_NS x LPO_ARMS, laid out row-major over the array index:
#   task = n_index * |ARMS| + arm_index + 1
# Defaults: 3 n-values x 4 arms = 12 tasks.  Submit:
#   sbatch --array=1-12 benchmarks/diagnostics/run_benders_lpo.sh
# Then aggregate (the cross-arm objective gate lives there, since no single job can see
# every arm):
#   julia --project=. benchmarks/diagnostics/benders_lpo_report.jl
#SBATCH --job-name=benders_lpo
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=64G
# Exclusive is an experimental CONTROL, not a performance grab: the arms are compared on
# wall and pricing time, and a neighbour competing for memory bandwidth and L3 shows up as
# arm-to-arm noise.
#SBATCH --exclusive
#SBATCH --time=03:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-lpo-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-lpo-%A_%a.err
set -euo pipefail

PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
TASK="${SLURM_ARRAY_TASK_ID:?submit via sbatch --array}"

# Override either axis from the submit environment, e.g.
#   LPO_NS="25 30" sbatch --array=1-8 ...
read -r -a NS <<< "${LPO_NS:-10 15 20}"
read -r -a ARMS <<< "${LPO_ARMS:-direct_enumeration column_generation column_generation_activated_lpo column_generation_activated_warm_start}"

N_ARMS=${#ARMS[@]}
N_NS=${#NS[@]}
TOTAL=$(( N_NS * N_ARMS ))
if (( TASK < 1 || TASK > TOTAL )); then
    echo "task $TASK out of range 1-$TOTAL (${N_NS} n-values x ${N_ARMS} arms)" >&2
    exit 2
fi
IDX=$(( TASK - 1 ))
export LP_N="${NS[$(( IDX / N_ARMS ))]}"
export LP_ORACLE="${ARMS[$(( IDX % N_ARMS ))]}"

echo "===== task $TASK/$TOTAL -> n=$LP_N oracle=$LP_ORACLE on $(hostname) (exclusive) ====="

export LP_S="${LP_S:-3}"
export LP_P="${LP_P:-8}"
export LP_SEED="${LP_SEED:-42}"
export LP_MAX_STOPS="${LP_MAX_STOPS:-4}"
export LP_THREADS="${LP_THREADS:-1}"
export LP_TOTAL_LIMIT="${LP_TOTAL_LIMIT:-1800.0}"
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"

# Per-task depot: concurrent tasks that share ~/.julia can rewrite a .ji while a sibling has
# it mmapped, which kills the sibling with SIGBUS in gc_mark_outrefs. Cost is one precompile
# per task; the alternative is losing tasks at random. Must be exported BEFORE
# slurm_array_task_env.sh, which is what reads it and stages the depot copy.
export CS_COPY_DEPOT="${CS_COPY_DEPOT:-1}"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"

cd "$PROJECT_ROOT"
julia --startup-file=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/benders_lpo_arm.jl
