#!/bin/bash
# ONE (instance, oracle) per array task. Pass --exclusive for timing runs (see below).
#
# The arms used to run sequentially inside one task per `n`. Three problems with that:
# a non-converging arm burned the wall the later arms needed; one expected non-convergence
# made the task exit non-zero, hiding real failures; and array tasks shared nodes (tasks 1
# and 2 of job 22435620 both landed on node4309), so the wall times the whole comparison
# rests on were contaminated by co-tenancy.
#
# Grid = LPO_NS x LPO_SEEDS x LPO_ARMS, laid out row-major over the array index:
#   task = (n_index * |SEEDS| + seed_index) * |ARMS| + arm_index + 1
# Defaults: 3 n x 1 seed x 4 arms = 12 tasks.  Submit:
#   sbatch --array=1-12 benchmarks/diagnostics/run_benders_lpo.sh
#
# Large n needs a different pricer. `:exact` completed ZERO pricing rounds in 17 minutes at
# n=30, so n>=25 wants LP_SUB_MODE=relaxed_cluster with LP_SUB_K around 0.6n (Study 10: K/n
# 0.6-0.8 certifies, 0.4 is 0/5 AND slower), and LP_MAX_STOPS=10 rather than 4. Example:
#   LPO_NS="30 40 50" LPO_SEEDS="42 43 44 45 46" \
#   LPO_ARMS="column_generation column_generation_activated_lpo column_generation_warm_start" \
#   LP_SUB_MODE=relaxed_cluster LP_MAX_STOPS=10 LP_TOTAL_LIMIT=13500 \
#   sbatch --array=1-45 --time=04:00:00 benchmarks/diagnostics/run_benders_lpo.sh
# Then aggregate (the cross-arm objective gate lives there, since no single job can see
# every arm):
#   julia --project=. benchmarks/diagnostics/benders_lpo_report.jl
#SBATCH --job-name=benders_lpo
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=64G
# NOTE: --exclusive is deliberately NOT a directive here. It is an experimental control for
# TIMING comparisons (a neighbour competing for memory bandwidth and L3 shows up as
# arm-to-arm noise), so pass it on the command line when wall and pricing times are the
# measurement:
#   sbatch --exclusive --array=1-30 run_benders_lpo.sh
# Diagnostic runs that only need a log or a trace should NOT ask for it -- a whole-node
# request queues behind every other job on a busy partition, for no benefit.
#SBATCH --time=03:00:00
# mit_preemptable is the right partition (large and quick to schedule) but it DOES preempt:
# 17 of 40 tasks in job 22439457 were killed by PREEMPTION, most inside 20 minutes and
# before a single Benders iteration -- which made preemption, not certification, the main
# cause of missing large-n data. --requeue puts a preempted task back in the queue instead
# of losing it. Each task is self-contained (it rebuilds and rewrites its own TSV row), so a
# restart from scratch is correct, just repeated work.
#SBATCH --requeue
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-lpo-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-lpo-%A_%a.err
set -euo pipefail

PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
TASK="${SLURM_ARRAY_TASK_ID:?submit via sbatch --array}"

# Override either axis from the submit environment, e.g.
#   LPO_NS="25 30" sbatch --array=1-8 ...
read -r -a NS <<< "${LPO_NS:-10 15 20}"
read -r -a SEEDS <<< "${LPO_SEEDS:-42}"
read -r -a ARMS <<< "${LPO_ARMS:-direct_enumeration column_generation column_generation_activated_lpo column_generation_warm_start}"

N_ARMS=${#ARMS[@]}
N_SEEDS=${#SEEDS[@]}
N_NS=${#NS[@]}
TOTAL=$(( N_NS * N_SEEDS * N_ARMS ))
if (( TASK < 1 || TASK > TOTAL )); then
    echo "task $TASK out of range 1-$TOTAL (${N_NS} n x ${N_SEEDS} seeds x ${N_ARMS} arms)" >&2
    exit 2
fi
IDX=$(( TASK - 1 ))
export LP_ORACLE="${ARMS[$(( IDX % N_ARMS ))]}"
CELL=$(( IDX / N_ARMS ))
export LP_SEED="${SEEDS[$(( CELL % N_SEEDS ))]}"
export LP_N="${NS[$(( CELL / N_SEEDS ))]}"

echo "===== task $TASK/$TOTAL -> n=$LP_N seed=$LP_SEED oracle=$LP_ORACLE on $(hostname) ====="

export LP_S="${LP_S:-3}"
export LP_P="${LP_P:-8}"
export LP_MAX_STOPS="${LP_MAX_STOPS:-4}"
# The relaxed-cluster pricer needs a partition count, and it is the one parameter that must
# scale with n. Derived, not fixed, so a grid spanning several n does not silently run one
# ratio everywhere: 0.6n rounded up (Study 10's workable band is K/n 0.6-0.8).
export LP_SUB_MODE="${LP_SUB_MODE:-exact}"
if [[ "$LP_SUB_MODE" == "relaxed_cluster" || "$LP_SUB_MODE" == "relaxed_cluster_two_tier" ]]; then
    export LP_SUB_K="${LP_SUB_K:-$(( (LP_N * 6 + 9) / 10 ))}"
fi
# Two-tier REQUIRES a macro count. Derived from the measured optimum rather than fixed:
# K1=14-16 at n=40 with K2=24, i.e. about 0.62*K2, and K1<=8 is nearly worthless while
# K1>=18 pays real time in the macro sweep.
if [[ "$LP_SUB_MODE" == "relaxed_cluster_two_tier" ]]; then
    export LP_SUB_K1="${LP_SUB_K1:-$(( (LP_SUB_K * 62 + 50) / 100 ))}"
fi
export LP_THREADS="${LP_THREADS:-1}"
export LP_TOTAL_LIMIT="${LP_TOTAL_LIMIT:-1800.0}"
export LP_CG_PRICE_LIMIT="${LP_CG_PRICE_LIMIT:-600.0}"
export LP_PARALLEL_SCENARIOS="${LP_PARALLEL_SCENARIOS:-1}"
export LP_GUIDE_ROUTES="${LP_GUIDE_ROUTES:-5}"
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
