#!/bin/bash
# Push the Benders/CG frontier: one SEED per array task at fixed n, CG oracle only.
# Submit: sbatch --array=1-5 run_benders_frontier_n30.sh
#   task i -> seed (FR_SEED_BASE + i - 1), default base 42, so 1-5 covers seeds 42..46.
#
# Enumeration is not attempted: at n=30 its pool is hopeless (237k columns already at n=20),
# which is the whole reason the CG oracle exists. So OR_NS is empty and only the
# baseline_ms arm runs.
#
# OR_REQUIRE_CG_REF=0 on purpose. The measured CGSolver frontier is n<=20 all scenarios,
# n=25 to <=5 scenarios, n=30 only s=1 -- so at n=30/s=3 the CG MASTER is expected not to
# converge, and treating its absence as a failure would mislabel an expected condition as a
# defect. When it is unavailable the run says so explicitly rather than passing quietly.
#
# The interesting outcome is whether Benders' per-scenario subproblem CG converges where the
# full CG master cannot: y is fixed there and each subproblem carries one scenario, so the
# pricing problem is materially smaller. If it does, this beats the CG frontier. If it does
# not, the expected failure is `pricing_inconclusive` -- the convergence gate refusing to
# derive a cut -- which is a loud stop, not a wrong answer.
#SBATCH --job-name=benders_fr
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=64G
#SBATCH --time=04:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-fr-%A_%a.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/benders-fr-%A_%a.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
TASK="${SLURM_ARRAY_TASK_ID:?submit via sbatch --array}"
export OR_NS=""
export OR_FULL_NS="${FR_N:-30}"
export OR_S="${FR_S:-3}"
export OR_SEED=$(( ${FR_SEED_BASE:-42} + TASK - 1 ))
export OR_FULL_MS="${FR_MAX_STOPS:-10}"
export OR_REQUIRE_CG_REF=0
# Subproblem pricer. :relaxed_cluster at n=30 because that is past where the exact search
# still exhausts (measured CG frontier: n<=20 all scenarios, n=25 to <=5, n=30 only s=1).
# It exhausts by CERTIFYING a relaxation that lower-bounds every real route's reduced cost,
# which licenses a Benders cut exactly as an exhaustive search does. K defaults to 60% of n,
# the ratio Study 10 found workable (K/n 0.6-0.8 certified; 0.4 was 0/5 AND slower).
export OR_SUB_MODE="${FR_SUB_MODE:-relaxed_cluster}"
export OR_SUB_K="${FR_SUB_K:-$(( (${FR_N:-30} * 6 + 9) / 10 ))}"
export CS_COPY_DEPOT="${CS_COPY_DEPOT:-1}"
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
source "$PROJECT_ROOT/scripts/lib/slurm_array_task_env.sh"
cd "$PROJECT_ROOT"
echo "=== task $TASK -> n=$OR_FULL_NS s=$OR_S seed=$OR_SEED max_stops=$OR_FULL_MS ==="
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/benders_cg_oracle.jl
