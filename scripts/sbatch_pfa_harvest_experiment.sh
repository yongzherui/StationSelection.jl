#!/bin/bash
# Does harvesting MORE columns per pricing call beat running more CG iterations?
#
# Motivation, from the n=10..30 scaling grid: iteration counts are small (38-71
# on the cells that ran out of time), so the cost is per-iteration, not the
# number of iterations. Inside those iterations the split is stark on n=30/p=16:
#
#   early_return  38 iters  3897s pricing   715 cols -> 5.5 s/col  (32/38 hit the 120s cap)
#   certification  5 iters  6900s pricing  3908 cols -> 1.8 s/col
#
# A pricing call's cost is dominated by exploring the label space, not by
# harvesting columns out of it -- and `n_candidates=20` (x3 scenarios = 60)
# throws most of each search away. Early iterations hit that harvest cap
# outright (58-60 columns returned); later ones run the full 120s and return
# 0-5. Certification, with the harvest cap lifted and a longer budget, pulls
# 1279-3706 columns from a single call.
#
# So this runs the hardest truncated cell (n=30, p=16) with the harvest cap and
# pricing budget raised, against the grid's own baseline of n_candidates=20 /
# 120s. Metric: does it certify inside the same 3h, and where does lp_bound get
# to if not.
#
#SBATCH --job-name=pfa_harvest
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=04:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/pfa-harvest-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/pfa-harvest-%j.err

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"

# Variant knobs, supplied by the submitter. Everything else is pinned to the
# scaling grid's settings so the comparison is clean.
CAND="${HARVEST_CAND:?set HARVEST_CAND}"
PTIME="${HARVEST_PTIME:?set HARVEST_PTIME}"
N_STATIONS="${HARVEST_N:-30}"
N_PAIRS="${HARVEST_P:-16}"

OUTDIR="$PROJECT_ROOT/experiments/2026-07-30_pfa_harvest/n${N_STATIONS}_p${N_PAIRS}_cand${CAND}_pt${PTIME}_cd${HARVEST_COMPDOM:-1}"
mkdir -p "$OUTDIR"

module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6

cd "$PROJECT_ROOT"
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"

export PFA_N_PAIRS="$N_PAIRS"
export PFA_N_SCENARIOS=3
export PFA_SEEDS=42
export PFA_MAX_STOPS=0
export PFA_MAX_VISITS=3
export PFA_STATION_BUDGET_CAP=0
export PFA_CASE_TIME=10800
export PFA_CERT_TIME=1800
export PFA_IP_TIME=900
# The two variables under test.
export PFA_N_CANDIDATES="$CAND"
export PFA_PRICING_TIME="$PTIME"
# Dominance rule under test in the companion A/B: compensated prices faster but
# yields ~50% fewer distinct columns per search.
export PFA_COMPENSATED_DOMINANCE="${HARVEST_COMPDOM:-1}"

echo "=== n=$N_STATIONS p=$N_PAIRS n_candidates=$CAND pricing_time=${PTIME}s compensated_dominance=${HARVEST_COMPDOM:-1} ==="
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    "$PROJECT_ROOT/scripts/passenger_free_assignment_cg_scaling.jl" "$OUTDIR" "$N_STATIONS"
