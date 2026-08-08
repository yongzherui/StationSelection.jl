# scripts/lib/slurm_array_task_env.sh
#
# Shared module-load + Julia-depot setup for SLURM ARRAY TASK runners
# (sbatch_method_compare.sh, sbatch_single_instance.sh, sbatch_zhuzhou_instance.sh).
# Source this after `TASK="${SLURM_ARRAY_TASK_ID:-}"` is set.
#
# The depot is rsync'd into a per-job scratch path (not shared ~/.julia)
# because concurrent array tasks sharing one depot can race on the precompile
# cache; set CS_COPY_DEPOT=0 to use the shared depot directly instead (faster
# startup, only safe for non-array or otherwise non-concurrent submissions).

echo "===== Loading modules ====="
JULIA_MODULE="${CS_JULIA_MODULE:-julia/1.12.6}"
GUROBI_MODULE="${CS_GUROBI_MODULE:-}"

module load "$JULIA_MODULE"
if [ -n "$GUROBI_MODULE" ]; then
    module load "$GUROBI_MODULE"
fi
julia --version
echo ""

echo "===== Setting up Julia depot ====="
JULIA_VERSION=$(julia --startup-file=no -e 'print(VERSION)')
COPY_DEPOT="${CS_COPY_DEPOT:-1}"
if [ "$COPY_DEPOT" = "0" ]; then
    export JULIA_DEPOT_PATH="${JULIA_DEPOT_PATH:-$HOME/.julia}"
    echo "Using existing depot: $JULIA_DEPOT_PATH"
else
    if [ -n "${SLURM_TMPDIR:-}" ]; then
        export JULIA_DEPOT_PATH="$SLURM_TMPDIR/julia_depot_v${JULIA_VERSION}"
    else
        export JULIA_DEPOT_PATH="/tmp/$USER/julia_depot_v${JULIA_VERSION}_${SLURM_ARRAY_JOB_ID}_${TASK}"
    fi
    mkdir -p "$JULIA_DEPOT_PATH"
    rsync -a --exclude='compiled/' --exclude='logs/' ~/.julia/ "$JULIA_DEPOT_PATH/"
    echo "Depot ready: $JULIA_DEPOT_PATH"
fi
echo ""
