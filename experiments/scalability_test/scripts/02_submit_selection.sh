#!/bin/bash
#SBATCH -J selection                   # Job name
#SBATCH -p mit_normal                  # Partition (adjust to your cluster)
#SBATCH -N 1                           # 1 node per job
#SBATCH --ntasks=1                     # 1 task per job
#SBATCH --cpus-per-task=8              # 8 CPU cores
#SBATCH --mem=128G                     # 128GB memory per job
#SBATCH --array=1-8                    # Job array (ADJUST based on job count)
#SBATCH -o slurm_logs/selection-%A_%a.out
#SBATCH -e slurm_logs/selection-%A_%a.err
#SBATCH --time=04:00:00                # 4 hour time limit per selection

# =============================================================================
# StationSelection Scalability Test - Selection Stage
# =============================================================================
# Usage:
#   sbatch experiments/scalability_test/scripts/02_submit_selection.sh
# =============================================================================

# Set project root (use SLURM_SUBMIT_DIR if on cluster, otherwise current dir)
if [ -n "$SLURM_SUBMIT_DIR" ]; then
	PROJECT_ROOT="$SLURM_SUBMIT_DIR"
else
	PROJECT_ROOT="$(pwd)"
fi

STUDY_DIR="$PROJECT_ROOT/experiments/scalability_test"

echo "============================================================"
echo "StationSelection Scalability Test - Selection"
echo "============================================================"
echo "Project root: $PROJECT_ROOT"
echo "Study dir:    $STUDY_DIR"
echo "Start time:   $(date)"
echo ""

# Create logs directory if it doesn't exist
mkdir -p "$STUDY_DIR/slurm_logs"

# Detect environment and load modules if on cluster
if command -v module &>/dev/null; then
	echo "===== Loading modules ====="
	module load julia 2>/dev/null || module load julia/1.10.4 2>/dev/null || true
	module load gurobi 2>/dev/null || true
	echo ""
fi

# Check Julia is available
if ! command -v julia &>/dev/null; then
	echo "ERROR: Julia not found. Please ensure Julia is installed and in PATH."
	exit 1
fi

echo "===== Environment ====="
echo "Julia version: $(julia --version)"
echo "Working directory: $PROJECT_ROOT"
if [ -n "$SLURM_JOB_ID" ]; then
	echo "SLURM Job ID: $SLURM_JOB_ID"
	echo "Node: $SLURM_NODELIST"
fi
echo ""

# Navigate to project root
cd "$PROJECT_ROOT" || exit 1

SELECTION_JOB_FILE="$STUDY_DIR/config/selection_jobs.txt"
if [ ! -f "$SELECTION_JOB_FILE" ]; then
	echo "ERROR: Selection job file not found: $SELECTION_JOB_FILE"
	echo "Please run 01_setup_pipeline.jl first"
	exit 1
fi

JOB_ID=$(sed -n "${SLURM_ARRAY_TASK_ID}p" "$SELECTION_JOB_FILE")
if [ -z "$JOB_ID" ]; then
	echo "ERROR: Could not read job ID for task ID $SLURM_ARRAY_TASK_ID"
	exit 1
fi

echo "===== Station Selection ====="
echo "Job ID:  $JOB_ID"
echo "Config:  $STUDY_DIR/config/selection/job_${JOB_ID}.toml"
echo ""

julia --project=. "$STUDY_DIR/scripts/03_run_selection.jl" "$JOB_ID"
EXIT_CODE=$?

echo ""
echo "============================================================"
echo "Completed"
echo "============================================================"
echo "Exit code: $EXIT_CODE"
echo "End time:  $(date)"

exit $EXIT_CODE
