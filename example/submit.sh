#!/bin/bash
#SBATCH -J station_selection           # Job name
#SBATCH -N 1                           # 1 node
#SBATCH --ntasks=1                     # 1 task
#SBATCH --cpus-per-task=4              # 4 CPU cores
#SBATCH --mem=128G                     # 128GB memory
#SBATCH -o logs/selection-%j.out       # Output log
#SBATCH -e logs/selection-%j.err       # Error log
#SBATCH --time=02:00:00                # 2 hour time limit

# =============================================================================
# StationSelection Example Run Script
# =============================================================================
# Usage (run from project root):
#   Local:  ./example/submit.sh [config_file]
#   SLURM:  sbatch example/submit.sh [config_file]
#
# If no config file is specified, defaults to example/config.toml
# =============================================================================

# Set project root (use SLURM_SUBMIT_DIR if on cluster, otherwise current dir)
if [ -n "$SLURM_SUBMIT_DIR" ]; then
	PROJECT_ROOT="$SLURM_SUBMIT_DIR"
else
	PROJECT_ROOT="$(pwd)"
fi

# Config file: use argument if provided, otherwise default
CONFIG_FILE="${1:-example/config.toml}"

# Make config path absolute if relative
if [[ ! "$CONFIG_FILE" = /* ]]; then
	CONFIG_FILE="$PROJECT_ROOT/$CONFIG_FILE"
fi

echo "============================================================"
echo "StationSelection Optimization"
echo "============================================================"
echo "Project root: $PROJECT_ROOT"
echo "Config file:  $CONFIG_FILE"
echo "Start time:   $(date)"
echo ""

# Check if config file exists
if [ ! -f "$CONFIG_FILE" ]; then
	echo "ERROR: Config file not found: $CONFIG_FILE"
	exit 1
fi

# Create logs directory if it doesn't exist
mkdir -p "$PROJECT_ROOT/logs"

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
cd "$PROJECT_ROOT"

# Run the optimization
echo "===== Running Optimization ====="
julia --project=. example/run.jl --config "$CONFIG_FILE"
EXIT_CODE=$?

echo ""
echo "============================================================"
echo "Completed"
echo "============================================================"
echo "Exit code: $EXIT_CODE"
echo "End time:  $(date)"

exit $EXIT_CODE
