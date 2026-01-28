#!/bin/bash
#SBATCH -J selection                   # Job name
#SBATCH -p mit_normal                  # Partition (adjust to your cluster)
#SBATCH -N 1                           # 1 node per job
#SBATCH --ntasks=1                     # 1 task per job
#SBATCH --cpus-per-task=8              # 8 CPU cores
#SBATCH --mem=128G                     # 128GB memory per job
#SBATCH --array=1-2                   # Job array (ADJUST based on job count from 01_setup_pipeline.jl)
#SBATCH -o <study_path>/slurm_logs/selection-%A_%a.out   # Output log
#SBATCH -e <study_path>/slurm_logs/selection-%A_%a.err   # Error log
#SBATCH --time=04:00:00                # 4 hour time limit per selection

# NOTE: This script runs ONLY the selection stage
# Transformation and simulation stages should be submitted with dependencies
# Update --array=1-N with actual job count from 01_setup_pipeline.jl output

# Get study directory (parent of scripts/)
PROJECT_ROOT="$SLURM_SUBMIT_DIR"
STUDY_DIR="$PROJECT_ROOT/<study_path>"

echo "===== Pipeline Experiment Job ====="
echo "Study: $STUDY_DIR"
echo "Project: $PROJECT_ROOT"
echo "Job Array Master ID: $SLURM_ARRAY_JOB_ID"
echo "Job Array Task ID: $SLURM_ARRAY_TASK_ID"
echo "Job ID: $SLURM_JOB_ID"
echo "Node: $SLURM_NODELIST"
echo "Start time: $(date)"
echo ""

# Load modules
echo "===== Loading modules ====="
module load julia/1.10.4
module load gurobi

julia --version
echo ""

# Navigate to project root
cd "$PROJECT_ROOT"
echo "Working directory: $(pwd)"
echo ""

# Read job parameters
SELECTION_JOB_FILE="$STUDY_DIR/config/selection_jobs.txt"

if [ ! -f "$SELECTION_JOB_FILE" ]; then
	echo "ERROR: Selection job file not found: $SELECTION_JOB_FILE"
	echo "Please run 01_setup_pipeline.jl first"
	exit 1
fi

# Read the job ID from the file
JOB_ID=$(sed -n "${SLURM_ARRAY_TASK_ID}p" "$SELECTION_JOB_FILE")

if [ -z "$JOB_ID" ]; then
	echo "ERROR: Could not read job ID for task ID $SLURM_ARRAY_TASK_ID"
	exit 1
fi

echo "===== Station Selection ====="
echo "Job ID: $JOB_ID"
echo "Config: $STUDY_DIR/config/selection/job_${JOB_ID}.toml"
echo ""

julia "$STUDY_DIR/scripts/03_run_selection.jl" "$JOB_ID"
SELECTION_EXIT=$?

if [ $SELECTION_EXIT -ne 0 ]; then
	echo "ERROR: Selection failed with exit code $SELECTION_EXIT"
	exit $SELECTION_EXIT
fi

echo ""
echo "===== Selection Complete ====="
echo "Exit code: 0"
echo "End time: $(date)"

exit 0
