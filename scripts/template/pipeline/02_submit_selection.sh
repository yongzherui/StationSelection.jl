#!/bin/bash
#SBATCH -J selection                   # Job name
#SBATCH -p mit_normal                  # Partition (adjust to your cluster)
#SBATCH -N 1                           # 1 node per job
#SBATCH --ntasks=1                     # 1 task per job
#SBATCH --cpus-per-task=8              # 8 CPU cores
#SBATCH --mem=128G                     # 128GB memory per job
#SBATCH --array=1-2                    # Job array (ADJUST based on job count)
#SBATCH -o <study_path>/slurm_logs/selection-%A_%a.out
#SBATCH -e <study_path>/slurm_logs/selection-%A_%a.err
#SBATCH --time=04:00:00                # 4 hour time limit per selection

# NOTE: Update --array=1-N with actual job count from 01_setup_pipeline.jl output

PROJECT_ROOT="$SLURM_SUBMIT_DIR"
STUDY_DIR="$PROJECT_ROOT/<study_path>"

echo "===== Station Selection Job ====="
echo "Study: $STUDY_DIR"
echo "Project: $PROJECT_ROOT"
echo "Job Array Task ID: $SLURM_ARRAY_TASK_ID"
echo "Job ID: $SLURM_JOB_ID"
echo "Start time: $(date)"
echo ""

module load julia/1.10.4
module load gurobi

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

echo "Job ID: $JOB_ID"
echo "Config: $STUDY_DIR/config/selection/job_${JOB_ID}.toml"
echo ""

if command -v stdbuf >/dev/null 2>&1; then
    JULIA_CMD="stdbuf -oL -eL julia"
else
    JULIA_CMD="julia"
fi

$JULIA_CMD "$STUDY_DIR/scripts/03_run_selection.jl" "$JOB_ID"
EXIT_CODE=$?

if [ $EXIT_CODE -ne 0 ]; then
    echo "ERROR: Selection failed with exit code $EXIT_CODE"
    exit $EXIT_CODE
fi

echo ""
echo "===== Selection Complete ====="
echo "Exit code: 0"
echo "End time: $(date)"

exit 0
