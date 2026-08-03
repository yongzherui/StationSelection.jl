#!/bin/bash
# Array task 0 benchmarks the pre-port PFA snapshot; task 1 benchmarks the
# Nearest Open mechanical port plus its shared-mechanics refactor.
# Submit with NEAREST_OPEN_AB_OUTDIR set to a shared results directory.
#SBATCH --job-name=no_pricer_ab
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=24G
#SBATCH --time=00:30:00
#SBATCH --array=0-1

set -euo pipefail

REPO="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
OUTDIR="${NEAREST_OPEN_AB_OUTDIR:?set NEAREST_OPEN_AB_OUTDIR}"
TASK="${SLURM_ARRAY_TASK_ID:?submit as an array}"
case "$TASK" in
    0) LABEL=baseline; COMMIT=f0bdfaf ;;
    1) LABEL=optimized; COMMIT=3eca2e3 ;;
    *) exit 2 ;;
esac

module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6

SNAP="${SLURM_TMPDIR:-/tmp/$USER/no_pricer_ab_$SLURM_JOB_ID}/StationSelection.jl"
mkdir -p "$SNAP" "$OUTDIR"
git -C "$REPO" archive "$COMMIT" -o "$SNAP/source.tar"
tar -xf "$SNAP/source.tar" -C "$SNAP"
cp "$REPO/Manifest.toml" "$SNAP/Manifest.toml"
cp "$REPO/scripts/bench_nearest_open_pricer_mechanics.jl" "$SNAP/bench.jl"

export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
export JULIA_DEPOT_PATH="${SLURM_TMPDIR:-/tmp/$USER/no_pricer_ab_$SLURM_JOB_ID}/julia_depot"
mkdir -p "$JULIA_DEPOT_PATH"
rsync -a --exclude='compiled/' --exclude='logs/' ~/.julia/ "$JULIA_DEPOT_PATH/"

cd "$SNAP"
echo "BENCH label=$LABEL commit=$COMMIT n=10"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$SNAP" \
    "$SNAP/bench.jl" 10 "$OUTDIR/${LABEL}_signatures.txt"
