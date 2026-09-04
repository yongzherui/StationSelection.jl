#!/bin/bash
# Submit scripts/bench_passenger_free_assignment_labels.jl against a FROZEN snapshot
# of the current working tree.
#
# Why a snapshot: benchmark jobs sit in the SLURM queue for minutes, and Julia reads
# the package source when the job finally starts -- not when it was submitted. Editing
# src/ while a job is queued therefore silently benchmarks the *new* code and labels
# it with the old variant name. Copying the tree at submit time pins each result to
# exactly the code that was submitted, so variants can be pipelined instead of
# strictly serialized.
#
# The snapshot reproduces the real <project_root>/StationSelection.jl layout, with
# Data/ symlinked back to the original, because the bench script locates inputs as
# `@__DIR__/../../Data/base_data`.
#
# Usage: scripts/submit_pfa_bench_snapshot.sh <variant-name> [bench args...]
#   BENCH_SCRIPT=scripts/foo.jl scripts/submit_pfa_bench_snapshot.sh <name> [args...]
#     runs a different script (e.g. the brute-force correctness oracle) against the
#     same frozen snapshot.

set -euo pipefail

BENCH_SCRIPT="${BENCH_SCRIPT:-scripts/bench_passenger_free_assignment_labels.jl}"

VARIANT="${1:?usage: submit_pfa_bench_snapshot.sh <variant-name> [bench args...]}"
shift

ROOT="/home/yongzr/2025-09-JacqWang-Microtransit"
REPO="$ROOT/StationSelection.jl"
SNAP_ROOT="$HOME/.pfa_bench_snapshots"
SNAP="$SNAP_ROOT/$VARIANT/StationSelection.jl"
LOGS="$ROOT/slurm_logs"

mkdir -p "$LOGS"
rm -rf "${SNAP_ROOT:?}/$VARIANT"
mkdir -p "$SNAP"
ln -s "$ROOT/Data" "$SNAP_ROOT/$VARIANT/Data"

# Only what the benchmark needs to load the package and build instances.
cp -r "$REPO/src" "$REPO/scripts" "$REPO/test" "$SNAP/"
cp "$REPO/Project.toml" "$REPO/Manifest.toml" "$SNAP/"

cat > "$SNAP/run.sh" <<EOF
#!/bin/bash
#SBATCH --job-name=pfa_$VARIANT
#SBATCH --partition=mit_preemptable
# mit_preemptable, not mit_normal: mit_normal queues behind whatever else this
# account is running (a short job was estimated 29 h out during a large array),
# while preemptable nodes start in minutes and preemption is rare in practice.
# A preempted job shows as CANCELLED/PREEMPTED with truncated output -- check the
# sacct state before reading missing results as a failure.
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=01:30:00
#SBATCH -o $LOGS/pfa-$VARIANT-%j.out
#SBATCH -e $LOGS/pfa-$VARIANT-%j.err

set -uo pipefail
module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6

cd "$SNAP"
julia --startup-file=no --project="$SNAP" "$SNAP/$BENCH_SCRIPT" "\$@"
EOF

chmod +x "$SNAP/run.sh"
sbatch "$SNAP/run.sh" "$@"
