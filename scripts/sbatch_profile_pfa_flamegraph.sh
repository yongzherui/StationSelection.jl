#!/bin/bash
# Flame-graph profile of the PFA pricers. Same frozen-snapshot discipline as
# scripts/submit_pfa_bench_snapshot.sh: the job reads package source when it
# STARTS, so editing src/ while it queues would profile the wrong code.
#
# Usage: scripts/sbatch_profile_pfa_flamegraph.sh <label> [profile args...]
#   e.g. scripts/sbatch_profile_pfa_flamegraph.sh micro --cases 20:5:3
#
# Writes HTML/folded output to <project_root>/tmp_ss_bench/pfa_profile/ in the
# ORIGINAL tree (not the snapshot), so results survive the next snapshot wipe.

set -euo pipefail

LABEL="${1:?usage: sbatch_profile_pfa_flamegraph.sh <label> [args...]}"
shift

ROOT="/home/yongzr/2025-09-JacqWang-Microtransit"
REPO="$ROOT/StationSelection.jl"
SNAP_ROOT="$HOME/.pfa_bench_snapshots"
SNAP="$SNAP_ROOT/profile_$LABEL/StationSelection.jl"
LOGS="$ROOT/slurm_logs"
OUT="$REPO/tmp_ss_bench/pfa_profile"

mkdir -p "$LOGS" "$OUT"
rm -rf "${SNAP_ROOT:?}/profile_$LABEL"
mkdir -p "$SNAP"
ln -s "$ROOT/Data" "$SNAP_ROOT/profile_$LABEL/Data"

cp -r "$REPO/src" "$REPO/scripts" "$SNAP/"
cp "$REPO/Project.toml" "$REPO/Manifest.toml" "$SNAP/"

cat > "$SNAP/run.sh" <<EOF
#!/bin/bash
#SBATCH --job-name=pfaprof_$LABEL
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=24G
#SBATCH --time=01:30:00
#SBATCH -o $LOGS/pfaprof-$LABEL-%j.out
#SBATCH -e $LOGS/pfaprof-$LABEL-%j.err

set -uo pipefail
module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6

cd "$SNAP"
julia --startup-file=no --project="$SNAP" \\
    "$SNAP/scripts/profile_pfa_flamegraph.jl" --out "$OUT" --label "$LABEL" "\$@"
EOF

chmod +x "$SNAP/run.sh"
sbatch "$SNAP/run.sh" "$@"
