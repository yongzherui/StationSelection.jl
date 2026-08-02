#!/bin/bash
#SBATCH --job-name=cg_vs_direct_grid
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=8G
#SBATCH --time=02:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/cg-vs-direct-grid-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/slurm_logs/cg-vs-direct-grid-%j.err

set -uo pipefail

PROJECT_ROOT="/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl"
SCRIPT="$PROJECT_ROOT/scripts/diag_passenger_cg_vs_direct_full_milp.jl"

module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6

cd "$PROJECT_ROOT"

# grid of (n_stations, n_pairs, max_stops); kept small so the COMPLETE column
# set stays enumerable. One Julia process per line (fresh GRB env each).
CASES=(
  "6 4 3"
  "8 4 3"
  "10 4 3"
  "10 6 4"
  "12 4 3"
)

overall=0
for c in "${CASES[@]}"; do
  echo "############################################################"
  echo "### CASE: $c"
  echo "############################################################"
  julia --startup-file=no --project="$PROJECT_ROOT" "$SCRIPT" $c
  rc=$?
  echo "### CASE $c exit code: $rc"
  if [ "$rc" -ne 0 ]; then overall=1; fi
  echo
done

echo "############################################################"
echo "### GRID OVERALL exit code: $overall"
exit $overall
