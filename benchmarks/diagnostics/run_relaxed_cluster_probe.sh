#!/bin/bash
#SBATCH --job-name=rc_probe
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=3
#SBATCH --mem=8G
#SBATCH --time=02:00:00
#SBATCH -o /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/rc-probe-%j.out
#SBATCH -e /home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/slurm_logs/rc-probe-%j.err
set -euo pipefail
export JULIA_NUM_THREADS="${SLURM_CPUS_PER_TASK:-1}"
PROJECT_ROOT=/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl
source "$PROJECT_ROOT/scripts/lib/slurm_modules.sh"
cd "$PROJECT_ROOT"
export PROBE_CERT_MODE="${PROBE_CERT_MODE:-relaxed_cluster}"
export PROBE_K="${PROBE_K:-base,3,6,9,12,15}"
export PROBE_CERT_LIMIT="${PROBE_CERT_LIMIT:-60.0}"
export PROBE_CERT_ROUNDS="${PROBE_CERT_ROUNDS:-32}"
stdbuf -oL -eL julia --startup-file=no --color=no --project="$PROJECT_ROOT" \
    benchmarks/diagnostics/relaxed_cluster_certification_probe.jl
