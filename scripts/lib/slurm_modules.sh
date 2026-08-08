# scripts/lib/slurm_modules.sh
#
# Shared module loads for single-task (non-array) sbatch launchers
# (sbatch_bench_passenger_free_assignment_labels.sh, sbatch_diagnose.sh,
# sbatch_passenger_free_assignment_cg_scaling.sh, sbatch_run_test.sh,
# sbatch_run_tests.sh). These run once per submission rather than as a SLURM
# array, so -- unlike scripts/lib/slurm_array_task_env.sh -- there is no
# concurrent-depot-access race to avoid, hence no depot copy here.

module load gcc/12.2.0
module load community-modules
module load StdEnv
module load gurobi/12.0.3
module load julia/1.12.6
