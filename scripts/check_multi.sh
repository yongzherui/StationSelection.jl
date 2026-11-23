#!/bin/bash
# run several check seeds in one julia session-free sequence
for seed in "$@"; do
  echo "== seed $seed"
  julia --startup-file=no --project=/home/yongzr/ss-work/env_cap /home/yongzr/ss-work/exact_capacitated/scripts/check_exact_capacitated.jl $seed ${TRIALS:-150}
done
