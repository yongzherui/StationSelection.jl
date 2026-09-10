# Certification

The loop that turns the [Relaxed-cluster relaxation](@ref) into a proof of CG optimality:
price the relaxation, exact-search the cluster supports it names, and cut the supports that
come back barren until one attempt exhausts with nothing improving left.

Only an *exhausted* subset search may be cut on, and the obvious stronger cut form is
unsound -- `certify.jl` and `cuts.jl` below say why.

## The certification round

{{autodocs opt/label_setting/joint_routing_assignment/relaxed_cluster/utils/certification}}

## Guiding and refinement

{{autodocs opt/label_setting/joint_routing_assignment/relaxed_cluster/utils/guiding opt/label_setting/joint_routing_assignment/relaxed_cluster/utils/refinement}}
