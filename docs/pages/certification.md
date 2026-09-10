# Certification

`:relaxed_cluster` is the only mode that can certify. It runs on `K` cluster nodes rather
than `n` stations, and because its minimum reduced cost lower-bounds every *real* route's,
exhausting it without finding anything improving proves no real improving column exists —
regardless of which pricer found the columns.

Two properties are easy to get backwards, and both are load-bearing:

1. **It prices first and certifies second.** The ordinary outcome of an attempt is a
   column, not a proof. That is the mode working as designed.
2. **The no-good cuts are the mechanism, not an optimization on top of it.** A cut-free
   round is exactly this loop's round 1, and round 1 does not certify. The relaxation's
   slack is orders of magnitude larger than the margin a converged master leaves.

The cut direction also matters: only an *exhausted* subset search may be cut on, and the
obvious stronger cut form is unsound. See `certify.jl` below for why.

```@autodocs
Modules = [StationSelection]
Pages = [
    "opt/label_setting/joint_routing_assignment/relaxed_cluster/data.jl",
    "opt/label_setting/joint_routing_assignment/relaxed_cluster/clustering.jl",
    "opt/label_setting/joint_routing_assignment/relaxed_cluster/relaxation.jl",
    "opt/label_setting/joint_routing_assignment/relaxed_cluster/cuts.jl",
]
```

### The certification round

```@autodocs
Modules = [StationSelection]
Pages = [
    "opt/label_setting/joint_routing_assignment/relaxed_cluster/utils/certification/results.jl",
    "opt/label_setting/joint_routing_assignment/relaxed_cluster/utils/certification/common.jl",
    "opt/label_setting/joint_routing_assignment/relaxed_cluster/utils/certification/round.jl",
    "opt/label_setting/joint_routing_assignment/relaxed_cluster/utils/certification/certify.jl",
]
```

### Guiding and refinement

```@autodocs
Modules = [StationSelection]
Pages = [
    "opt/label_setting/joint_routing_assignment/relaxed_cluster/utils/guiding/guide.jl",
    "opt/label_setting/joint_routing_assignment/relaxed_cluster/utils/refinement/refine.jl",
]
```
