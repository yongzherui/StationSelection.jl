# Instance generation

Synthetic and real-instance builders. The Zhuzhou generator is what every benchmark in
`benchmarks/` is built on; the `test*` cases are small hand-built geometries with known
answers, used to pin specific behaviours rather than to measure performance.

Note on the Zhuzhou generator: **the seed varies demand only, not geography.** Stations are
the deterministic top-N by popularity, so the partition and the cost matrices are identical
across seeds and only the OD draw changes.

## Zhuzhou

```@autodocs
Modules = [StationSelection]
Pages = ["generators/zhuzhou.jl"]
```

## Grid

```@autodocs
Modules = [StationSelection]
Pages = ["generators/grid.jl"]
```

## Hand-built test cases

```@autodocs
Modules = [StationSelection]
Pages = [
    "generators/test_cases/base_middle_zone.jl",
    "generators/test_cases/test1_vehicle.jl",
    "generators/test_cases/test2_zone_proximity.jl",
    "generators/test_cases/test3_north_shift.jl",
    "generators/test_cases/test4_mirrored_zone.jl",
    "generators/test_cases/test5_triangle.jl",
    "generators/test_cases/test6_bidirectional.jl",
]
```
