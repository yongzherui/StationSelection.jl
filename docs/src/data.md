# Data & mappings

## Instance data

```@autodocs
Modules = [StationSelection]
Pages = [
    "data/core/struct.jl",
]
```

## Mappings

A mapping holds the index bookkeeping that `build_model` and the analysis helpers rely on:
station id ↔ array index, scenario label ↔ index, and (for AggregateODRoute) the
`Omega_s`/`Q_s` demand-group indexing. Each formulation family has its own, because the
bookkeeping a formulation needs is part of how it is encoded.

```@autodocs
Modules = [StationSelection]
Pages = [
    "data/maps/create_map.jl",
    "data/maps/clustering_base_map.jl",
    "data/maps/clustering_od_map.jl",
    "data/maps/clustering_two_stage_station_map.jl",
    "data/maps/aggregate_od_route_map.jl",
]
```

## IO

```@autodocs
Modules = [StationSelection]
Pages = [
    "data/io/stations.jl",
    "data/io/requests.jl",
]
```
