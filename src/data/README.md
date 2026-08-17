# Data Layer

- `core/`: shared in-memory data structs, scenario helpers, station index mappings, and cost accessors.
- `io/`: raw station/request CSV loading helpers.
- `maps/`: formulation-specific optimization maps derived from `StationSelectionData`
  (e.g. `ClusteringTwoStageODMap`, `AggregateODRouteMap`); dispatched via `create_map`.
