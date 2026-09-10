# Data & mappings

## Instance data

{{autodocs data/core}}

## Mappings

A mapping holds the index bookkeeping that `build_model` and the analysis helpers rely on:
station id ↔ array index, scenario label ↔ index, and (for AggregateODRoute) the
`Omega_s`/`Q_s` demand-group indexing. Each formulation family has its own, because the
bookkeeping a formulation needs is part of how it is encoded.

{{autodocs data/maps}}

## IO

{{autodocs data/io}}
