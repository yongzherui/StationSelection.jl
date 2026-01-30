# Pipeline Template

This folder contains a simple pipeline template for running station selection
using the StationSelection.jl package.

Files:
- `01_setup_pipeline.jl`: Prepare inputs and run directories.
- `02_submit_selection.sh`: Submit the run (HPC or local queue).
- `03_run_selection.jl`: Execute selection and write outputs.

Notes:
- `03_run_selection.jl` uses `run_opt` which returns an `OptResult`.
- Counts are available in `result.counts` when `show_counts=true` or `count=true`.
