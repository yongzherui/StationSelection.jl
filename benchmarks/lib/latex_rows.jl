"""
`latex_rows.jl` -- shared helper for emitting manuscript-ready LaTeX row macros
alongside a study's summary CSV, following the convention already established in
`../../experiments/2026-07-15_restricted_pricing_report/slides_results.tex`: one
`\newcommand{\SomeRowName}{col & col & col & ...}` per table row, values pre-formatted
to display precision, meant to be `\input` directly into
`../../../VBS-Location-Manuscripts/Presentation/main.tex` (or a slides deck) rather than
hand-transcribed from a CSV.

Every study's `analyze.jl` should call `write_latex_rows` once its summary CSV is built,
writing `<results_dir>/slides_results.tex` next to the CSVs.

TODO: not implemented yet -- this is the one genuinely new shared piece (instance
generation and SLURM plumbing are referenced from `../../scripts/` in place, not
duplicated here). Fill in once the first study's `analyze.jl` needs it, rather than
speculatively now, so the row-name/formatting conventions are driven by a real table.
"""

export write_latex_rows, format_latex_value

"""
    format_latex_value(x; digits=2) -> String

Render one cell for a `\newcommand` row macro. TODO: decide the precision/formatting
convention here (e.g. 2 decimal places for seconds/percentages, integers for counts,
`--` for missing/censored cells per the OOM/timeout bookkeeping convention in
`notes/2026-08-05_free_assignment_cg_direct_ms5_comparison.md`) and implement.
"""
function format_latex_value end

"""
    write_latex_rows(path::String, rows::Vector{<:NamedTuple}; name_field::Symbol)

Write one `\newcommand{\<CamelCase name_field value>}{col & col & ...}` per row of
`rows` to `path`. TODO: implement -- camel-case the row-name field into a valid LaTeX
command name, format every other field via `format_latex_value`, join with ` & `.
"""
function write_latex_rows end
