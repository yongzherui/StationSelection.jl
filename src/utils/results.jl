module Results

using DataFrames
using JuMP

export Result

"""
Result struct that describes the output of station selection optimization methods.

# Fields
- `method::String`: Name of the optimization method used
- `status::Bool`: Whether optimization was successful
- `value::Union{Vector{Int}, Nothing}`: Vector of selected station IDs (if successful)
- `stations::Union{Dict{Int, Bool}, Nothing}`: Dictionary mapping station ID to selection status
- `station_df::DataFrame`: DataFrame containing station information
- `model::Union{JuMP.Model, Nothing}`: The JuMP optimization model
- `metadata::Dict{String, Any}`: Additional metadata about the optimization run
"""
struct Result
    method::String
    status::Bool
    value::Union{Vector{Int}, Nothing}
    stations::Union{Dict{Int, Bool}, Nothing}
    station_df::DataFrame
    model::Union{JuMP.Model, Nothing}
    metadata::Dict{String, Any}
end

# Convenience constructor without metadata (defaults to empty Dict)
function Result(
    method::String,
    status::Bool,
    value::Union{Vector{Int}, Nothing},
    stations::Union{Dict{Int, Bool}, Nothing},
    station_df::DataFrame,
    model::Union{JuMP.Model, Nothing}
)
    return Result(method, status, value, stations, station_df, model, Dict{String, Any}())
end

end # module
