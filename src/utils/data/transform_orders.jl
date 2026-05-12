"""
Order transformation utilities for converting selected station results to simulation-ready
orders.

This module provides functionality to:
1. Read station selection results (with `selected` column or timeframe-specific columns)
2. Transform original orders by assigning pickup/dropoff to selected stations
3. Replay exported assignment decisions onto orders
4. Remap scenario times for stacked simulations
5. Extend uncovered windows using `z*`/`y*` station decisions
"""

using CSV
using DataFrames
using Distances
using Dates

export transform_orders,
       transform_orders_from_assignments,
       transform_orders_quick_extend,
       remap_order_times_stacked,
       parse_station_list,
       precompute_distances,
       find_closest_selected_station,
       get_timeframe_column

include("transform_orders/common.jl")
include("transform_orders/time_lookup.jl")
include("transform_orders/selection_fallback.jl")
include("transform_orders/assignment_replay.jl")
include("transform_orders/quick_extend.jl")
