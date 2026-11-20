export joint_split_column_max_load, joint_split_group_pax

"""
HEURISTIC vehicle capacity for Joint LAR column generation under M3 (`:exact_no_anticipation`,
`first_boarding = :after_last_walker`). Not exact capacitated pricing (that is `:exact_capacitated`). Added 2026-11-20
(study experiments/2026-11-17_g2_time_blind); usage in the package CLAUDE.md.

Off unless `CGPricingConfig(vehicle_capacity = C)` (passengers per vehicle; `nothing` = off). `CGPricingConfig`
accepts it only with `mode = :exact_no_anticipation, first_boarding = :after_last_walker`. Then:

1. **Passengers, not orders.** A demand group's load is the sum of its requests' `:pax` column (1 per
   request when the column is absent). Q (`demand_weighting = :count`) still counts orders in the
   objective; only the capacity uses passengers. A group is never split across vehicles.
2. **Split (LP master).** Every column added to the LP master (seeds and priced columns) whose on-board
   load exceeds the capacity on some leg is kept AND split: its groups (intervals pickup position ->
   drop-off position on the column's stop sequence, weight = passengers) are packed into vehicles so
   that each vehicle's load is <= capacity on every leg. Each vehicle drives the induced subsequence
   of the column's stops that its groups use, and becomes a new column. Packing is first-fit over
   three group orders (pickup position, drop-off position, passengers descending); the packing with the
   least total route time (drive + repositioning) is kept. A group heavier than the capacity on its
   own rides alone (the only over-capacity columns allowed).
3. **Timing check.** Skipping stops makes later stops come earlier, and under M3 a later rider boards
   only if they have walked there by then (the vehicle never waits after its first stop), so a
   subsequence of a feasible route is NOT always feasible. Every vehicle is checked against the M3
   rules before a group joins it: first stop t0 = the longest walk among its first-stop riders (the
   trigger; t0 <= pickup deadline), later boardings need walk(o, j) <= visit time <= pickup deadline,
   every rider's ride <= ride_time_limit_ratio * drive(j, k) and drop-off <= the door-to-door deadline
   from the request. A group that fits no vehicle opens a new one (a lone group on [j, k] always passes:
   the pair filter guarantees it).
4. **Integer recovery.** The final MIP keeps only columns within capacity (plus lone over-capacity
   groups). Exact: the MIP's columns are capacity-feasible and M3-feasible. Heuristic: the LP (and so
   pricing and convergence) is M3's uncapacitated master plus the split columns; capacity-feasible
   routes the split does not produce are never considered.

Counters (one Dict per solve, on the model as `:joint_split_stats`, shared with the integer-recovery rebuild) are
reported as `metadata["cg_split_stats"]`:
  - `checked_seed`, `checked_priced`: columns entering the LP master; `check_fail_seed`, `check_fail_priced`: of
    those, how many fail the M3 check on their own full route (a self-test: should be 0; nonzero = the check and the
    pricer disagree);
  - `over_capacity_columns`, `split_ok`, `split_failed`: columns over capacity and whether a packing passed;
    `pieces` (added), `pieces_duplicate` (already in the pool at no greater drive time);
  - `mip_columns_kept`, `mip_columns_dropped`: the integer recovery's pool after the capacity filter.

Improving the heuristic: `joint_split_column` is the only place that decides the pieces (packing orders, what a
vehicle may drive); everything else is wiring.
"""
_joint_split_count!(m::JuMP.Model, key::String, n::Int=1) =
    (d = m[:joint_split_stats]; d[key] = get(d, key, 0) + n; nothing)

_joint_split_on(m::JuMP.Model) = !isnothing(get(m.obj_dict, :joint_vehicle_capacity, nothing))

"""
    joint_split_group_pax(data, mapping) -> Dict{Int, Vector{Int}}

Passengers per demand group, parallel to `mapping.Omega_s[s]` (sum of the requests' `:pax`, 1 per request without it).
"""
function joint_split_group_pax(data::StationSelectionData, mapping::AggregateODRouteMap)
    out = Dict{Int, Vector{Int}}()
    for s in 1:n_scenarios(data)
        req = data.scenarios[s].requests
        haspax = hasproperty(req, :pax)
        pax = Dict{Tuple{Int, Int}, Int}()
        for row in eachrow(req)
            od = (row.origin_idx, row.dest_idx)
            pax[od] = get(pax, od, 0) + (haspax ? Int(row.pax) : 1)
        end
        out[s] = [pax[od] for od in mapping.Omega_s[s]]
    end
    return out
end

"Stash the capacity (`nothing` = off), passengers per group and the counters on the master (every joint CG build)."
function _joint_split_stash!(m::JuMP.Model, data::StationSelectionData, mapping::AggregateODRouteMap,
                             cap::Union{Nothing, Int})
    m[:joint_vehicle_capacity] = cap
    m[:joint_split_group_pax] = isnothing(cap) ? nothing : joint_split_group_pax(data, mapping)
    m[:joint_split_stats] = Dict{String, Int}()
    return nothing
end

"""
    joint_split_column_max_load(column, pax) -> Int

Most passengers on board on any leg of `column` (from its `assignment_positions`). `pax[s][p]` = passengers of
demand group `p` of scenario `s` (`joint_split_group_pax(data, mapping)`).
"""
function joint_split_column_max_load(column, pax::Dict{Int, Vector{Int}})::Int
    pos = column.metadata["assignment_positions"]
    s = Int(column.metadata["scenario"])
    arc = zeros(Int, max(length(column.route) - 1, 0))
    for (p, _j, _k) in column.assignments
        a, b = pos[p]
        for e in a:(b - 1); arc[e] += pax[s][p]; end
    end
    return maximum(arc; init = 0)
end

"Allowed in the capacitated MIP: within capacity, or one group alone."
_joint_split_column_ok(column, pax, cap) =
    length(column.assignments) == 1 || joint_split_column_max_load(column, pax) <= cap

# one group of a column being split: p, pickup j, drop-off k, positions (a, b) on the column's route, passengers w
const _SplitItem = NamedTuple{(:p, :j, :k, :a, :b, :w), Tuple{Int, Int, Int, Int, Int, Int}}

"""
The induced route of `items` on `route`: the stop positions they use, consecutive repeats of a station merged.
Returns (stops, new position per original position used).
"""
function _joint_split_induced(route::Vector{Int}, items; keep_all::Bool=false)
    P = keep_all ? collect(eachindex(route)) : sort!(unique!(vcat([it.a for it in items], [it.b for it in items])))
    stops = Int[]; newpos = Dict{Int, Int}()
    for q in P
        (isempty(stops) || stops[end] != route[q]) && push!(stops, route[q])
        newpos[q] = length(stops)
    end
    return stops, newpos
end

"""
M3 check of `items` on their induced route (see the file docstring, point 3). Returns (ok, stops, positions, t0, tau).
`positions` maps p => (pickup, drop-off) on `stops`.
"""
function _joint_split_m3_check(m::JuMP.Model, data::StationSelectionData, mapping::AggregateODRouteMap, s::Int,
                               route::Vector{Int}, items; keep_all::Bool=false)
    stops, newpos = _joint_split_induced(route, items; keep_all = keep_all)
    pos = Dict{Int, Tuple{Int, Int}}(it.p => (newpos[it.a], newpos[it.b]) for it in items)
    omega = mapping.Omega_s[s]
    ratio = mapping.door_to_door_limit_ratio
    rtl = Float64(m[:joint_routing_assignment_ride_time_limit_ratio])
    pd = Float64(m[:joint_routing_assignment_pickup_deadline_s])
    walk(it) = get_walking_cost(data, omega[it.p][1], it.j)
    t0 = maximum((walk(it) for it in items if pos[it.p][1] == 1); init = -Inf)
    fail = (false, stops, pos, 0.0, 0.0)
    (isfinite(t0) && t0 <= pd + 1e-9) || return fail       # no first-stop rider (cannot happen) or start too late
    T = zeros(length(stops)); T[1] = t0
    for i in 2:length(stops)
        T[i] = T[i - 1] + get_routing_cost(data, stops[i - 1], stops[i])
    end
    for it in items
        a, b = pos[it.p]
        o, d = omega[it.p]
        direct = get_routing_cost(data, o, d)
        deadline = (isfinite(ratio) && isfinite(direct)) ? ratio * direct - get_walking_cost(data, it.k, d) : Inf
        cap = rtl * get_routing_cost(data, it.j, it.k)
        if a > 1
            (walk(it) <= T[a] + NO_ANTICIPATION_EPS && T[a] <= pd + 1e-9) || return fail
        end
        (T[b] - T[a] <= cap + 1e-9 && T[b] <= deadline + 1e-9) || return fail
    end
    return (true, stops, pos, t0, T[end] - T[1])
end

"Leg loads of `items` on the column's route stay within `cap` after adding `it`?"
function _joint_split_fits(load::Vector{Int}, it, cap::Int)
    for e in it.a:(it.b - 1)
        load[e] + it.w > cap && return false
    end
    return true
end

"""
    joint_split_column(m, data, mapping, column) -> Vector of (stops, assignments, positions, t0, tau)

The capacity split of an over-capacity column (file docstring, points 2-3). Empty when no packing passes.
"""
function joint_split_column(m::JuMP.Model, data::StationSelectionData, mapping::AggregateODRouteMap, column)
    cap = Int(m[:joint_vehicle_capacity]); pax = m[:joint_split_group_pax]
    rho = Float64(m[:joint_routing_assignment_repositioning_time])
    s = Int(column.metadata["scenario"]); route = column.route
    pos = column.metadata["assignment_positions"]
    items = _SplitItem[(p = p, j = j, k = k, a = pos[p][1], b = pos[p][2], w = pax[s][p]) for (p, j, k) in column.assignments]
    orders = (sort(items; by = it -> (it.a, it.b, it.p)), sort(items; by = it -> (it.b, it.a, it.p)),
              sort(items; by = it -> (-it.w, it.a, it.b, it.p)))
    best = nothing; best_cost = Inf
    for order in orders
        vehicles = Vector{Vector{_SplitItem}}(); loads = Vector{Vector{Int}}(); checks = Any[]
        ok = true
        for it in order
            placed = false
            for v in eachindex(vehicles)
                _joint_split_fits(loads[v], it, cap) || continue
                chk = _joint_split_m3_check(m, data, mapping, s, route, vcat(vehicles[v], it))
                chk[1] || continue
                push!(vehicles[v], it); checks[v] = chk
                for e in it.a:(it.b - 1); loads[v][e] += it.w; end
                placed = true; break
            end
            placed && continue
            chk = _joint_split_m3_check(m, data, mapping, s, route, [it])
            chk[1] || (ok = false; break)
            push!(vehicles, [it]); push!(checks, chk)
            l = zeros(Int, length(route) - 1); for e in it.a:(it.b - 1); l[e] += it.w; end
            push!(loads, l)
        end
        ok || continue
        cost = sum(c[5] + rho for c in checks)
        if cost < best_cost - 1e-9
            best_cost = cost
            best = [(c[2], [(it.p, it.j, it.k) for it in veh], c[3], c[4], c[5]) for (veh, c) in zip(vehicles, checks)]
        end
    end
    return something(best, Any[])
end

"""
    _joint_split_pass!(m, data, mapping, columns) -> Int

Split every over-capacity column of `columns` (already in the LP master) and add the pieces. Returns the number of
pieces added. No-op when the split is off or the master is the integer recovery.
"""
function _joint_split_pass!(m::JuMP.Model, data::StationSelectionData, mapping::AggregateODRouteMap, columns)
    _joint_split_on(m) || return 0
    Bool(m[:joint_routing_assignment_relax_integrality]) || return 0
    cap = Int(m[:joint_vehicle_capacity]); pax = m[:joint_split_group_pax]
    added = 0
    for column in columns
        # self-test of the M3 check on every column entering the master (seeds and priced)
        kind = haskey(column.metadata, "seed") ? "seed" : "priced"
        _joint_split_count!(m, "checked_$kind")
        joint_split_check_column(m, data, mapping, column) || _joint_split_count!(m, "check_fail_$kind")
        length(column.assignments) > 1 || continue
        joint_split_column_max_load(column, pax) > cap || continue
        _joint_split_count!(m, "over_capacity_columns")
        pieces = joint_split_column(m, data, mapping, column)
        if isempty(pieces)
            _joint_split_count!(m, "split_failed"); continue
        end
        _joint_split_count!(m, "split_ok"); _joint_split_count!(m, "pieces", length(pieces))
        s = Int(column.metadata["scenario"])
        for (stops, assignments, positions, t0, tau) in pieces
            id = maximum(keys(m[:joint_routing_assignment_columns]); init = 0) + 1
            piece = JointRoutingAssignmentRouteColumn(id, stops, assignments, tau;
                metadata = Dict{String, Any}("scenario" => s, "route" => Tuple(stops), "assignment_positions" => positions,
                                             "start_time" => t0, "split_from" => column.id))
            _theta, action = add_joint_routing_assignment_column!(m, data, mapping, piece)
            action === :added ? (added += 1) : _joint_split_count!(m, "pieces_duplicate")
        end
    end
    return added
end

"""
    joint_split_check_column(m, data, mapping, column) -> Bool

The M3 check (point 3) of a column on its own route and positions, with its own start time: every M3-priced column
must pass (a self-test of the check, counted as `check_priced_fail` when one does not).
"""
function joint_split_check_column(m::JuMP.Model, data::StationSelectionData, mapping::AggregateODRouteMap, column)
    s = Int(column.metadata["scenario"]); pos = column.metadata["assignment_positions"]
    items = _SplitItem[(p = p, j = j, k = k, a = pos[p][1], b = pos[p][2], w = 0) for (p, j, k) in column.assignments]
    # on the column's FULL route: a priced route may pass stops where none of its riders board or alight, and
    # dropping them makes later stops come earlier (the first version of this self-test did, and failed 5-8%)
    chk = _joint_split_m3_check(m, data, mapping, s, column.route, items; keep_all = true)
    # the check derives t0 from the first-stop riders; the pricer's start time must be the same
    return chk[1] && abs(chk[4] - Float64(get(column.metadata, "start_time", chk[4]))) <= 1e-6
end
