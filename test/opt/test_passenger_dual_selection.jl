@testset "Pricing-aware dual selection" begin
    using Gurobi

    # One Gurobi.Env reused across this whole testset -- constructing several per
    # process has previously caused a silent multi-minute stall on this cluster.
    # (Plain local, not `const`: `const` is illegal in a @testset's local scope.)
    DS_ENV = Gurobi.Env()

    function tiny_instance(; n_stations::Int=4, l::Int=2, max_stops::Int=3)
        stations = DataFrame(
            id=collect(1:n_stations),
            lon=Float64.(0:(n_stations - 1)),
            lat=zeros(n_stations),
        )
        requests = DataFrame(
            id=[1, 2],
            origin_station_id=[1, 2],
            destination_station_id=[n_stations, n_stations - 1],
            request_time=[DateTime(2024, 1, 1, 8), DateTime(2024, 1, 1, 8, 1)],
        )
        walking = Dict{Tuple{Int, Int}, Float64}()
        routing = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:n_stations, j in 1:n_stations
            walking[(i, j)] = 10.0 * abs(i - j)
            routing[(i, j)] = Float64(abs(i - j))
        end
        data = create_station_selection_data(stations, requests, walking; routing_costs=routing)
        model = AggregateODRouteModel(
            l;
            route_regularization_weight = 1.0,
            walk_cost_weight            = 0.1,
            repositioning_time          = 0.0,
            max_walking_distance        = 1000.0,
            max_wait_time               = 100.0,
            detour_factor               = 3.0,
            max_stops                   = max_stops,
            max_visits_per_node         = 2,
        )
        return data, model
    end

    """Every physical route within the caps, as concrete columns via replay."""
    function enumerate_all_columns(md, rewards_all_positive=true)
        nodes = md.nodes
        max_stops = md.max_stops == typemax(Int) ? length(nodes) : md.max_stops
        max_visits = md.max_visits_per_node == typemax(Int) ? max_stops : md.max_visits_per_node
        out = PassengerFreeAssignmentRouteColumn[]
        next_id = 1
        for s in sort!(collect(keys(md.passengers_by_scenario)))
            cands = PassengerAssignmentCandidate[]
            for p_id in md.passengers_by_scenario[s]
                for (j, k) in md.feasible_assignments[p_id]
                    push!(cands, PassengerAssignmentCandidate(
                        p_id, j, k, md.ride_limit[(p_id, j, k)], 1.0,
                    ))
                end
            end
            isempty(cands) && continue
            pd = create_passenger_free_assignment_pricing_data(
                s, nodes, md.travel_cost, cands;
                route_regularization_weight=md.route_regularization_weight,
                max_wait_time=md.max_wait_time,
                repositioning_time=md.repositioning_time,
                max_stops=md.max_stops, max_visits_per_node=md.max_visits_per_node,
            )
            seen = Set{Any}()
            route = Int[]; counts = Dict{Int, Int}()
            function dfs!()
                if length(route) >= 2
                    a, tau, _rc = StationSelection._passenger_free_assignment_column_from_route(
                        copy(route), pd,
                    )
                    if !isempty(a)
                        col = PassengerFreeAssignmentRouteColumn(
                            next_id, copy(route), a, tau;
                            metadata=Dict{String, Any}("scenario" => s),
                        )
                        sig = StationSelection._passenger_free_assignment_column_signature(col)
                        if !(sig in seen)
                            push!(seen, sig); push!(out, col); next_id += 1
                        end
                    end
                end
                length(route) >= max_stops && return
                for nd in nodes
                    !isempty(route) && nd == route[end] && continue
                    get(counts, nd, 0) < max_visits || continue
                    push!(route, nd); counts[nd] = get(counts, nd, 0) + 1
                    dfs!()
                    counts[nd] -= 1; pop!(route)
                end
            end
            for st in nodes
                push!(route, st); counts[st] = 1; dfs!(); counts[st] = 0; pop!(route)
            end
        end
        return out
    end

    @testset "dual signs and the Phi = -reduced_cost convention" begin
        data, model = tiny_instance()
        mapping = create_map(model, data)
        md = create_passenger_free_assignment_master_data(model, data, mapping)
        master = build_passenger_free_assignment_master(md, DS_ENV; relax_integrality=true)
        set_silent(master.model)

        all_cols = enumerate_all_columns(md)
        @test !isempty(all_cols)
        for c in all_cols
            add_passenger_free_assignment_column!(master, c)
        end
        optimize!(master.model)
        @test termination_status(master.model) == MOI.OPTIMAL
        z = objective_value(master.model)

        alpha, gamma_o, gamma_d = extract_passenger_free_assignment_duals(master)
        # signs implied by the implemented row senses
        @test all(v -> v >= -1e-7, values(alpha))       # coverage is `>=`
        @test all(v -> v >= -1e-7, values(gamma_o))     # negated `<=` linking duals
        @test all(v -> v >= -1e-7, values(gamma_d))
        # slack variable's own dual constraint
        @test all(v -> v <= md.unserved_penalty + 1e-6, values(alpha))

        walk = (p, j, k) -> md.walk_cost_weight * get(md.assignment_walk_cost, (p, j, k), 0.0)
        for c in all_cols
            beta_c = md.route_regularization_weight * (c.tau + md.repositioning_time)
            phi = route_dual_violation(c.assignments, alpha, gamma_o, gamma_d, walk, beta_c)
            # master-dual-implied reduced cost, computed independently
            rc = passenger_free_assignment_column_cost(c, md)
            for (p, j, k) in c.assignments
                rc -= get(alpha, p, 0.0)
                rc += get(gamma_o, (p, j), 0.0)
                rc += get(gamma_d, (p, k), 0.0)
            end
            @test isapprox(phi, -rc; atol=1e-6)          # Phi = -reduced_cost
            @test phi <= 1e-6                            # all pooled columns dual feasible
        end
        @test z > -Inf
    end

    @testset "selector stays on the optimal face and keeps every RMP column feasible" begin
        data, model = tiny_instance()
        mapping = create_map(model, data)
        md = create_passenger_free_assignment_master_data(model, data, mapping)
        master = build_passenger_free_assignment_master(md, DS_ENV; relax_integrality=true)
        set_silent(master.model)
        for c in enumerate_all_columns(md)
            add_passenger_free_assignment_column!(master, c)
        end
        optimize!(master.model)
        z = objective_value(master.model)

        cfg = PassengerDualSelectorConfig(use_pricing_aware_dual_selection=true)
        selector = build_dual_selector(md, cfg, DS_ENV)
        sync_rmp_columns!(selector, values(master.columns))
        update_optimal_face!(selector, z)

        ref = Dict{Tuple{Int, Int, Int}, Float64}()
        @test solve_dual_selector!(selector, ref) == MOI.OPTIMAL
        a1, u1, v1, eta1, s1, D1 = extract_selected_duals(selector)

        ok, diag = validate_selected_dual(
            selector, a1, u1, v1, eta1, s1, D1, z, values(master.columns),
        )
        @test ok
        @test isapprox(D1, z; atol=1e-4)              # same dual objective
        @test diag.worst_column_violation <= 1e-6     # every RMP column still feasible
        @test diag.worst_station_violation <= 1e-6
        @test diag.worst_sign_violation <= 1e-6
    end

    @testset "repeated selector solves stay feasible as the reference moves" begin
        # Regression: the stabilization rows were once re-added on every call, so a
        # second solve with a different reference stacked contradictory equalities
        # and the model was INFEASIBLE from then on -- silently disabling the feature.
        data, model = tiny_instance()
        mapping = create_map(model, data)
        md = create_passenger_free_assignment_master_data(model, data, mapping)
        master = build_passenger_free_assignment_master(md, DS_ENV; relax_integrality=true)
        set_silent(master.model)
        for c in enumerate_all_columns(md)
            add_passenger_free_assignment_column!(master, c)
        end
        optimize!(master.model)
        z = objective_value(master.model)

        cfg = PassengerDualSelectorConfig(use_pricing_aware_dual_selection=true)
        selector = build_dual_selector(md, cfg, DS_ENV)
        sync_rmp_columns!(selector, values(master.columns))
        update_optimal_face!(selector, z)

        ref = Dict{Tuple{Int, Int, Int}, Float64}()
        @test solve_dual_selector!(selector, ref) == MOI.OPTIMAL
        _a, _u, _v, _e, _s, D_first = extract_selected_duals(selector)

        # move the reference several times; each solve must remain optimal AND on the face
        for shift in (1.0, -2.5, 7.0)
            ref2 = Dict(tr => shift for tr in selector.triples)
            @test solve_dual_selector!(selector, ref2) == MOI.OPTIMAL
            a2, u2, v2, e2, s2, D2 = extract_selected_duals(selector)
            @test isapprox(D2, z; atol=1e-4)
            ok2, _diag2 = validate_selected_dual(
                selector, a2, u2, v2, e2, s2, D2, z, values(master.columns))
            @test ok2
        end
        @test isapprox(D_first, z; atol=1e-4)
    end

    @testset "no false certificate: selector termination checked against full enumeration" begin
        data, model = tiny_instance()
        mapping = create_map(model, data)
        md = create_passenger_free_assignment_master_data(model, data, mapping)
        all_cols = enumerate_all_columns(md)

        # reference: LP over the FULL enumerated column set
        full_master = build_passenger_free_assignment_master(md, DS_ENV; relax_integrality=true)
        set_silent(full_master.model)
        for c in all_cols
            add_passenger_free_assignment_column!(full_master, c)
        end
        optimize!(full_master.model)
        @test termination_status(full_master.model) == MOI.OPTIMAL
        z_full = objective_value(full_master.model)

        # ordinary CG
        plain = run_passenger_free_assignment_column_generation(
            model, data; optimizer_env=DS_ENV, max_cg_iters=500,
            n_candidates=3, max_new_columns=3,
            pricing_time_limit_sec=20.0, certification_time_limit_sec=60.0,
            ip_time_limit_sec=60.0, verbose=false,
        )
        @test plain.cg_stop_reason == :optimality_proven
        @test isapprox(plain.lp_bound, z_full; atol=1e-4)
        @test plain.final_result isa OptResult
        @test plain.final_result.termination_status == plain.mip_termination
        @test plain.final_result.metadata["column_generation_formulation"] ==
            "passenger_free_assignment"

        # Public solver dispatch must select this passenger-level formulation for
        # AggregateODRouteModel + free assignment + column generation, while
        # returning the framework-standard OptResult rather than the CG wrapper.
        dispatch_log_dir = mktempdir()
        dispatched = run_opt(
            data,
            model,
            ColumnGenerationSolver(
                config=SolverConfig(optimizer_env=DS_ENV, silent=true),
                max_iterations=500,
                max_columns_per_iteration=3,
                n_candidates=3,
                pricing_time_limit_sec=20.0,
                final_ip_time_limit_sec=60.0,
                log_dir=dispatch_log_dir,
            ),
        )
        @test dispatched isa OptResult
        @test dispatched.metadata["assignment_policy"] == "FreeAggregateODAssignmentPolicy"
        @test dispatched.metadata["column_generation_formulation"] ==
            "passenger_free_assignment"
        @test isapprox(dispatched.objective_value, plain.final_result.objective_value; atol=1e-4)
        @test dispatched.counts isa ModelCounts
        @test dispatched.counts.variables["theta"] > 0
        @test haskey(JuMP.object_dictionary(dispatched.model), :theta)
        @test haskey(JuMP.object_dictionary(dispatched.model), :v)
        @test haskey(JuMP.object_dictionary(dispatched.model), :x_same)
        @test haskey(JuMP.object_dictionary(dispatched.model), :passenger_coverage)
        @test !isempty(dispatched.metadata["iteration_rows"])
        @test !isempty(dispatched.metadata["column_rows"])
        @test isfile(joinpath(dispatch_log_dir, "passenger_free_assignment_cg_iterations.csv"))
        @test isfile(joinpath(dispatch_log_dir, "passenger_free_assignment_cg_columns.csv"))

        # CG with pricing-aware dual selection
        sel_cfg = PassengerDualSelectorConfig(use_pricing_aware_dual_selection=true)
        selected = run_passenger_free_assignment_column_generation(
            model, data; optimizer_env=DS_ENV, max_cg_iters=500,
            n_candidates=3, max_new_columns=3,
            pricing_time_limit_sec=20.0, certification_time_limit_sec=60.0,
            ip_time_limit_sec=60.0, dual_selector=sel_cfg, verbose=false,
        )
        @test selected.cg_stop_reason == :optimality_proven
        # all three agree: enumerated master == ordinary CG == selector CG
        @test isapprox(selected.lp_bound, z_full; atol=1e-4)
        @test isapprox(selected.lp_bound, plain.lp_bound; atol=1e-4)

        # and the selector actually ran
        @test !isempty(selected.selector_logs)

        # The real no-false-certificate check is the bound comparison above: a
        # certified CG bound that equals the LP over EVERY enumerated column cannot
        # be hiding an improving route. (Both asserted with atol=1e-4 above.)
        #
        # This separate check is narrower and worth stating precisely so it is not
        # mistaken for the certificate check: it verifies the SHARED
        # `route_dual_violation` function against LP optimality on the fully
        # enumerated master. That is true by construction for an optimal LP, so it
        # tests the sign/cost conventions of the shared helper -- not the CG loop.
        full_alpha, full_go, full_gd = extract_passenger_free_assignment_duals(full_master)
        walk = (p, j, k) -> md.walk_cost_weight * get(md.assignment_walk_cost, (p, j, k), 0.0)
        worst_full = -Inf
        for c in all_cols
            beta_c = md.route_regularization_weight * (c.tau + md.repositioning_time)
            worst_full = max(worst_full, route_dual_violation(c.assignments, full_alpha, full_go, full_gd, walk, beta_c))
        end
        @test worst_full <= 1e-6
    end

    @testset "disabled selector leaves ordinary CG untouched" begin
        data, model = tiny_instance()
        a = run_passenger_free_assignment_column_generation(
            model, data; optimizer_env=DS_ENV, max_cg_iters=500,
            n_candidates=3, max_new_columns=3,
            pricing_time_limit_sec=20.0, certification_time_limit_sec=60.0,
            ip_time_limit_sec=60.0, verbose=false,
        )
        b = run_passenger_free_assignment_column_generation(
            model, data; optimizer_env=DS_ENV, max_cg_iters=500,
            n_candidates=3, max_new_columns=3,
            pricing_time_limit_sec=20.0, certification_time_limit_sec=60.0,
            ip_time_limit_sec=60.0,
            dual_selector=PassengerDualSelectorConfig(),  # default: disabled
            verbose=false,
        )
        @test a.cg_stop_reason == b.cg_stop_reason
        @test isapprox(a.lp_bound, b.lp_bound; atol=1e-9)
        @test isempty(a.selector_logs)
        @test isempty(b.selector_logs)
        @test a.selector_iterations_used == 0
        @test b.selector_iterations_used == 0
    end
end
