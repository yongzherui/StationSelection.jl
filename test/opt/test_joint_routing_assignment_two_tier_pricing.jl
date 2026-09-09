@testset "Two-tier relaxed-cluster certification" begin
    SS = StationSelection

    # Stations on a line so cluster membership and every distance is checkable by hand.
    function line_travel_cost(nodes)
        costs = Dict{Tuple{Int, Int}, Float64}()
        for i in nodes, j in nodes
            i == j && continue
            costs[(i, j)] = Float64(abs(i - j))
        end
        return costs
    end

    # Six stations in three well-separated pairs, so K2=3 has one sensible answer and K1=2
    # must merge exactly two of those pairs.
    pair_nodes() = [1, 2, 11, 12, 21, 22]

    @testset "_nested_macro_clustering nests by construction" begin
        nodes = pair_nodes()
        tc = line_travel_cost(nodes)
        meso = cluster_stations_by_travel_cost(nodes, tc, 3)
        macro_cl, parent = SS._nested_macro_clustering(meso, 2, tc)

        @test macro_cl.n_clusters == 2
        @test length(parent) == meso.n_clusters
        # Every station appears exactly once in the macro partition, and over the same set.
        @test sort!(reduce(vcat, macro_cl.members)) == sort(nodes)
        @test sort(collect(keys(macro_cl.cluster_of))) == sort(nodes)
        # THE nesting property: each macro cell is exactly the union of the meso cells
        # whose parent it is. This is what the whole bound chain rests on.
        for g in 1:macro_cl.n_clusters
            expected = Int[]
            for c in 1:meso.n_clusters
                parent[c] == g && append!(expected, meso.members[c])
            end
            @test macro_cl.members[g] == sort(expected)
            @test !isempty(expected)
        end
        # And it is consistent with cluster_of both ways.
        for c in 1:meso.n_clusters, st in meso.members[c]
            @test macro_cl.cluster_of[st] == parent[c]
        end

        # A macro layer at or above the meso count is no coarsening at all.
        @test_throws ArgumentError SS._nested_macro_clustering(meso, 3, tc)
        @test_throws ArgumentError SS._nested_macro_clustering(meso, 4, tc)
        @test_throws ArgumentError SS._nested_macro_clustering(meso, 0, tc)
    end

    @testset "_two_tier_restrict renumbers without losing membership" begin
        nodes = pair_nodes()
        tc = line_travel_cost(nodes)
        meso = cluster_stations_by_travel_cost(nodes, tc, 3)
        keep = [1, 3]
        r = SS._two_tier_restrict(meso, keep)

        @test r.n_clusters == 2
        @test r.members[1] == meso.members[1]
        @test r.members[2] == meso.members[3]
        @test r.nodes == sort(vcat(meso.members[1], meso.members[3]))
        for (i, c) in enumerate(keep), st in meso.members[c]
            @test r.cluster_of[st] == i
        end
        # Stations of the dropped cell must not be addressable in the restricted layer.
        for st in meso.members[2]
            @test !haskey(r.cluster_of, st)
        end
        @test_throws ArgumentError SS._two_tier_restrict(meso, Int[])
    end

    @testset "_two_tier_local_cut_sets translates global cuts into local indices" begin
        keep = [2, 5, 7]
        # Only the members that exist locally survive; a cut is "visit a cell OUTSIDE this
        # set", so dropping absent members is what keeps it satisfiable locally.
        @test SS._two_tier_local_cut_sets([Set([5])], keep) == [Set([2])]
        @test SS._two_tier_local_cut_sets([Set([2, 7])], keep) == [Set([1, 3])]
        @test SS._two_tier_local_cut_sets([Set([3, 4])], keep) == [Set{Int}()]
        # A cut swallowing every local cell translates to the full local set, which no
        # local route can escape -- correct, since the region is already proved barren.
        @test SS._two_tier_local_cut_sets([Set([2, 5, 7, 9])], keep) == [Set([1, 2, 3])]
        @test SS._two_tier_local_cut_sets(Set{Int}[], keep) == Set{Int}[]
    end

    @testset "_two_tier_aligned_support rounds up only within its station budget" begin
        nodes = pair_nodes()
        tc = line_travel_cost(nodes)
        meso = cluster_stations_by_travel_cost(nodes, tc, 3)
        macro_cl, parent = SS._nested_macro_clustering(meso, 2, tc)

        # Pick a meso cell whose macro parent owns more than just it, so alignment has
        # something to add.
        merged_parent = findfirst(g -> count(==(g), parent) > 1, 1:macro_cl.n_clusters)
        @test merged_parent !== nothing
        inside = findall(==(merged_parent), parent)
        support = Set([first(inside)])

        # Generous budget: alignment applies and returns the macro cells to cut.
        cells, stations, macro_cells, used =
            SS._two_tier_aligned_support([support], parent, meso, 100)
        @test macro_cells == Set([merged_parent])
        @test sort(cells) == sort(inside)
        @test used == 1
        # THE alignment identity: the priced stations are exactly the macro cells' stations.
        @test stations == macro_cl.members[merged_parent]

        # A budget below the aligned size falls back to the bare support: still a valid
        # meso cut, but no macro cut is licensed.
        tight = length(meso.members[first(inside)])
        cells2, stations2, macro_cells2, used2 =
            SS._two_tier_aligned_support([support], parent, meso, tight)
        @test macro_cells2 === nothing
        @test cells2 == [first(inside)]
        @test stations2 == meso.members[first(inside)]
        @test length(stations2) <= tight
        @test used2 == 0
    end

    @testset "_two_tier_aligned_support shrinks the guide prefix to keep the macro cut" begin
        nodes = pair_nodes()
        tc = line_travel_cost(nodes)
        meso = cluster_stations_by_travel_cost(nodes, tc, 3)
        macro_cl, parent = SS._nested_macro_clustering(meso, 2, tc)

        # Guide 1 lives inside ONE macro cell; guide 2 reaches into the other. Their union
        # therefore aligns to every macro cell, while the prefix of length 1 aligns to one.
        g1 = Set([1])
        other = findfirst(c -> parent[c] != parent[1], 1:meso.n_clusters)
        @test other !== nothing
        g2 = Set([other])

        aligned_one = SS.relaxed_cluster_station_subset(
            meso, [[c for c in 1:meso.n_clusters if parent[c] == parent[1]]])
        # A cap that admits guide 1's aligned set but not the union's must NOT give up the
        # macro cut: it must drop guide 2 and still price aligned. This is the fix for the
        # 53-82% alignment-refusal rate measured at n=40.
        cells, stations, macro_cells, used =
            SS._two_tier_aligned_support([g1, g2], parent, meso, length(aligned_one))
        @test used == 1
        @test macro_cells == Set([parent[1]])
        @test stations == aligned_one

        # A cap that admits the union keeps every guide.
        all_st = SS.relaxed_cluster_station_subset(meso, [collect(1:meso.n_clusters)])
        _, _, macro_all, used_all =
            SS._two_tier_aligned_support([g1, g2], parent, meso, length(all_st))
        @test used_all == 2
        @test macro_all == Set([parent[1], parent[other]])
    end

    @testset "CGPricingConfig validation for the two-tier mode" begin
        # K1 is required, and required to be a real coarsening.
        @test_throws ArgumentError CGPricingConfig(
            mode = :relaxed_cluster_two_tier, relaxed_cluster_count = 6,
        )
        @test_throws ArgumentError CGPricingConfig(
            mode = :relaxed_cluster_two_tier, relaxed_cluster_count = 6,
            relaxed_cluster_macro_count = 6,
        )
        @test_throws ArgumentError CGPricingConfig(
            mode = :relaxed_cluster_two_tier, relaxed_cluster_count = 6,
            relaxed_cluster_macro_count = 7,
        )
        @test_throws ArgumentError CGPricingConfig(
            mode = :relaxed_cluster_two_tier, relaxed_cluster_count = 6,
            relaxed_cluster_macro_count = 0,
        )
        # The mode needs a meso partition too.
        @test_throws ArgumentError CGPricingConfig(
            mode = :relaxed_cluster_two_tier, relaxed_cluster_macro_count = 3,
        )
        # A macro count nothing reads is rejected rather than silently ignored.
        @test_throws ArgumentError CGPricingConfig(
            mode = :relaxed_cluster, relaxed_cluster_count = 6,
            relaxed_cluster_macro_count = 3,
        )
        @test_throws ArgumentError CGPricingConfig(relaxed_cluster_macro_count = 3)
        # Refinement re-partitions the meso layer and would strand the parent map.
        @test_throws ArgumentError CGPricingConfig(
            mode = :relaxed_cluster_two_tier, relaxed_cluster_count = 6,
            relaxed_cluster_macro_count = 3, relaxed_cluster_max_count = 8,
        )
        # Same no-handoff argument as :relaxed_cluster.
        @test_throws ArgumentError CGPricingConfig(
            warm_start_mode = :relaxed_cluster_two_tier, relaxed_cluster_count = 6,
        )
        @test_throws ArgumentError CGPricingConfig(
            mode = :relaxed_cluster_two_tier, relaxed_cluster_count = 6,
            relaxed_cluster_macro_count = 3, relaxed_cluster_aligned_subset_max = 0,
        )

        ok = CGPricingConfig(
            mode = :relaxed_cluster_two_tier, relaxed_cluster_count = 6,
            relaxed_cluster_macro_count = 4,
        )
        @test ok.relaxed_cluster_macro_count == 4
        # 15, lowered from 20 after the n=40 stall: station-search cost is super-linear
        # and the seeds that certified had subset medians of 8-10 stations.
        @test ok.relaxed_cluster_aligned_subset_max == 15
        @test CGPricingConfig().relaxed_cluster_macro_count === nothing
    end

    @testset "end to end: same certified optimum as the one-tier mode" begin
        instance = generate_middle_zone_benchmark_instance("balanced", 1, 1, 1)
        data = create_middle_zone_station_selection_data(instance; max_walking_distance = 800.0)
        problem = StationSelectionProblem(data, 5; max_walking_distance = 800.0)
        formulation() = AggregateODRouteJointRoutingAssignmentFormulation(max_stops = 4)

        # `recover_integer_solution` is what makes the LP/IP split exist at all
        # (`cg_lp_objective_value` is only written on that path), and it is how Study 9
        # runs these arms -- so both arms ask for it and the comparison covers both values.
        one_tier = run_opt(problem, formulation(), CGSolver(
            pricing = CGPricingConfig(mode = :relaxed_cluster, relaxed_cluster_count = 6),
            recover_integer_solution = true,
        ))
        two_tier = run_opt(problem, formulation(), CGSolver(
            pricing = CGPricingConfig(
                mode = :relaxed_cluster_two_tier, relaxed_cluster_count = 6,
                relaxed_cluster_macro_count = 3,
            ),
            recover_integer_solution = true,
        ))

        # A mode is a search algorithm: both arms solve the identical model, so a certified
        # optimum from either must be the same number.
        @test two_tier.termination_status == one_tier.termination_status
        @test two_tier.objective_value ≈ one_tier.objective_value atol = 1e-6
        @test two_tier.metadata["cg_lp_objective_value"] ≈
            one_tier.metadata["cg_lp_objective_value"] atol = 1e-6
        # And the certificate it reports is the same KIND of certificate.
        @test two_tier.metadata["cg_final_pricing_mode"] === :relaxed_cluster_two_tier
        @test two_tier.metadata["cg_optimality_scope"] == "full_route_universe"
        @test two_tier.metadata["cg_pricing_universe_restricted"] === false
        if two_tier.termination_status == SOLVE_OPTIMAL
            @test two_tier.metadata["cg_certified_by_relaxation"] === true
            @test two_tier.metadata["cg_stop_reason"] == "converged_by_certification"
        end
        # The stat rows must show the loop really ran on both layers.
        stats = two_tier.metadata["cg_relaxed_cluster_guide_stats"]
        @test !isempty(stats)
        @test any(r -> :macro in r.two_tier_tier_trace, stats)
        @test all(r -> r.subset_size <= 20, stats)
    end

    @testset "a two-tier mode without a macro layer is not silently supported" begin
        instance = generate_middle_zone_benchmark_instance("balanced", 1, 1, 1)
        data = create_middle_zone_station_selection_data(instance; max_walking_distance = 800.0)
        problem = StationSelectionProblem(data, 5; max_walking_distance = 800.0)
        # Built for the ONE-tier mode, so no macro layer exists on the model.
        br = build_model(
            problem,
            AggregateODRouteJointRoutingAssignmentFormulation(max_stops = 4),
            CGSolver(pricing = CGPricingConfig(
                mode = :relaxed_cluster, relaxed_cluster_count = 6,
            )),
        )
        m = br.model
        @test !haskey(m.obj_dict, :joint_routing_assignment_macro_clustering)
        @test SS.cg_certification_supported(br, br.mapping, m) === true
        # Ask that same model for the two-tier round: support must be withdrawn, which is
        # what makes CGSolver reject the mode up front instead of running a half-wired loop.
        m[:joint_routing_assignment_pricing_mode] = :relaxed_cluster_two_tier
        @test SS.cg_certification_supported(br, br.mapping, m) === false
    end

    @testset "the build stashes a macro layer only when asked" begin
        instance = generate_middle_zone_benchmark_instance("balanced", 1, 1, 1)
        data = create_middle_zone_station_selection_data(instance; max_walking_distance = 800.0)
        problem = StationSelectionProblem(data, 5; max_walking_distance = 800.0)
        br = build_model(
            problem,
            AggregateODRouteJointRoutingAssignmentFormulation(max_stops = 4),
            CGSolver(pricing = CGPricingConfig(
                mode = :relaxed_cluster_two_tier, relaxed_cluster_count = 6,
                relaxed_cluster_macro_count = 3,
            )),
        )
        m = br.model
        meso = m[:joint_routing_assignment_station_clustering]
        macro_cl = m[:joint_routing_assignment_macro_clustering]
        parent = m[:joint_routing_assignment_macro_parent]
        @test meso.n_clusters == 6
        @test macro_cl.n_clusters == 3
        @test length(parent) == 6
        # The stashed pair must satisfy the nesting property the loop assumes.
        for g in 1:macro_cl.n_clusters
            expected = sort!(reduce(vcat,
                [meso.members[c] for c in 1:meso.n_clusters if parent[c] == g]; init = Int[]))
            @test macro_cl.members[g] == expected
        end
        @test br.counts isa ModelCounts
    end
end
