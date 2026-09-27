@testset "Spiderweb gadget generator + continuous demand points" begin
    net = generate_spiderweb_network()
    K = length(SPIDERWEB_DEFAULT_HALF_WIDTHS)

    @testset "geometry" begin
        cands = spiderweb_candidates(net)
        @test length(cands) == 4K                       # centre is a road node only
        @test length(net.nodes) == 4K + 1
        @test count(e -> e.kind === :ring, net.edges) == 4K
        @test count(e -> e.kind === :spoke, net.edges) == 4K
        sw1 = only(s for s in cands if s.label == "SW1")
        @test (sw1.x, sw1.y) == (0.0, 0.0)
        @test sw1.radial ≈ sqrt(2) * 0.5
        # opposite outer corners: through the centre along the spokes (2 * sqrt(2) * 0.5)
        # beats going round the outer ring (2.0) at equal speeds...
        ne1 = only(s for s in cands if s.label == "NE1")
        @test net.road_time[sw1.id, ne1.id] ≈ sqrt(2)
        # ...and loses to it once spokes are slower than 1/sqrt(2) of ring speed
        slow = generate_spiderweb_network(spoke_speed = 0.5)
        @test slow.road_time[sw1.id, ne1.id] ≈ 2.0
        fast = generate_spiderweb_network(spoke_speed = 2.0)
        @test fast.road_time[sw1.id, ne1.id] ≈ sqrt(2) / 2
        # 90-degree rotational symmetry of the road metric
        rot = Dict(:SW => :SE, :SE => :NE, :NE => :NW, :NW => :SW)
        id(q, l) = only(s.id for s in cands if s.quadrant == q && s.ring == l)
        for a in cands, b in cands
            @test net.road_time[a.id, b.id] ≈ net.road_time[id(rot[a.quadrant], a.ring), id(rot[b.quadrant], b.ring)]
        end
        @test_throws ArgumentError generate_spiderweb_network(half_widths = [0.25, 0.5])
        centre = generate_spiderweb_network(include_center = true)
        @test length(spiderweb_candidates(centre)) == 4K + 1
    end

    reqs = [ContinuousRequest(1, 0.12, 0.16, 0.82, 0.86, "SW->NE"),
            ContinuousRequest(2, 0.14, 0.84, 0.86, 0.17, "NW->SE")]
    inst = SpiderwebInstance(net, reqs)

    @testset "demand generators" begin
        rng = StationSelection.Random.MersenneTwister(1)
        r = sample_spiderweb_requests(rng, :opposite; n = 8)
        @test [x.group for x in r[1:4]] == ["SW->NE", "NE->SW", "NW->SE", "SE->NW"]
        @test all(spiderweb_point_quadrant(x.ox, x.oy) == Symbol(split(x.group, "->")[1]) for x in r)
        c = sample_spiderweb_requests(rng, :crossing; n = 2)
        @test spiderweb_segments_cross(c[1], c[2])
        @test length(perturb_spiderweb_requests(rng, r; n_moved = 2)) == 8
        @test_throws ArgumentError sample_spiderweb_requests(rng, :nope; n = 2)
    end

    @testset "access sets and breakpoints" begin
        O, D = spiderweb_access_sets(inst, 0.1)
        @test O[1] == [only(s.id for s in spiderweb_candidates(net) if s.label == "SW2")]
        bps = spiderweb_walk_breakpoints(inst)
        @test issorted(bps) && length(bps) == length(unique(bps))
        O2, _ = spiderweb_access_sets(inst, bps[end])
        @test all(length(o) == 4K for o in O2)
    end

    data = create_spiderweb_problem_data(inst)
    @testset "StationSelectionData carries exact continuous walking" begin
        @test data.n_stations == 4K + 2 * length(reqs)
        mask = candidate_station_mask(data)
        @test count(mask) == 4K
        @test candidate_station_indices(data) == findall(mask)
        o1 = data.station_id_to_array_idx[spiderweb_origin_id(1)]
        sw2 = data.station_id_to_array_idx[only(s.id for s in spiderweb_candidates(net) if s.label == "SW2")]
        @test get_walking_cost(data, o1, sw2) == hypot(0.12 - 0.125, 0.16 - 0.125)
        d1 = data.station_id_to_array_idx[spiderweb_destination_id(1)]
        @test isinf(get_walking_cost(data, o1, d1)) && isinf(get_walking_cost(data, o1, o1))
        @test isinf(get_routing_cost(data, o1, sw2))
        # no demand point is ever offered as a pickup or dropoff, whatever the radius
        pairs = StationSelection.compute_valid_jk_pairs(Set([(o1, d1)]), data, 10.0)
        @test !isempty(pairs[(o1, d1)])
        @test all(mask[j] && mask[k] && j != k for (j, k) in pairs[(o1, d1)])
        # a data set without the column: everything is a candidate (backward compatible)
        legacy = StationSelectionData(select(data.stations, Not(:candidate)), data.n_stations,
            data.station_id_to_array_idx, data.array_idx_to_station_id, data.walking_costs,
            data.routing_costs, data.scenarios)
        @test all(candidate_station_mask(legacy))
    end

    @testset "non-candidate y is fixed to zero" begin
        m = JuMP.Model()
        StationSelection.add_station_selection_variables!(m, data)
        y = m[:y]
        mask = candidate_station_mask(data)
        @test all(JuMP.is_fixed(y[j]) && JuMP.fix_value(y[j]) == 0.0 for j in findall(!, mask))
        @test !any(JuMP.is_fixed(y[j]) for j in findall(mask))
    end

    m = spiderweb_station_set_metrics(net, [s.id for s in spiderweb_candidates(net) if s.ring == K])
    @test m.bbox_area ≈ 0.25^2 && m.n_rings == 1 && m.n_quadrants == 4 && m.radial_range == 0
end
