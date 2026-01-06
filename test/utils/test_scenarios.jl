using StationSelection
using Test
using .GenerateScenarios
using Dates

@testset "GenerateScenarios Tests" begin

    # --- Test 1: 1-hour segments for 2 days ---
    @testset "1-hour segments" begin
        scenarios = GenerateScenarios.generate_scenarios(Date(2025,6,1), Date(2025,6,2); segment_hours=1)
        @test length(scenarios) == 48  # 24*2 hours
        # Check first segment
        @test scenarios[1] == ("2025-06-01 00:00:00", "2025-06-01 00:59:59")
        # Check last segment
        @test scenarios[end] == ("2025-06-02 23:00:00", "2025-06-02 23:59:59")
    end

    # --- Test 2: 6-hour segments for 3 days ---
    @testset "6-hour segments" begin
        scenarios = GenerateScenarios.generate_scenarios(Date(2025,6,1), Date(2025,6,3); segment_hours=6)
        @test length(scenarios) == 12  # 4 segments per day * 3 days
        # Check first segment
        @test scenarios[1] == ("2025-06-01 00:00:00", "2025-06-01 05:59:59")
        # Check last segment
        @test scenarios[end] == ("2025-06-03 18:00:00", "2025-06-03 23:59:59")
    end

    # --- Test 3: 24-hour segments with weekly cycle ---
    @testset "weekly cycle 24-hour" begin
        scenarios = GenerateScenarios.generate_scenarios(Date(2025,6,1), Date(2025,6,30); segment_hours=24, weekly_cycle=true)
        # 1st, 8th, 15th, 22nd, 29th of June 2025
        expected_dates = [
            "2025-06-01 00:00:00", "2025-06-08 00:00:00", "2025-06-15 00:00:00",
            "2025-06-22 00:00:00", "2025-06-29 00:00:00"
        ]
        @test length(scenarios) == length(expected_dates)
        @test all(scenarios[i][1] == expected_dates[i] for i in 1:length(expected_dates))
    end

    # --- Test 4: 4-hour segments with weekly cycle ---
    @testset "weekly cycle 4-hour" begin
        scenarios = GenerateScenarios.generate_scenarios(Date(2025,6,1), Date(2025,6,14); segment_hours=4, weekly_cycle=true)
        # Only segments starting on same weekday as June 1 (Sunday)
        @test all(dayofweek(Date(DateTime(s[1], "yyyy-mm-dd HH:MM:SS"))) == dayofweek(Date(2025,6,1)) for s in scenarios)
        @test !isempty(scenarios)
    end

end
