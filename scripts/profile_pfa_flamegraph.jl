"""
    scripts/profile_pfa_flamegraph.jl

Run the passenger free-assignment pricers under Julia's sampling profiler and the
allocation profiler, and write a **self-contained interactive flame graph** plus
the raw folded stacks.

This exists because the PFA pricers' own `PROFILE` counters
(`bench_passenger_free_assignment_labels.jl`) only time four hand-instrumented
regions -- dominance, queue, candidates, extension. That is enough for the
revisit-tolerant pricer, where dominance is ~90% of wall, but for the
station-simple pricer those four regions add up to under 20% of wall and the
counters cannot say where the rest goes. A flame graph attributes 100% of the
samples by construction.

No new package dependency: the profile is folded and rendered here (the ecosystem
renderers -- ProfileSVG, PProf, StatProfilerHTML -- are not in this project's
Manifest, and adding them to run on a cluster node is more moving parts than the
~200 lines below).

# Outputs (into `--out`, default `tmp_ss_bench/pfa_profile_<date>/`)

  - `<case>.html`   -- interactive flame graph: click a frame to zoom, hover for
                       exact sample counts, plus the self-time table and the
                       allocation table. Open it directly in a browser; it embeds
                       everything and loads nothing from the network.
  - `<case>.folded` -- Brendan-Gregg collapsed stacks (`a;b;c count`), for
                       diffing two runs or feeding any other flame-graph tool.
  - stdout          -- `SELFTIME` and `ALLOC` rows, so a run is greppable from a
                       SLURM log without downloading the HTML.

# Usage

    julia --project=. scripts/profile_pfa_flamegraph.jl \\
        [--cases 15:6,20:5] [--pricer both|revisit|station_simple] \\
        [--out DIR] [--label NAME] [--no-allocs]

`--label` tags the output filenames (e.g. `--label baseline` vs `--label micro`)
so two variants can be rendered side by side.
"""

using Printf, Profile, JSON, StationSelection

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = 16
const SEED = 42
const N_SCENARIOS = 3
const MAX_WALK = 600.0
const ROUTE_REG_WEIGHT = 10.0
const MAX_WAIT_TIME = 900.0
const REPOSITIONING_TIME = 20.0
const DETOUR_FACTOR = 2.0
const WALK_COST_WEIGHT = 0.1
const BASE_VALUE = 5000.0
const MAX_VISITS_PER_NODE = 3
const N_CANDIDATES = 1_000_000_000
const TIME_LIMIT_SEC = parse(Float64, get(ENV, "PFA_TIME_LIMIT", "900"))

# Frames below this share of total samples are folded away before rendering: a
# flame graph with tens of thousands of one-sample slivers is unreadable and
# enormous, and nothing under a tenth of a percent is actionable anyway.
const MIN_FRAME_FRACTION = 0.001

# ---------------------------------------------------------------------------
# instance construction (same knobs as bench_passenger_free_assignment_labels.jl,
# so profile and benchmark describe the same workload)
# ---------------------------------------------------------------------------

function build_scenario_candidates(data::StationSelectionData, n_stations::Int, s::Int)
    candidates = PassengerAssignmentCandidate[]
    requests = data.scenarios[s].requests
    for row in eachrow(requests)
        o, d = row.origin_idx, row.dest_idx
        for j in 1:n_stations
            walk_o = get_walking_cost(data, o, j)
            walk_o <= MAX_WALK || continue
            for k in 1:n_stations
                k == j && continue
                walk_d = get_walking_cost(data, d, k)
                walk_d <= MAX_WALK || continue
                reward = BASE_VALUE - WALK_COST_WEIGHT * (walk_o + walk_d)
                reward > 0 || continue
                ride_limit = DETOUR_FACTOR * get_routing_cost(data, j, k)
                push!(candidates, PassengerAssignmentCandidate(row.id, j, k, ride_limit, reward))
            end
        end
    end
    return candidates
end

function build_pricing_data(n_stations::Int, raw_max_stops::Int, scenario::Int)
    max_stops = raw_max_stops <= 0 ? typemax(Int) : raw_max_stops
    data, _meta = generate_zhuzhou_data(
        DATA_DIR, n_stations, N_PAIRS; n_scenarios=N_SCENARIOS, seed=SEED,
    )
    nodes = collect(1:n_stations)
    travel_cost = Dict{Tuple{Int, Int}, Float64}()
    for i in nodes, j in nodes
        i == j && continue
        travel_cost[(i, j)] = get_routing_cost(data, i, j)
    end
    candidates = build_scenario_candidates(data, n_stations, scenario)
    return create_passenger_free_assignment_pricing_data(
        scenario, nodes, travel_cost, candidates;
        route_regularization_weight=ROUTE_REG_WEIGHT,
        max_wait_time=MAX_WAIT_TIME,
        repositioning_time=REPOSITIONING_TIME,
        max_stops=max_stops,
        max_visits_per_node=MAX_VISITS_PER_NODE,
    )
end

price_revisit(pd) = passenger_free_assignment_pricing_by_label_setting(
    pd, PassengerFreeAssignmentRouteColumn[];
    next_column_id=1, max_new_columns=N_CANDIDATES, n_candidates=N_CANDIDATES,
    time_limit=TIME_LIMIT_SEC,
)

price_station_simple(pd) = passenger_free_assignment_pricing_by_station_simple_label_setting(
    pd, PassengerFreeAssignmentRouteColumn[];
    next_column_id=1, max_new_columns=N_CANDIDATES, n_candidates=N_CANDIDATES,
    time_limit=TIME_LIMIT_SEC,
)

# ---------------------------------------------------------------------------
# profile -> folded stacks
# ---------------------------------------------------------------------------

"""
Which bucket a stack frame belongs to -- what the flame graph colours by. Three
buckets only, and they are chosen so that *any* two can end up adjacent in the
graph and still be told apart; the frame's own name is drawn on it as well, so
colour never carries the identity alone.

Decided from the frame's **full** path, not its basename: `data.jl`, `types.jl`
and `search.jl` are names half the ecosystem uses.
"""
function frame_category(file::String)::Int
    occursin("aggregate_od_route/pricing", file) && return 1  # the pricer itself
    return 2
end

"""
Turn one raw profile into `stack => sample_count`, where a stack is a root-to-leaf
vector of `"func (file:line)"` frames.

`Profile.fetch` hands back instruction pointers leaf-first with a `0` between
samples; each pointer expands to several `StackFrame`s when the compiler inlined
through it, innermost first. Both orders are reversed here so the folded stack
reads root to leaf, which is the direction a flame graph is drawn in.

C frames are dropped (they are runtime internals with no source location worth
showing) *except* garbage collection, which is kept and collapsed to a single
`GC` frame -- allocation pressure is one of the things this profile is meant to
expose, and dropping it would silently redistribute its time onto whatever Julia
frame happened to trigger the collection.

**Idle worker threads are dropped too.** The sampler fires on every thread, and
this is a single-threaded search, so the idle threads park in the scheduler
(`poptask`) and contribute a sample each per tick -- exactly 50% of the profile on
a 2-thread job, which then rescales every real frame by an arbitrary factor of the
thread count. Any stack ending in the scheduler's park loop is discarded, and
`idle_samples` reports how many, so the drop is visible rather than silent.
"""
const IDLE_FRAMES = ("poptask", "wait", "task_done_hook", "jfptr_poptask")
function fold_profile(data::Vector{UInt64}, lidict)
    folded = Dict{Vector{String}, Int}()
    # Frame name => colour bucket, decided here where the full path is still in
    # hand; the rendered name only carries the basename.
    categories = Dict{String, Int}("GC" => 3)
    stack = String[]
    n_samples = 0
    gc_samples = 0
    idle_samples = 0
    # `data` is leaf-first per sample; collect one sample at a time, then reverse.
    sample_start = 1
    for i in eachindex(data)
        data[i] == 0 || continue
        empty!(stack)
        saw_gc = false
        for j in (i - 1):-1:sample_start   # reversed: root first
            frames = get(lidict, data[j], nothing)
            frames === nothing && continue
            for k in length(frames):-1:1   # reversed: outermost inlined frame first
                sf = frames[k]
                file = String(sf.file)
                name = String(sf.func)
                if sf.from_c
                    if occursin("gc", lowercase(name))
                        saw_gc = true
                    end
                    continue
                end
                frame = string(name, " (", basename(file), ":", sf.line, ")")
                get!(categories, frame, frame_category(file))
                push!(stack, frame)
            end
        end
        saw_gc && push!(stack, "GC")
        if isempty(stack)
            sample_start = i + 1
            continue
        end
        # An idle worker parked in the scheduler is not work this search did.
        if any(f -> startswith(stack[end], f * " ("), IDLE_FRAMES)
            idle_samples += 1
            sample_start = i + 1
            continue
        end
        key = copy(stack)
        folded[key] = get(folded, key, 0) + 1
        n_samples += 1
        saw_gc && (gc_samples += 1)
        sample_start = i + 1
    end
    return folded, categories, n_samples, gc_samples, idle_samples
end

mutable struct FlameNode
    name::String
    value::Int
    children::Dict{String, FlameNode}
end
FlameNode(name::String) = FlameNode(name, 0, Dict{String, FlameNode}())

function build_tree(folded::Dict{Vector{String}, Int}, total::Int)
    root = FlameNode("all")
    root.value = total
    threshold = max(1, floor(Int, MIN_FRAME_FRACTION * total))
    for (stack, count) in folded
        node = root
        for frame in stack
            child = get(node.children, frame, nothing)
            if child === nothing
                child = FlameNode(frame)
                node.children[frame] = child
            end
            child.value += count
            node = child
        end
    end
    prune!(root, threshold)
    return root
end

function prune!(node::FlameNode, threshold::Int)
    for (name, child) in collect(node.children)
        if child.value < threshold
            delete!(node.children, name)
        else
            prune!(child, threshold)
        end
    end
    return node
end

function node_to_dict(node::FlameNode, categories::Dict{String, Int})
    children = sort!(collect(values(node.children)); by=c -> (-c.value, c.name))
    return Dict(
        "n" => node.name,
        "v" => node.value,
        "c" => [node_to_dict(c, categories) for c in children],
        "g" => get(categories, node.name, 2),
    )
end

"""
Self time per source line: the leaf frame of each sample. This is the table that
actually says which line to optimise -- the flame graph says which *path* is
expensive, which is not the same question.
"""
function self_times(folded::Dict{Vector{String}, Int})
    self = Dict{String, Int}()
    for (stack, count) in folded
        isempty(stack) && continue
        leaf = stack[end]
        self[leaf] = get(self, leaf, 0) + count
    end
    return sort!(collect(self); by=p -> -p[2])
end

# ---------------------------------------------------------------------------
# allocation profile
# ---------------------------------------------------------------------------

"""
Top allocation sites by total bytes, from `Profile.Allocs`.

Sampled (`sample_rate` below 1) because recording every allocation of a search
that makes tens of millions of them is slower than the search itself; the ranking
is what matters here, not the absolute totals.
"""
function alloc_report(; top::Int=25)::Vector{Pair{String, Tuple{Int, Int}}}
    results = Profile.Allocs.fetch()
    by_site = Dict{String, Tuple{Int, Int}}()  # site => (count, bytes)
    for alloc in results.allocs
        site = "unknown"
        for sf in alloc.stacktrace
            sf.from_c && continue
            file = basename(String(sf.file))
            site = string(sf.func, " (", file, ":", sf.line, ")")
            break
        end
        n, b = get(by_site, site, (0, 0))
        by_site[site] = (n + 1, b + alloc.size)
    end
    rows = sort!(collect(by_site); by=p -> -p[2][2])
    return rows[1:min(top, length(rows))]
end

# ---------------------------------------------------------------------------
# HTML rendering
# ---------------------------------------------------------------------------

const HTML_HEAD = raw"""
<style>
  :root {
    color-scheme: light;
    --surface-1: #fcfcfb;
    --page: #f9f9f7;
    --text-primary: #0b0b0b;
    --text-secondary: #52514e;
    --muted: #898781;
    --grid: #e1e0d9;
    --border: rgba(11, 11, 11, 0.10);
    --cat-1: #2a78d6;
    --cat-2: #eb6834;
    --cat-3: #1baf7a;
  }
  @media (prefers-color-scheme: dark) {
    :root:where(:not([data-theme="light"])) {
      color-scheme: dark;
      --surface-1: #1a1a19;
      --page: #0d0d0d;
      --text-primary: #ffffff;
      --text-secondary: #c3c2b7;
      --muted: #898781;
      --grid: #2c2c2a;
      --border: rgba(255, 255, 255, 0.10);
      --cat-1: #3987e5;
      --cat-2: #d95926;
      --cat-3: #199e70;
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --surface-1: #1a1a19;
    --page: #0d0d0d;
    --text-primary: #ffffff;
    --text-secondary: #c3c2b7;
    --muted: #898781;
    --grid: #2c2c2a;
    --border: rgba(255, 255, 255, 0.10);
    --cat-1: #3987e5;
    --cat-2: #d95926;
    --cat-3: #199e70;
  }
  body {
    margin: 0;
    padding: 24px 20px 64px;
    background: var(--page);
    color: var(--text-primary);
    font: 14px/1.5 system-ui, -apple-system, "Segoe UI", sans-serif;
  }
  .wrap { max-width: 1180px; margin: 0 auto; }
  h1 { font-size: 20px; margin: 0 0 4px; font-weight: 650; letter-spacing: -0.01em; }
  h2 { font-size: 15px; margin: 32px 0 10px; font-weight: 600; }
  .sub { color: var(--text-secondary); margin: 0 0 20px; font-size: 13px; }
  .card {
    background: var(--surface-1);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 16px;
  }
  .tiles { display: flex; flex-wrap: wrap; gap: 10px; margin-bottom: 18px; }
  .tile {
    background: var(--surface-1); border: 1px solid var(--border);
    border-radius: 10px; padding: 10px 14px; min-width: 120px;
  }
  .tile .k { color: var(--muted); font-size: 11px; text-transform: uppercase; letter-spacing: 0.05em; }
  .tile .v { font-size: 20px; font-weight: 620; margin-top: 2px; }
  .controls { display: flex; flex-wrap: wrap; gap: 12px; align-items: center; margin-bottom: 12px; }
  .controls input {
    background: var(--page); color: var(--text-primary);
    border: 1px solid var(--border); border-radius: 7px;
    padding: 6px 10px; font: inherit; font-size: 13px; min-width: 220px;
  }
  .controls button {
    background: var(--page); color: var(--text-primary);
    border: 1px solid var(--border); border-radius: 7px;
    padding: 6px 12px; font: inherit; font-size: 13px; cursor: pointer;
  }
  .legend { display: flex; flex-wrap: wrap; gap: 16px; font-size: 12px; color: var(--text-secondary); }
  .legend span { display: inline-flex; align-items: center; gap: 6px; }
  .swatch { width: 11px; height: 11px; border-radius: 3px; display: inline-block; }
  #flame { overflow-x: auto; }
  #flameInner { position: relative; min-width: 100%; }
  .frame {
    position: absolute; height: 19px; overflow: hidden;
    border-radius: 3px; cursor: pointer;
    font-size: 11px; line-height: 19px; white-space: nowrap;
    padding-left: 4px; box-sizing: border-box;
    color: #fff;
  }
  .frame.dim { opacity: 0.25; }
  #tip {
    position: fixed; pointer-events: none; z-index: 20; display: none;
    background: var(--surface-1); color: var(--text-primary);
    border: 1px solid var(--border); border-radius: 8px;
    padding: 8px 10px; font-size: 12px; max-width: 460px;
    box-shadow: 0 6px 22px rgba(0, 0, 0, 0.18);
  }
  #tip .tn { font-weight: 600; word-break: break-all; }
  #tip .tv { color: var(--text-secondary); margin-top: 4px; }
  table { border-collapse: collapse; width: 100%; font-size: 12.5px; }
  th, td { text-align: left; padding: 6px 10px; border-bottom: 1px solid var(--grid); }
  th { color: var(--muted); font-weight: 600; font-size: 11px; text-transform: uppercase; letter-spacing: 0.05em; }
  td.num, th.num { text-align: right; font-variant-numeric: tabular-nums; }
  td.name { word-break: break-all; }
  .bar { height: 8px; border-radius: 4px; background: var(--cat-1); display: block; }
  .barcell { width: 130px; }
  .crumb { font-size: 12px; color: var(--text-secondary); margin-bottom: 8px; min-height: 18px; word-break: break-all; }
</style>
"""

const HTML_SCRIPT = raw"""
<script>
(function () {
  const CAT = ["", "--cat-1", "--cat-2", "--cat-3"];
  const el = document.getElementById("flameInner");
  const tip = document.getElementById("tip");
  const crumb = document.getElementById("crumb");
  const search = document.getElementById("search");
  const ROW = 20;
  let focus = DATA;
  let query = "";

  const pct = (v) => ((100 * v) / DATA.v).toFixed(2) + "%";
  const ms = (v) => (v * PERIOD_MS).toFixed(0) + " ms";

  function depth(node) {
    let d = 1;
    for (const c of node.c) d = Math.max(d, 1 + depth(c));
    return d;
  }

  function render() {
    el.innerHTML = "";
    const h = depth(focus) * ROW;
    el.style.height = h + "px";
    const width = el.clientWidth || 900;
    const q = query.toLowerCase();
    const draw = (node, x0, w, row) => {
      if (w < 0.4) return;
      const d = document.createElement("div");
      d.className = "frame" + (q && !node.n.toLowerCase().includes(q) ? " dim" : "");
      d.style.left = x0 + "px";
      // 2px surface gap so adjacent fills never fuse into one block.
      d.style.width = Math.max(1, w - 2) + "px";
      d.style.top = row * ROW + "px";
      d.style.background = "var(" + (CAT[node.g] || "--cat-2") + ")";
      if (w > 34) d.textContent = node.n.replace(/ \(.*/, "");
      d.addEventListener("mousemove", (e) => {
        tip.style.display = "block";
        tip.innerHTML = '<div class="tn"></div><div class="tv"></div>';
        tip.querySelector(".tn").textContent = node.n;
        tip.querySelector(".tv").textContent =
          node.v + " samples · " + pct(node.v) + " of total · ~" + ms(node.v);
        const pad = 14;
        let left = e.clientX + pad;
        if (left + tip.offsetWidth > window.innerWidth - 8)
          left = e.clientX - tip.offsetWidth - pad;
        tip.style.left = left + "px";
        tip.style.top = Math.min(e.clientY + pad, window.innerHeight - tip.offsetHeight - 8) + "px";
      });
      d.addEventListener("mouseleave", () => { tip.style.display = "none"; });
      d.addEventListener("click", () => { focus = node; crumb.textContent = node.n; render(); });
      el.appendChild(d);
      let cx = x0;
      for (const c of node.c) {
        const cw = (c.v / node.v) * w;
        draw(c, cx, cw, row + 1);
        cx += cw;
      }
    };
    draw(focus, 0, width, 0);
  }

  document.getElementById("reset").addEventListener("click", () => {
    focus = DATA; crumb.textContent = ""; render();
  });
  search.addEventListener("input", (e) => { query = e.target.value.trim(); render(); });
  window.addEventListener("resize", render);
  render();
})();
</script>
"""

function esc(s::AbstractString)
    return replace(s, "&" => "&amp;", "<" => "&lt;", ">" => "&gt;")
end

function render_html(;
    title::String,
    subtitle::String,
    tiles::Vector{Pair{String, String}},
    tree::Dict,
    period_ms::Float64,
    self_rows::Vector,
    total_samples::Int,
    alloc_rows::Vector,
    alloc_rate::Float64,
)
    io = IOBuffer()
    println(io, "<title>", esc(title), "</title>")
    println(io, HTML_HEAD)
    println(io, "<div class=\"wrap\">")
    println(io, "<h1>", esc(title), "</h1>")
    println(io, "<p class=\"sub\">", esc(subtitle), "</p>")

    println(io, "<div class=\"tiles\">")
    for (k, v) in tiles
        println(io, "<div class=\"tile\"><div class=\"k\">", esc(k),
                "</div><div class=\"v\">", esc(v), "</div></div>")
    end
    println(io, "</div>")

    println(io, "<h2>Flame graph</h2>")
    println(io, "<div class=\"card\">")
    println(io, """
      <div class="controls">
        <input id="search" type="text" placeholder="Highlight frames matching…" />
        <button id="reset">Reset zoom</button>
        <div class="legend">
          <span><i class="swatch" style="background:var(--cat-1)"></i>PFA pricer</span>
          <span><i class="swatch" style="background:var(--cat-2)"></i>Julia Base / runtime</span>
          <span><i class="swatch" style="background:var(--cat-3)"></i>GC</span>
        </div>
      </div>
      <div class="crumb" id="crumb"></div>
      <div id="flame"><div id="flameInner"></div></div>
    """)
    println(io, "</div>")

    println(io, "<h2>Self time by source line</h2>")
    println(io, "<div class=\"card\"><table>")
    println(io, "<thead><tr><th>Frame</th><th class=\"num\">Samples</th>",
            "<th class=\"num\">Share</th><th></th></tr></thead><tbody>")
    top_self = self_rows[1:min(30, length(self_rows))]
    max_self = isempty(top_self) ? 1 : top_self[1][2]
    for (name, count) in top_self
        share = 100 * count / max(total_samples, 1)
        w = round(Int, 100 * count / max_self)
        println(io, "<tr><td class=\"name\">", esc(name), "</td>",
                "<td class=\"num\">", count, "</td>",
                @sprintf("<td class=\"num\">%.2f%%</td>", share),
                "<td class=\"barcell\"><i class=\"bar\" style=\"width:", w, "%\"></i></td></tr>")
    end
    println(io, "</tbody></table></div>")

    if !isempty(alloc_rows)
        println(io, "<h2>Allocation sites</h2>")
        println(io, "<p class=\"sub\">Sampled at rate ", @sprintf("%.3f", alloc_rate),
                "; ranking is meaningful, absolute totals are scaled.</p>")
        println(io, "<div class=\"card\"><table>")
        println(io, "<thead><tr><th>Site</th><th class=\"num\">Allocations</th>",
                "<th class=\"num\">Bytes</th><th></th></tr></thead><tbody>")
        max_bytes = maximum(r[2][2] for r in alloc_rows)
        for (site, (n, bytes)) in alloc_rows
            w = round(Int, 100 * bytes / max(max_bytes, 1))
            println(io, "<tr><td class=\"name\">", esc(site), "</td>",
                    "<td class=\"num\">", n, "</td>",
                    "<td class=\"num\">", bytes, "</td>",
                    "<td class=\"barcell\"><i class=\"bar\" style=\"width:", w, "%\"></i></td></tr>")
        end
        println(io, "</tbody></table></div>")
    end

    println(io, "</div><div id=\"tip\"></div>")
    println(io, "<script>const DATA = ", JSON.json(tree), ";")
    println(io, "const PERIOD_MS = ", period_ms, ";</script>")
    println(io, HTML_SCRIPT)
    return String(take!(io))
end

# ---------------------------------------------------------------------------
# driver
# ---------------------------------------------------------------------------

function profile_case(pricer::String, n::Int, ms::Int, scenario::Int, outdir::String, label::String;
                      do_allocs::Bool=true)
    pd = build_pricing_data(n, ms, scenario)
    run = pricer == "revisit" ? price_revisit : price_station_simple

    run(pd)  # warm up: compile everything before the sampler starts

    Profile.clear()
    # 1ms sampling with room for a long run; the buffer is sized for the whole
    # search rather than relying on the default, which overflows on the big cases.
    Profile.init(; n=40_000_000, delay=0.001)
    t0 = time()
    columns, exhausted, stats = @profile run(pd)
    wall = time() - t0

    data = Profile.fetch(; include_meta=false)
    lidict = Profile.getdict(data)
    folded, categories, n_samples, gc_samples, idle_samples = fold_profile(data, lidict)
    tree = node_to_dict(build_tree(folded, n_samples), categories)
    self_rows = self_times(folded)

    alloc_rows = Pair{String, Tuple{Int, Int}}[]
    # `AllocResults` does not report the rate back, so it is tracked here.
    alloc_rate = 0.002
    if do_allocs
        Profile.Allocs.clear()
        Profile.Allocs.@profile sample_rate = alloc_rate run(pd)
        alloc_rows = alloc_report()
        Profile.Allocs.clear()
    end

    case = "$(label)_$(pricer)_n$(n)_ms$(ms)_s$(scenario)"
    mkpath(outdir)

    open(joinpath(outdir, case * ".folded"), "w") do io
        for (stack, count) in sort!(collect(folded); by=p -> -p[2])
            println(io, join(stack, ";"), " ", count)
        end
    end

    best_rc = isempty(columns) ? NaN : columns[1].metadata["reduced_cost"]
    html = render_html(;
        title = "PFA $(pricer) pricer — n=$(n), max_stops=$(ms), scenario $(scenario) [$(label)]",
        subtitle = "Sampling profile of one exhaustive pricing pass. " *
                   "Click any frame to zoom into it; the search box dims non-matching frames.",
        tiles = [
            "Wall" => @sprintf("%.2f s", wall),
            "Samples" => string(n_samples),
            "GC" => @sprintf("%.1f%%", 100 * gc_samples / max(n_samples, 1)),
            "Idle dropped" => string(idle_samples),
            "Labels" => string(stats.labels_generated),
            "Max live" => string(stats.max_live_labels),
            "Columns" => string(length(columns)),
            "Best rc" => @sprintf("%.3f", best_rc),
        ],
        tree = tree,
        period_ms = 1.0,
        self_rows = self_rows,
        total_samples = n_samples,
        alloc_rows = alloc_rows,
        alloc_rate = alloc_rate,
    )
    write(joinpath(outdir, case * ".html"), html)

    @printf("PROFILED\tcase=%s\twall=%.3f\tsamples=%d\tgc_samples=%d\tidle_dropped=%d\tlabels=%d\tbest_rc=%.6f\n",
            case, wall, n_samples, gc_samples, idle_samples, stats.labels_generated, best_rc)
    for (name, count) in self_rows[1:min(20, length(self_rows))]
        @printf("SELFTIME\tcase=%s\tshare=%.4f\tsamples=%d\tframe=%s\n",
                case, count / max(n_samples, 1), count, name)
    end
    for (site, (cnt, bytes)) in alloc_rows[1:min(15, length(alloc_rows))]
        @printf("ALLOC\tcase=%s\tbytes=%d\tcount=%d\tsite=%s\n", case, bytes, cnt, site)
    end
    flush(stdout)
    return nothing
end

function main()
    cases = [(15, 6, 1), (20, 5, 3)]
    pricers = ["revisit", "station_simple"]
    outdir = joinpath(@__DIR__, "..", "tmp_ss_bench", "pfa_profile")
    label = "run"
    do_allocs = true

    i = 1
    while i <= length(ARGS)
        if ARGS[i] == "--cases"
            cases = Tuple{Int, Int, Int}[]
            for spec in split(ARGS[i + 1], ",")
                parts = split(spec, ":")
                push!(cases, (parse(Int, parts[1]), parse(Int, parts[2]),
                              length(parts) >= 3 ? parse(Int, parts[3]) : 1))
            end
            i += 2
        elseif ARGS[i] == "--pricer"
            pricers = ARGS[i + 1] == "both" ? ["revisit", "station_simple"] : [ARGS[i + 1]]
            i += 2
        elseif ARGS[i] == "--out"
            outdir = ARGS[i + 1]; i += 2
        elseif ARGS[i] == "--label"
            label = ARGS[i + 1]; i += 2
        elseif ARGS[i] == "--no-allocs"
            do_allocs = false; i += 1
        else
            error("unknown argument $(ARGS[i])")
        end
    end

    outdir = normpath(outdir)
    println("# profile_pfa_flamegraph label=$(label) cases=$(cases) pricers=$(pricers) out=$(outdir)")
    for (n, ms, s) in cases, pricer in pricers
        profile_case(pricer, n, ms, s, outdir, label; do_allocs=do_allocs)
    end
    println("# wrote HTML flame graphs to $(outdir)")
end

main()
