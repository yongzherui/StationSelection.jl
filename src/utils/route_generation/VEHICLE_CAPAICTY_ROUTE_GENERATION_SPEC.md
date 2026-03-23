Here is a clean DFS pseudocode based on the “covered / salvageable” idea.

## Idea

We build a route left to right, up to length 4.

At each step, we only keep a partial route if every station already on the route satisfies one of these:

- it is **covered**: it already participates in at least one realized allowed assignment pair inside the current route, or
- it is **salvageable**: it could still be the pickup of some allowed assignment pair with a station that may be added later.

If a station is neither covered nor salvageable, we prune the branch immediately.

---

# Definitions

Let:

- (J): set of candidate stations
- (Q_s): requests in scenario (s)
- (\Delta_o^P): allowed pickup stations for origin (o)
- (\Delta_d^D): allowed dropoff stations for destination (d)

Build the allowed assignment-pair set:

[
\mathcal A_s
============

\bigcup\_{(o,d)\in Q_s}
\left(\Delta_o^P \times \Delta_d^D\right)
]

So ((j,k)\in \mathcal A_s) means station (j) can be a pickup and station (k) can be a dropoff for some possible request in scenario (s).

Define the active pickup stations:

[
\mathcal P_s = {j \in J : \exists k \text{ with } (j,k)\in \mathcal A_s}.
]

These are valid starting stations.

Define the active stations:

[
\mathcal V_s
============

{j \in J : \exists k \text{ with } (j,k)\in \mathcal A_s \text{ or } (k,j)\in \mathcal A_s}.
]

---

# Coverage concept

For a partial route

[
r=(j_1,\dots,j_m),
]

station (j_h) is **covered** if there exists some later position (b>h) such that

[
(j_h,j_b)\in \mathcal A_s.
]

That means (j_h) already has a realized pickup-dropoff role in the current route.

Station (j_h) is **salvageable** if it is not covered yet, but there exists some unused station (u) such that

[
(j_h,u)\in \mathcal A_s.
]

That means (j_h) could still be justified later by extending the route.

A branch is pruned if any station in the partial route is neither covered nor salvageable.

---

# DFS pseudocode

```text
Input:
    Station set J
    Scenario requests Q_s
    Pickup sets Δ_o^P
    Dropoff sets Δ_d^D
    Maximum route length L = 4

Output:
    Set of feasible routes R

--------------------------------------------------
Preprocessing
--------------------------------------------------

1. Build allowed assignment pairs:
       A_s = ∅
       for each request (o,d) in Q_s:
           for each pickup station p in Δ_o^P:
               for each dropoff station q in Δ_d^D:
                   add (p,q) to A_s

2. Build active station sets:
       P_s = { j : exists k with (j,k) in A_s }      // valid starting pickups
       V_s = { j : exists k with (j,k) in A_s or (k,j) in A_s }

3. Initialize:
       R = ∅

--------------------------------------------------
Helper routines
--------------------------------------------------

Function CoveredPositions(route r):
    // r = [j1, ..., jm]
    covered = empty set
    m = length(r)

    for h = 1 to m:
        for b = h+1 to m:
            if (r[h], r[b]) in A_s:
                add h to covered
                add b to covered

    return covered


Function IsSalvageable(route r, position h):
    // position h is uncovered; check whether r[h] can still be a pickup later
    station = r[h]

    for each u in V_s:
        if u not in r and (station, u) in A_s:
            return true

    return false


Function IsFeasiblePrefix(route r):
    covered = CoveredPositions(r)
    m = length(r)

    for h = 1 to m:
        if h not in covered:
            if not IsSalvageable(r, h):
                return false

    return true


Function IsFinalFeasible(route r):
    covered = CoveredPositions(r)
    return (covered == {1, 2, ..., length(r)})

--------------------------------------------------
DFS
--------------------------------------------------

Procedure Extend(route r):
    // Step 1: prune bad prefix immediately
    if not IsFeasiblePrefix(r):
        return

    // Step 2: if every station is covered, record route
    if length(r) >= 2 and IsFinalFeasible(r):
        add r to R

    // Step 3: stop at max length
    if length(r) == L:
        return

    // Step 4: try extending to a new station
    for each u in V_s:
        if u in r:
            continue

        // Optional immediate relevance test:
        // u should either close an earlier pickup now,
        // or itself be able to open a pickup for later
        useful_now = false
        for h = 1 to length(r):
            if (r[h], u) in A_s:
                useful_now = true
                break

        useful_later = false
        if not useful_now:
            for each v in V_s:
                if v not in r and v != u and (u, v) in A_s:
                    useful_later = true
                    break

        if not useful_now and not useful_later:
            continue

        Extend(r + [u])

--------------------------------------------------
Main
--------------------------------------------------

for each start station j in P_s:
    Extend([j])

return R
```

---

# What each part is doing

## 1. Start only from valid pickups

We do not start from every active station.
We only start from

[
\mathcal P_s = {j : \exists k,\ (j,k)\in\mathcal A_s},
]

because the first station must be able to act as a pickup for something.

---

## 2. `CoveredPositions(route r)`

This checks which stations in the current route are already justified by realized allowed pairs.

Example: if

[
r=(A,B,C), \qquad \mathcal A_s={(A,C),(B,C)},
]

then all three positions become covered.

---

## 3. `IsSalvageable(route r, position h)`

If a station is not yet covered, this checks whether it could still be saved by a future extension.

Since we build left to right, an existing station can only still be saved by becoming a pickup for some future station.

So we look for an unused (u) such that

[
(r[h],u)\in\mathcal A_s.
]

---

## 4. `IsFeasiblePrefix(route r)`

This is the pruning rule.

A partial route stays alive only if every station is either:

- already covered, or
- still salvageable.

If some station is neither, we immediately kill the branch.

That is the memory-efficient part.

---

## 5. `IsFinalFeasible(route r)`

A route is output only if every station on it is covered.

So in the final kept route, every visited station is justified by at least one realized allowed assignment pair.

This avoids routes like ((A,D,B,C)) when only ((A,C)) and ((B,C)) are allowed, because (D) would remain uncovered.

---

# Small example

Suppose

[
\mathcal A_s={(A,C),(B,C)}.
]

Then:

- (P_s={A,B})
- (V_s={A,B,C})

DFS progression:

### Start with (A)

Route ((A)):

- not covered yet,
- salvageable because ((A,C)\in \mathcal A_s),
- keep exploring.

### Extend to (B)

Route ((A,B)):

- still no realized pair,
- (A) salvageable via future (C),
- (B) salvageable via future (C),
- keep exploring.

### Extend to (C)

Route ((A,B,C)):

- realized pairs: ((A,C)), ((B,C)),
- all stations covered,
- record route.

### Try route ((A,D))

If (D\notin V_s), never generated.
If somehow (D) were considered but had no outgoing pair to any unused station, it would fail salvageability and be pruned immediately.

---

# Why this is memory-smart

This avoids enumerating all (O(n^4)) sequences first.

Instead:

- branches with useless stations die immediately,
- you only keep the current DFS path in memory,
- feasibility is checked at every extension.

So it is much more scalable.

---

# Optional improvement

You can make it faster by updating coverage incrementally instead of recomputing `CoveredPositions(r)` from scratch every time.

That would mean storing with each DFS node:

- the current route,
- the current covered set.

Then when adding a new station (u), you only check new pairs of the form

[
(r[h],u)\in \mathcal A_s.
]

That is a cleaner implementation once the logic is settled.

---

# Compact verbal version

The DFS starts from stations that can serve as valid pickups. It extends a partial route only to stations that either close a previously possible pickup-dropoff assignment or can themselves open a valid assignment to be closed later. After each extension, the algorithm checks whether every station in the partial route is either already justified by a realized allowed pair or still can be justified by a future extension. If not, the branch is pruned immediately. Routes of length at most four are retained only when all visited stations are ultimately justified by realized allowed assignment pairs.
