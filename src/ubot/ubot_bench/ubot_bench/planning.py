"""Grid shortest paths on the ground-truth map.

Used for (a) generating missions (goal spread, mapping route) and (b) the SPL metric, whose
denominator is the shortest feasible path length from the start to the goal. Both use the same
definition of 'feasible': free cells at least ROBOT_RADIUS from any occupied or unknown cell.
"""
import heapq
import math

import numpy as np

from ubot_bench.grid import FREE, distance_to

# Circumscribed radius of the chassis footprint (0.357 x 0.232 m -> 0.213 m) plus 3.7 cm margin.
ROBOT_RADIUS = 0.25

_NEIGHBOURS = [(-1, -1, math.sqrt(2)), (-1, 0, 1.0), (-1, 1, math.sqrt(2)), (0, -1, 1.0),
               (0, 1, 1.0), (1, -1, math.sqrt(2)), (1, 0, 1.0), (1, 1, math.sqrt(2))]


def clearance_field(grid):
    """Distance (m) from each cell to the nearest non-free cell."""
    return distance_to(grid.data != FREE, grid.res)


def dijkstra(passable, sources, cost=None, targets=None):
    """Geodesic distance (in cells, 8-connected) from a set of source cells.

    passable: bool array; sources: [(r, c)]; cost: optional per-cell multiplier (>= 1);
    targets: optional set of cells, stop when all are settled.
    Returns (dist, parent) where parent is a flat index array (-1 = none).
    """
    h, w = passable.shape
    dist = np.full(h * w, np.inf)
    parent = np.full(h * w, -1, dtype=np.int64)
    pq = []
    for r, c in sources:
        if passable[r, c]:
            i = r * w + c
            dist[i] = 0.0
            heapq.heappush(pq, (0.0, i))
    remaining = set(r * w + c for r, c in targets) if targets else None
    pas = passable.ravel()
    cst = cost.ravel() if cost is not None else None
    while pq:
        d, i = heapq.heappop(pq)
        if d > dist[i]:
            continue
        if remaining is not None:
            remaining.discard(i)
            if not remaining:
                break
        r, c = divmod(i, w)
        for dr, dc, step in _NEIGHBOURS:
            rr, cc = r + dr, c + dc
            if 0 <= rr < h and 0 <= cc < w:
                j = rr * w + cc
                if pas[j]:
                    nd = d + step * (cst[j] if cst is not None else 1.0)
                    if nd < dist[j]:
                        dist[j] = nd
                        parent[j] = i
                        heapq.heappush(pq, (nd, j))
    return dist.reshape(h, w), parent


def path_to(parent, shape, target):
    w = shape[1]
    i = target[0] * w + target[1]
    out = []
    while i != -1:
        out.append(divmod(i, w))
        i = parent[i]
    return out[::-1]


def feasible_mask(grid, radius=ROBOT_RADIUS):
    return clearance_field(grid) >= radius


def nearest_feasible(grid, feasible, x, y, max_m=0.5):
    """Nearest feasible cell to (x, y) within max_m (a start/goal pose can sit a few cm inside
    the inflated band); None if there is none."""
    r, c = grid.world_to_cell(x, y)
    k = int(math.ceil(max_m / grid.res))
    best = None
    for dr in range(-k, k + 1):
        for dc in range(-k, k + 1):
            rr, cc = int(r) + dr, int(c) + dc
            if 0 <= rr < feasible.shape[0] and 0 <= cc < feasible.shape[1] and feasible[rr, cc]:
                d = math.hypot(dr, dc)
                if d * grid.res <= max_m and (best is None or d < best[0]):
                    best = (d, rr, cc)
    return None if best is None else (best[1], best[2])


def shortest_path_length(grid, feasible, start_xy, goal_xy):
    """Length (m) of the shortest feasible path, or inf if the goal is unreachable."""
    s = nearest_feasible(grid, feasible, *start_xy)
    g = nearest_feasible(grid, feasible, *goal_xy)
    if s is None or g is None:
        return math.inf
    dist, _ = dijkstra(feasible, [s], targets=[g])
    return float(dist[g]) * grid.res
