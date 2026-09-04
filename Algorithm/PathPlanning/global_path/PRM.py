from __future__ import annotations

import heapq
import math
import random
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from shapely.geometry import Point, Polygon

from .utils import Point2D, euclidean, is_segment_free, sample_free, shortcut_path, to_shapely_polygons


def plan(
    start: Point2D,
    goal: Point2D,
    polygons_like: Sequence,
    map_size: Tuple[int, int],
    n_samples: int = 600,
    k_neighbors: int = 12,
    max_edge_len: float = 8.0,
    rng_seed: Optional[int] = None,
) -> List[Point2D]:
    rng = random.Random(rng_seed)
    polygons: List[Polygon] = to_shapely_polygons(polygons_like)

    samples: List[Point2D] = []
    # ensure start and goal are free
    def is_free(pt: Point2D) -> bool:
        p = Point(pt[0], pt[1])
        return not any(poly.contains(p) for poly in polygons)

    if not is_free(start) or not is_free(goal):
        return []

    samples.append(start)
    samples.append(goal)
    for _ in range(n_samples):
        pt = sample_free(map_size, polygons, rng)
        samples.append(pt)

    # build kNN edges
    coords = np.array(samples)
    graph: Dict[int, List[Tuple[int, float]]] = {i: [] for i in range(len(samples))}

    for i, p in enumerate(samples):
        dists = np.linalg.norm(coords - coords[i], axis=1)
        order = np.argsort(dists)
        added = 0
        for j in order[1:]:  # skip itself
            if added >= k_neighbors:
                break
            if dists[j] > max_edge_len:
                continue
            if is_segment_free(samples[i], samples[j], polygons):
                w = float(dists[j])
                graph[i].append((j, w))
                graph[j].append((i, w))
                added += 1

    # A*/Dijkstra from start to goal
    start_idx, goal_idx = 0, 1
    dist = [math.inf] * len(samples)
    prev = [-1] * len(samples)
    dist[start_idx] = 0.0
    pq: List[Tuple[float, int]] = [(0.0, start_idx)]

    while pq:
        d_u, u = heapq.heappop(pq)
        if d_u != dist[u]:
            continue
        if u == goal_idx:
            break
        for v, w in graph[u]:
            nd = d_u + w
            if nd + 1e-9 < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    if not math.isfinite(dist[goal_idx]):
        return []

    # reconstruct
    path: List[Point2D] = []
    cur = goal_idx
    while cur != -1:
        path.append(samples[cur])
        cur = prev[cur]
    path.reverse()

    # shortcut smoothing
    path = shortcut_path(path, polygons, trials=80)
    return path


# Backward-compatible wrapper: match A_star/RRT signature
def PRM(
    start: Point2D,
    goal: Point2D,
    terrain_polygons,
    max_iter: int = 5000,  # unused, for compatibility
    step_size: float = 1.0,  # unused, for compatibility
    goal_sample_rate: int = 5,  # unused
):
    polys = to_shapely_polygons(terrain_polygons)
    xs = [start[0], goal[0]] + [xy[0] for p in polys for xy in p.exterior.coords]
    ys = [start[1], goal[1]] + [xy[1] for p in polys for xy in p.exterior.coords]
    width = int(math.ceil(max(xs))) + 1
    height = int(math.ceil(max(ys))) + 1
    return plan(
        start=start,
        goal=goal,
        polygons_like=polys,
        map_size=(width, height),
        n_samples=600,
        k_neighbors=12,
        max_edge_len=max(4.0, 4.0 * float(step_size)),
        rng_seed=None,
    )


