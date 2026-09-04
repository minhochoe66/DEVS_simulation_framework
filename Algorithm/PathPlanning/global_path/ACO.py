from __future__ import annotations

import math
import random
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from shapely.geometry import Point, Polygon

from .utils import (
    Point2D,
    euclidean,
    is_segment_free,
    sample_free,
    shortcut_path,
    to_shapely_polygons,
)


class ACOConfig:
    def __init__(
        self,
        n_samples: int = 300,
        k_neighbors: int = 12,
        max_edge_len: float = 8.0,
        n_ants: int = 40,
        n_iters: int = 80,
        alpha: float = 1.0,  # pheromone influence
        beta: float = 2.0,  # heuristic (1/length) influence
        rho: float = 0.35,  # evaporation rate
        q: float = 60.0,  # pheromone deposit scaling
        rng_seed: Optional[int] = None,
    ) -> None:
        self.n_samples = n_samples
        self.k_neighbors = k_neighbors
        self.max_edge_len = max_edge_len
        self.n_ants = n_ants
        self.n_iters = n_iters
        self.alpha = alpha
        self.beta = beta
        self.rho = rho
        self.q = q
        self.rng_seed = rng_seed


def _bounds_from_scene(start: Point2D, goal: Point2D, polygons: List[Polygon]) -> Tuple[int, int]:
    xs = [start[0], goal[0]] + [xy[0] for p in polygons for xy in p.exterior.coords]
    ys = [start[1], goal[1]] + [xy[1] for p in polygons for xy in p.exterior.coords]
    width = int(math.ceil(max(xs))) + 1
    height = int(math.ceil(max(ys))) + 1
    return width, height


def _is_point_free(pt: Point2D, polygons: Sequence[Polygon]) -> bool:
    p = Point(pt[0], pt[1])
    return not any(poly.contains(p) for poly in polygons)


def plan(
    start: Point2D,
    goal: Point2D,
    polygons_like: Sequence,
    map_size: Tuple[int, int],
    config: Optional[ACOConfig] = None,
) -> List[Point2D]:
    rng = random.Random(None if config is None else config.rng_seed)
    polygons: List[Polygon] = to_shapely_polygons(polygons_like)

    if not _is_point_free(start, polygons) or not _is_point_free(goal, polygons):
        return []

    # 1) Sample collision-free milestone points (like PRM nodes)
    samples: List[Point2D] = [start, goal]
    for _ in range((config.n_samples if config else 300)):
        samples.append(sample_free(map_size, polygons, rng))

    # 2) Build neighbor graph with collision-free edges
    coords = np.array(samples)
    graph: Dict[int, List[Tuple[int, float]]] = {i: [] for i in range(len(samples))}
    k_neighbors = (config.k_neighbors if config else 12)
    max_edge_len = (config.max_edge_len if config else 8.0)

    for i in range(len(samples)):
        dists = np.linalg.norm(coords - coords[i], axis=1)
        order = np.argsort(dists)
        added = 0
        for j in order[1:]:
            if added >= k_neighbors:
                break
            if dists[j] > max_edge_len:
                continue
            if is_segment_free(samples[i], samples[j], polygons):
                w = float(dists[j])
                graph[i].append((j, w))
                graph[j].append((i, w))
                added += 1

    start_idx, goal_idx = 0, 1
    n_nodes = len(samples)
    # 3) Initialize pheromone on each undirected edge
    tau: Dict[Tuple[int, int], float] = {}
    for u, edges in graph.items():
        for v, _ in edges:
            key = (min(u, v), max(u, v))
            tau[key] = tau.get(key, 1.0)

    def choose_next(u: int, visited: set) -> Optional[int]:
        choices = []
        for v, w in graph[u]:
            if v in visited:
                continue
            key = (min(u, v), max(u, v))
            pher = tau.get(key, 1e-6)
            heuristic = 1.0 / max(w, 1e-6)
            alpha = config.alpha if config else 1.0
            beta = config.beta if config else 2.0
            score = (pher ** alpha) * (heuristic ** beta)
            choices.append((v, w, max(score, 1e-12)))
        if not choices:
            return None
        scores = np.array([c[2] for c in choices], dtype=float)
        probs = scores / scores.sum()
        idx = int(np.random.choice(len(choices), p=probs))
        return choices[idx][0]

    best_cost = float("inf")
    best_path: List[int] = []
    n_iters = (config.n_iters if config else 80)
    n_ants = (config.n_ants if config else 40)
    rho = (config.rho if config else 0.35)
    Q = (config.q if config else 60.0)

    for _ in range(n_iters):
        ant_paths: List[List[int]] = []
        ant_costs: List[float] = []

        for _ in range(n_ants):
            visited = {start_idx}
            path = [start_idx]
            cur = start_idx
            fail_guard = 0
            while cur != goal_idx and fail_guard < n_nodes:
                nxt = choose_next(cur, visited)
                if nxt is None:
                    break
                path.append(nxt)
                visited.add(nxt)
                cur = nxt
                fail_guard += 1

            # compute cost if reached goal
            if path and path[-1] == goal_idx:
                cost = 0.0
                for i in range(len(path) - 1):
                    u, v = path[i], path[i + 1]
                    # find weight
                    w = next(w for (vv, w) in graph[u] if vv == v)
                    cost += w
                ant_paths.append(path)
                ant_costs.append(cost)
                if cost < best_cost:
                    best_cost, best_path = cost, path

        # evaporate
        for k in list(tau.keys()):
            tau[k] *= (1.0 - rho)

        # deposit from ants
        for path, cost in zip(ant_paths, ant_costs):
            deposit = Q / max(cost, 1e-6)
            for i in range(len(path) - 1):
                u, v = path[i], path[i + 1]
                key = (min(u, v), max(u, v))
                tau[key] = tau.get(key, 0.0) + deposit

    if not best_path:
        return []

    # Reconstruct coordinate path
    pts: List[Point2D] = [samples[idx] for idx in best_path]
    pts = shortcut_path(pts, polygons, trials=100)
    return pts


def ACO(
    start: Point2D,
    goal: Point2D,
    terrain_polygons,
    max_iter: int = 5000,  # unused compatibility
    step_size: float = 1.0,  # used to scale max edge length slightly
    goal_sample_rate: int = 5,  # unused
):
    polys = to_shapely_polygons(terrain_polygons)
    width, height = _bounds_from_scene(start, goal, polys)
    approx_d = euclidean(start, goal)
    cfg = ACOConfig(
        n_samples=int(350 + 0.8 * approx_d),
        k_neighbors=14,
        max_edge_len=max(6.0, 6.0 * float(step_size)),
        n_ants=50,
        n_iters=90,
        alpha=1.0,
        beta=2.0,
        rho=0.35,
        q=70.0,
    )
    return plan(start, goal, polys, (width, height), cfg)


