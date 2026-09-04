from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

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


@dataclass
class GAConfig:
    population_size: int = 80
    num_generations: int = 120
    num_waypoints: int = 8  # interior points between start and goal
    mutation_rate: float = 0.25
    crossover_rate: float = 0.8
    tournament_k: int = 3
    rng_seed: Optional[int] = None


def _bounds_from_scene(start: Point2D, goal: Point2D, polygons: List[Polygon]) -> Tuple[int, int]:
    xs = [start[0], goal[0]] + [xy[0] for p in polygons for xy in p.exterior.coords]
    ys = [start[1], goal[1]] + [xy[1] for p in polygons for xy in p.exterior.coords]
    width = int(math.ceil(max(xs))) + 1
    height = int(math.ceil(max(ys))) + 1
    return width, height


def _is_point_free(pt: Point2D, polygons: Sequence[Polygon]) -> bool:
    p = Point(pt[0], pt[1])
    return not any(poly.contains(p) for poly in polygons)


def _evaluate_path(path: List[Point2D], polygons: Sequence[Polygon]) -> float:
    """Lower is better. Includes large penalties for collisions and detours.

    - Sum of segment lengths
    - + collision_penalty per intersecting segment
    - + slight waypoint penalty to prefer simpler paths
    """
    if not path or len(path) < 2:
        return float("inf")

    total = 0.0
    collision_penalty = 1e4
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        total += euclidean(a, b)
        if not is_segment_free(a, b, polygons):
            total += collision_penalty

    total += 0.2 * float(len(path))  # prefer fewer turns
    return total


def _biased_sample(rng: random.Random, start: Point2D, goal: Point2D, map_size: Tuple[int, int], polygons: List[Polygon]) -> Point2D:
    """Sample a point with a bias towards the straight-line corridor between start and goal."""
    width, height = map_size
    if rng.random() < 0.6:  # biased corridor sampling
        t = rng.random()
        # point near the line segment with gaussian offset
        base_x = start[0] * (1 - t) + goal[0] * t
        base_y = start[1] * (1 - t) + goal[1] * t
        x = float(np.clip(rng.gauss(base_x, max(1.0, 0.04 * width)), 0.0, float(width)))
        y = float(np.clip(rng.gauss(base_y, max(1.0, 0.04 * height)), 0.0, float(height)))
        if _is_point_free((x, y), polygons):
            return (x, y)
    # fallback to uniform free sampling
    return sample_free(map_size, polygons, rng)


def plan(
    start: Point2D,
    goal: Point2D,
    polygons_like: Sequence,
    map_size: Tuple[int, int],
    config: Optional[GAConfig] = None,
) -> List[Point2D]:
    polygons: List[Polygon] = to_shapely_polygons(polygons_like)
    if config is None:
        config = GAConfig()
    rng = random.Random(config.rng_seed)

    if not _is_point_free(start, polygons) or not _is_point_free(goal, polygons):
        return []

    # Individual = array of interior waypoints (num_waypoints x 2)
    def init_individual() -> np.ndarray:
        pts = [
            _biased_sample(rng, start, goal, map_size, polygons)
            for _ in range(config.num_waypoints)
        ]
        return np.asarray(pts, dtype=float)

    def decode(ind: np.ndarray) -> List[Point2D]:
        return [start] + [(float(x), float(y)) for x, y in ind.tolist()] + [goal]

    def mutate(ind: np.ndarray) -> None:
        for i in range(len(ind)):
            if rng.random() < config.mutation_rate:
                if rng.random() < 0.6:
                    # resample
                    ind[i] = _biased_sample(rng, start, goal, map_size, polygons)
                else:
                    # small jitter
                    ind[i, 0] += rng.gauss(0.0, 0.8)
                    ind[i, 1] += rng.gauss(0.0, 0.8)

    def crossover(a: np.ndarray, b: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if rng.random() > config.crossover_rate:
            return a.copy(), b.copy()
        cut = rng.randrange(1, config.num_waypoints)
        c1 = np.vstack([a[:cut], b[cut:]])
        c2 = np.vstack([b[:cut], a[cut:]])
        return c1, c2

    def tournament(pop: List[np.ndarray], k: int) -> int:
        cand_idxs = rng.sample(range(len(pop)), k)
        best_idx = cand_idxs[0]
        best_fit = _evaluate_path(decode(pop[best_idx]), polygons)
        for idx in cand_idxs[1:]:
            f = _evaluate_path(decode(pop[idx]), polygons)
            if f < best_fit:
                best_idx, best_fit = idx, f
        return best_idx

    pop: List[np.ndarray] = [init_individual() for _ in range(config.population_size)]
    best_path: List[Point2D] = []
    best_cost: float = float("inf")

    for _ in range(config.num_generations):
        # Evaluate and keep global best
        for ind in pop:
            path = decode(ind)
            cost = _evaluate_path(path, polygons)
            if cost < best_cost:
                best_cost = cost
                best_path = path

        # Generate next population
        next_pop: List[np.ndarray] = []
        while len(next_pop) < config.population_size:
            p1 = pop[tournament(pop, config.tournament_k)]
            p2 = pop[tournament(pop, config.tournament_k)]
            c1, c2 = crossover(p1, p2)
            mutate(c1)
            mutate(c2)
            next_pop.append(c1)
            if len(next_pop) < config.population_size:
                next_pop.append(c2)
        pop = next_pop

    # Post-process: remove intersecting segments by shortcutting and return
    if not best_path:
        return []
    best_path = shortcut_path(best_path, polygons, trials=120)
    # Ensure final path is collision-free; if not, try to prune offending points
    cleaned: List[Point2D] = [best_path[0]]
    for pt in best_path[1:]:
        if is_segment_free(cleaned[-1], pt, polygons):
            cleaned.append(pt)
        else:
            # attempt to insert one detour sample
            detour = _biased_sample(rng, cleaned[-1], pt, map_size, polygons)
            if is_segment_free(cleaned[-1], detour, polygons) and is_segment_free(detour, pt, polygons):
                cleaned.extend([detour, pt])
            else:
                # give up and skip
                cleaned.append(pt)
    return cleaned


# Backward-compatible wrapper matching other planners' signature
def GA(
    start: Point2D,
    goal: Point2D,
    terrain_polygons,
    max_iter: int = 5000,  # unused - kept for compatibility
    step_size: float = 1.0,  # used indirectly to set map bounds
    goal_sample_rate: int = 5,  # unused
):
    polys = to_shapely_polygons(terrain_polygons)
    width, height = _bounds_from_scene(start, goal, polys)
    # Configure GA; allow a few more generations for larger scenes
    approx_d = euclidean(start, goal)
    gens = 120 if approx_d < 100 else 180
    cfg = GAConfig(num_generations=gens, num_waypoints=max(6, int(approx_d / (6.0 * float(step_size) + 1e-6))))
    return plan(start, goal, polys, (width, height), cfg)


