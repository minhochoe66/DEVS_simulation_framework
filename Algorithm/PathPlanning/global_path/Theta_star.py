from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from shapely.geometry import Point, Polygon

from .utils import Point2D, euclidean, is_segment_free, to_shapely_polygons


def plan(
    start: Point2D,
    goal: Point2D,
    polygons_like: Sequence,
    map_size: Tuple[int, int],
    grid_resolution: float = 1.0,
) -> List[Point2D]:
    """Theta*: grid-based any-angle pathfinding with line-of-sight checks against polygons.

    We build a grid at given resolution, run Theta* using 8-connected neighbors, and then output
    continuous coordinates.
    """
    width, height = map_size
    cols = int(math.ceil(width / grid_resolution))
    rows = int(math.ceil(height / grid_resolution))
    polygons: List[Polygon] = to_shapely_polygons(polygons_like)

    def to_world(ix: int, iy: int) -> Point2D:
        return (min((ix + 0.5) * grid_resolution, width - 1e-6), min((iy + 0.5) * grid_resolution, height - 1e-6))

    def to_grid(p: Point2D) -> Tuple[int, int]:
        ix = int(min(max(p[0] / grid_resolution, 0), cols - 1))
        iy = int(min(max(p[1] / grid_resolution, 0), rows - 1))
        return ix, iy

    def cell_is_free(ix: int, iy: int) -> bool:
        p = to_world(ix, iy)
        point = Point(p[0], p[1])
        return not any(poly.contains(point) for poly in polygons)

    def neighbors(ix: int, iy: int):
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                jx, jy = ix + dx, iy + dy
                if 0 <= jx < cols and 0 <= jy < rows:
                    yield jx, jy

    def line_free_cells(a: Tuple[int, int], b: Tuple[int, int]) -> bool:
        pa = to_world(*a)
        pb = to_world(*b)
        return is_segment_free(pa, pb, polygons)

    s_ix, s_iy = to_grid(start)
    g_ix, g_iy = to_grid(goal)

    start_node = (s_ix, s_iy)
    goal_node = (g_ix, g_iy)

    # 시작/목표가 장애물 내부인 경우 경로 없음
    if not cell_is_free(*start_node) or not cell_is_free(*goal_node):
        return []

    g: Dict[Tuple[int, int], float] = {start_node: 0.0}
    parent: Dict[Tuple[int, int], Tuple[int, int]] = {start_node: start_node}
    f: Dict[Tuple[int, int], float] = {start_node: euclidean(to_world(*start_node), to_world(*goal_node))}
    open_pq: List[Tuple[float, Tuple[int, int]]] = [(f[start_node], start_node)]

    closed = set()

    while open_pq:
        _, u = heapq.heappop(open_pq)
        if u in closed:
            continue
        closed.add(u)
        if u == goal_node:
            break

        for v in neighbors(*u):
            if v in closed:
                continue
            # 이웃 셀 자체가 장애물 내부면 스킵
            if not cell_is_free(*v):
                continue

            # Theta*: 부모에서 직시 가능하면 부모를 통해 완화
            if parent[u] != u and line_free_cells(parent[u], v):
                # path from parent[u] to v
                cand_g = g[parent[u]] + euclidean(to_world(*parent[u]), to_world(*v))
                if cand_g + 1e-9 < g.get(v, math.inf):
                    g[v] = cand_g
                    parent[v] = parent[u]
                    f[v] = g[v] + euclidean(to_world(*v), to_world(*goal_node))
                    heapq.heappush(open_pq, (f[v], v))
            else:
                # 부모에서 직시 불가능하면 u->v 간 선분 충돌도 확인
                if not line_free_cells(u, v):
                    continue
                cand_g = g[u] + euclidean(to_world(*u), to_world(*v))
                if cand_g + 1e-9 < g.get(v, math.inf):
                    g[v] = cand_g
                    parent[v] = u
                    f[v] = g[v] + euclidean(to_world(*v), to_world(*goal_node))
                    heapq.heappush(open_pq, (f[v], v))

    if goal_node not in parent:
        return []

    # reconstruct
    path_cells: List[Tuple[int, int]] = []
    cur = goal_node
    while True:
        path_cells.append(cur)
        if cur == parent[cur]:
            break
        cur = parent[cur]
    path_cells.reverse()

    # convert to world coordinates and simplify consecutive duplicates
    path: List[Point2D] = []
    last: Optional[Point2D] = None
    for c in path_cells:
        p = to_world(*c)
        if last is None or (abs(p[0] - last[0]) > 1e-6 or abs(p[1] - last[1]) > 1e-6):
            path.append(p)
            last = p
    return path


# Backward-compatible wrapper: match A_star/RRT signature
def Theta_star(
    start: Point2D,
    goal: Point2D,
    terrain_polygons,
    max_iter: int = 5000,  # unused
    step_size: float = 1.0,  # use as grid resolution
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
        grid_resolution=max(0.5, float(step_size)),
    )

