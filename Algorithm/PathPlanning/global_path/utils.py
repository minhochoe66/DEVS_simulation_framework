"""
Utility functions for global path planning algorithms
"""
from __future__ import annotations

import math
import random
from typing import List, Sequence, Tuple, Union

from shapely.geometry import LineString, Point, Polygon

# Type alias for 2D points
Point2D = Tuple[float, float]


def euclidean(p1: Point2D, p2: Point2D) -> float:
    """Calculate Euclidean distance between two points"""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def to_shapely_polygons(polygons_like: Sequence) -> List[Polygon]:
    """
    Convert various polygon representations to shapely Polygons
    
    Args:
        polygons_like: List of polygon representations (can be shapely Polygons, 
                      list of points, or dict with 'position' and 'boundingBox')
    
    Returns:
        List of shapely Polygon objects
    """
    result = []
    for item in polygons_like:
        if isinstance(item, Polygon):
            result.append(item)
        elif isinstance(item, dict):
            # Handle bounding box format: {'position': {'x': ..., 'y': ...}, 'boundingBox': {'width': ..., 'height': ...}}
            if 'position' in item and 'boundingBox' in item:
                pos = item['position']
                bbox = item['boundingBox']
                x, y = pos['x'], pos['y']
                w, h = bbox['width'], bbox['height']
                # Create rectangle centered at (x, y)
                coords = [
                    (x - w/2, y - h/2),
                    (x + w/2, y - h/2),
                    (x + w/2, y + h/2),
                    (x - w/2, y + h/2),
                ]
                result.append(Polygon(coords))
            else:
                # Assume it's a list of coordinate dicts
                coords = [(p['x'], p['y']) for p in item] if isinstance(item[0], dict) else item
                result.append(Polygon(coords))
        elif isinstance(item, (list, tuple)):
            # List of (x, y) tuples or Point2D
            result.append(Polygon(item))
        else:
            raise ValueError(f"Cannot convert {type(item)} to Polygon")
    return result


def is_segment_free(p1: Point2D, p2: Point2D, polygons: List[Polygon]) -> bool:
    """
    Check if line segment from p1 to p2 is collision-free
    
    Args:
        p1: Start point
        p2: End point
        polygons: List of obstacle polygons
    
    Returns:
        True if segment is free of collisions
    """
    line = LineString([p1, p2])
    for poly in polygons:
        if line.intersects(poly):
            return False
    return True


def sample_free(
    map_size: Tuple[int, int],
    polygons: List[Polygon],
    rng: random.Random = None
) -> Point2D:
    """
    Sample a random collision-free point in the map
    
    Args:
        map_size: (width, height) of the map
        polygons: List of obstacle polygons
        rng: Random number generator (optional)
    
    Returns:
        Random free point (x, y)
    """
    if rng is None:
        rng = random.Random()
    
    max_attempts = 1000
    for _ in range(max_attempts):
        x = rng.uniform(0, map_size[0])
        y = rng.uniform(0, map_size[1])
        pt = Point(x, y)
        
        # Check if point is free
        is_free = True
        for poly in polygons:
            if poly.contains(pt):
                is_free = False
                break
        
        if is_free:
            return (x, y)
    
    # Fallback: return center of map
    return (map_size[0] / 2, map_size[1] / 2)


def shortcut_path(
    path: List[Point2D],
    polygons: List[Polygon],
    trials: int = 50
) -> List[Point2D]:
    """
    Smooth path by attempting shortcuts between non-adjacent waypoints
    
    Args:
        path: Original path as list of points
        polygons: List of obstacle polygons
        trials: Number of shortcut attempts
    
    Returns:
        Smoothed path
    """
    if len(path) <= 2:
        return path
    
    result = list(path)
    
    for _ in range(trials):
        if len(result) <= 2:
            break
        
        # Pick two random indices
        i = random.randint(0, len(result) - 1)
        j = random.randint(0, len(result) - 1)
        
        if abs(i - j) <= 1:
            continue
        
        if i > j:
            i, j = j, i
        
        # Try to shortcut from i to j
        if is_segment_free(result[i], result[j], polygons):
            # Remove intermediate points
            result = result[:i+1] + result[j:]
    
    return result

