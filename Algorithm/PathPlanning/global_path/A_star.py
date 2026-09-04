import numpy as np
from shapely.geometry import Point, LineString, Polygon
import heapq
from collections import defaultdict

def A_star(start, goal, terrain_polygons, max_iter=5000, step_size=1, goal_sample_rate=5):
    """A* global path planning.

    Returns a path from start to goal avoiding terrain_polygons, as [(x, y), ...].
    If no path is found, returns the path to the node closest to the goal.
    max_iter, step_size and goal_sample_rate are unused; they are kept so that
    every global planner shares one call signature.
    """
    
    def heuristic(a, b):
        """Manhattan-distance heuristic."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def euclidean_distance(a, b):
        """Euclidean distance."""
        return np.linalg.norm(np.array(a) - np.array(b))
    
    def get_neighbors(node, grid_size=1.0):
        """Generate the eight-connected neighbours of a cell."""
        x, y = node
        neighbors = []
        
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # orthogonal
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # diagonal
        ]
        
        for dx, dy in directions:
            new_x = x + dx * grid_size
            new_y = y + dy * grid_size
            neighbors.append((new_x, new_y))
        
        return neighbors
    
    def is_collision_free(point, polygons):
        """Test whether a point lies inside an obstacle."""
        point_obj = Point(point)
        for poly in polygons:
            if poly.contains(point_obj):
                return False
        return True
    
    def line_collision_free(from_point, to_point, polygons):
        """Test whether a segment intersects an obstacle."""
        line = LineString([from_point, to_point])
        for poly in polygons:
            if poly.intersects(line):
                return False
        return True
    
    def reconstruct_path(came_from, current):
        """Rebuild the path by walking back through came_from."""
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path
    
    # convert the obstacles to shapely polygons
    polygons = []
    for p in terrain_polygons:
        if isinstance(p, Polygon):
            polygons.append(p)
        else:
            polygons.append(Polygon(p))
    
    # planning is impossible if either endpoint is inside an obstacle
    if not is_collision_free(start, polygons):
        return []
    if not is_collision_free(goal, polygons):
        return []
    
    open_set = []  # priority queue
    heapq.heappush(open_set, (0, start))
    
    came_from = {}
    g_score = defaultdict(lambda: float('inf'))
    g_score[start] = 0
    
    f_score = defaultdict(lambda: float('inf'))
    f_score[start] = heuristic(start, goal)
    
    open_set_hash = {start}  # hash set, for fast membership tests
    
    iteration_count = 0
    
    while open_set and iteration_count < max_iter:
        iteration_count += 1
        
        current = heapq.heappop(open_set)[1]
        open_set_hash.remove(current)
        
        if euclidean_distance(current, goal) < step_size:
            return reconstruct_path(came_from, current)
        
        for neighbor in get_neighbors(current, step_size):
            if not is_collision_free(neighbor, polygons):
                continue
            
            # cost of reaching the neighbour through the current node
            tentative_g_score = g_score[current] + euclidean_distance(current, neighbor)
            
            if tentative_g_score < g_score[neighbor]:
                # a shorter route to this neighbour
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                
                if neighbor not in open_set_hash:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    open_set_hash.add(neighbor)
    
    # no path: fall back to the node closest to the goal
    if came_from:
        min_distance = float('inf')
        closest_node = None
        
        for node in g_score.keys():
            dist = euclidean_distance(node, goal)
            if dist < min_distance:
                min_distance = dist
                closest_node = node
        
        if closest_node and line_collision_free(closest_node, goal, polygons):
            # join that node to the goal with a straight segment
            path = reconstruct_path(came_from, closest_node)
            path.append(goal)
            return path
        elif closest_node:
            return reconstruct_path(came_from, closest_node)
    
    return []  # no path found
