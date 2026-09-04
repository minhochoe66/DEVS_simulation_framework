import numpy as np
from shapely.geometry import Point, LineString, Polygon
import heapq
from collections import defaultdict

def A_star(start, goal, terrain_polygons, max_iter=5000, step_size=1, goal_sample_rate=5):
    """A* 전역 경로 계획.

    start에서 goal까지 terrain_polygons를 피하는 경로를 [(x, y), ...]로 반환한다.
    경로를 찾지 못하면 목표에 가장 가까운 노드까지의 경로를 반환한다.
    max_iter, step_size, goal_sample_rate는 쓰이지 않으며 플래너 공통
    인터페이스를 맞추기 위해 남겨 둔 인자다.
    """
    
    def heuristic(a, b):
        """맨해튼 거리 휴리스틱."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def euclidean_distance(a, b):
        """유클리드 거리."""
        return np.linalg.norm(np.array(a) - np.array(b))
    
    def get_neighbors(node, grid_size=1.0):
        """8방향 이웃 노드를 생성한다."""
        x, y = node
        neighbors = []
        
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # 상하좌우
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # 대각선
        ]
        
        for dx, dy in directions:
            new_x = x + dx * grid_size
            new_y = y + dy * grid_size
            neighbors.append((new_x, new_y))
        
        return neighbors
    
    def is_collision_free(point, polygons):
        """점이 장애물 안에 있는지 확인한다."""
        point_obj = Point(point)
        for poly in polygons:
            if poly.contains(point_obj):
                return False
        return True
    
    def line_collision_free(from_point, to_point, polygons):
        """선분이 장애물과 교차하는지 확인한다."""
        line = LineString([from_point, to_point])
        for poly in polygons:
            if poly.intersects(line):
                return False
        return True
    
    def reconstruct_path(came_from, current):
        """came_from을 거슬러 경로를 복원한다."""
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path
    
    # 장애물을 shapely Polygon으로 변환
    polygons = []
    for p in terrain_polygons:
        if isinstance(p, Polygon):
            polygons.append(p)
        else:
            polygons.append(Polygon(p))
    
    # 시작점과 목표점이 장애물 안에 있으면 계획할 수 없다
    if not is_collision_free(start, polygons):
        return []
    if not is_collision_free(goal, polygons):
        return []
    
    open_set = []  # 우선순위 큐
    heapq.heappush(open_set, (0, start))
    
    came_from = {}
    g_score = defaultdict(lambda: float('inf'))
    g_score[start] = 0
    
    f_score = defaultdict(lambda: float('inf'))
    f_score[start] = heuristic(start, goal)
    
    open_set_hash = {start}  # 조회를 빠르게 하기 위한 해시셋
    
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
            
            # 현재 노드를 거쳐 이웃에 도달하는 비용
            tentative_g_score = g_score[current] + euclidean_distance(current, neighbor)
            
            if tentative_g_score < g_score[neighbor]:
                # 더 짧은 경로를 찾았다
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                
                if neighbor not in open_set_hash:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    open_set_hash.add(neighbor)
    
    # 경로가 없으면 목표에 가장 가까운 노드까지를 반환한다
    if came_from:
        min_distance = float('inf')
        closest_node = None
        
        for node in g_score.keys():
            dist = euclidean_distance(node, goal)
            if dist < min_distance:
                min_distance = dist
                closest_node = node
        
        if closest_node and line_collision_free(closest_node, goal, polygons):
            # 가장 가까운 노드에서 목표까지는 직선으로 잇는다
            path = reconstruct_path(came_from, closest_node)
            path.append(goal)
            return path
        elif closest_node:
            return reconstruct_path(came_from, closest_node)
    
    return []  # 경로를 찾지 못함
