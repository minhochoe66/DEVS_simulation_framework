import numpy as np
from shapely.geometry import Point, LineString, Polygon
import heapq
from collections import defaultdict

def A_star(start, goal, terrain_polygons, max_iter=5000, step_size=1, goal_sample_rate=5):
    """
    A* 알고리즘을 사용한 경로 계획
    
    Args:
        start: 시작점 (x, y)
        goal: 목표점 (x, y)
        terrain_polygons: 장애물 폴리곤 리스트
        max_iter: 최대 반복 횟수 (RRT 호환성을 위해 유지)
        step_size: 그리드 크기 (RRT 호환성을 위해 유지)
        goal_sample_rate: 목표 샘플링 비율 (RRT 호환성을 위해 유지)
    
    Returns:
        경로 리스트: [(x1, y1), (x2, y2), ...]
    """
    
    def heuristic(a, b):
        """맨해튼 거리 휴리스틱"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def euclidean_distance(a, b):
        """유클리드 거리"""
        return np.linalg.norm(np.array(a) - np.array(b))
    
    def get_neighbors(node, grid_size=1.0):
        """8방향 이웃 노드 생성"""
        x, y = node
        neighbors = []
        
        # 8방향 이동
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
        """점이 장애물과 충돌하는지 확인"""
        point_obj = Point(point)
        for poly in polygons:
            if poly.contains(point_obj):
                return False
        return True
    
    def line_collision_free(from_point, to_point, polygons):
        """선분이 장애물과 충돌하는지 확인"""
        line = LineString([from_point, to_point])
        for poly in polygons:
            if poly.intersects(line):
                return False
        return True
    
    def reconstruct_path(came_from, current):
        """경로 재구성"""
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path
    
    # 장애물 폴리곤을 shapely Polygon 객체로 변환
    polygons = []
    for p in terrain_polygons:
        if isinstance(p, Polygon):
            polygons.append(p)
        else:
            polygons.append(Polygon(p))
    
    # 시작점과 목표점이 장애물 내부에 있는지 확인
    if not is_collision_free(start, polygons):
        return []
    if not is_collision_free(goal, polygons):
        return []
    
    # A* 알고리즘 구현
    open_set = []  # 우선순위 큐
    heapq.heappush(open_set, (0, start))
    
    came_from = {}
    g_score = defaultdict(lambda: float('inf'))
    g_score[start] = 0
    
    f_score = defaultdict(lambda: float('inf'))
    f_score[start] = heuristic(start, goal)
    
    open_set_hash = {start}  # 빠른 검색을 위한 해시셋
    
    iteration_count = 0
    
    while open_set and iteration_count < max_iter:
        iteration_count += 1
        
        current = heapq.heappop(open_set)[1]
        open_set_hash.remove(current)
        
        # 목표에 도달했는지 확인
        if euclidean_distance(current, goal) < step_size:
            return reconstruct_path(came_from, current)
        
        # 이웃 노드들 탐색
        for neighbor in get_neighbors(current, step_size):
            if not is_collision_free(neighbor, polygons):
                continue
            
            # 현재 노드를 거쳐서 이웃에 도달하는 비용
            tentative_g_score = g_score[current] + euclidean_distance(current, neighbor)
            
            if tentative_g_score < g_score[neighbor]:
                # 더 나은 경로를 찾았음
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                
                if neighbor not in open_set_hash:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    open_set_hash.add(neighbor)
    
    # 경로를 찾지 못한 경우, 목표점에 가장 가까운 노드까지의 경로 반환
    if came_from:
        # 목표점에 가장 가까운 노드 찾기
        min_distance = float('inf')
        closest_node = None
        
        for node in g_score.keys():
            dist = euclidean_distance(node, goal)
            if dist < min_distance:
                min_distance = dist
                closest_node = node
        
        if closest_node and line_collision_free(closest_node, goal, polygons):
            # 가장 가까운 노드에서 목표점까지 직선 연결
            path = reconstruct_path(came_from, closest_node)
            path.append(goal)
            return path
        elif closest_node:
            return reconstruct_path(came_from, closest_node)
    
    return []  # 경로를 찾지 못함
