from enum import Enum
import numpy as np
import math
from shapely.geometry import Polygon, Point, LineString
try:
    from scipy.spatial import cKDTree as _cKDTree
except Exception:
    _cKDTree = None


class RobotType(Enum):
    circle = 0
    rectangle = 1


class DWAPlanner:
    def __init__(self, config):
        # Configuration parameters
        self.config = config
        self.robot_type = RobotType.rectangle
        self.robot_length = 1.0
        self.robot_width = 1.0
        self.robot_radius = config.getConfiguration('robot_radius') or 1.0
        # 안전 마진 추가 (회피 성능 향상) - 프로필에서 설정 가능
        self.safety_margin = config.getConfiguration('safety_margin') or 1.5
        # 정적 폴리곤 인플레이션/완충 구역 및 비용 가중치 - 프로필에서 설정 가능
        self.static_inflation = config.getConfiguration(
            'static_inflation') or 1.0
        self.static_soft_zone = config.getConfiguration(
            'static_soft_zone') or 2.0
        self.terrain_cost_gain = config.getConfiguration(
            'terrain_cost_gain') or 1.0

        # 정적 지형 폴리곤 인덱스(인플레이션 적용) 캐시
        try:
            from shapely.strtree import STRtree  # noqa: F401
            self._use_shapely = True
        except Exception:
            self._use_shapely = False
        self._inflated_tree = None
        self._inflated_geoms = []  # 인플레이션된 shapely Polygon 리스트
        self._terrain_polys_ref = None  # 원본 레퍼런스 추적
        self._terrain_first_last_id = None  # 인플레이스 변경 감지

    def calc_dwa(self, curPose, goal, obstacles, terrain_polygons, **kwargs):
        # print(f"[DWA] Input - Pose: {curPose[:3]}, Goal: {goal}")
        # print(f"[DWA] Obstacles shape: {obstacles.shape if hasattr(obstacles, 'shape') else type(obstacles)}")

        # tracking_only 프로필에서 장애물 회피 비용이 0인 경우 특별 처리
        obstacle_cost_gain = self.config.getConfiguration(
            'obstacle_cost_gain') or 1.0

        # tracking_only 모드 감지 (obstacle_cost_gain이 0이면 tracking_only)
        if obstacle_cost_gain == 0.0:
            # 장애물과의 최소 거리 계산
            min_obstacle_distance = self._calc_min_obstacle_distance(
                curPose, obstacles)
            safety_tolerance = self.config.getConfiguration(
                'safety_tolerance') or 5.0

            # 장애물이 safety_tolerance 내에 있으면 현재 위치에서 정지
            if min_obstacle_distance < safety_tolerance:
                print(
                    f"[DWA] tracking_only mode: obstacle detected at {min_obstacle_distance:.2f}m, stopping at current position")
                # 현재 위치를 목표로 반환 (정지)
                return [curPose[0], curPose[1], curPose[2]]

        # Dynamic Window 생성
        dw = self.calc_dynamic_window(curPose)
        # print(f"[DWA] Dynamic Window: {dw}")

        # 최적의 제어 입력과 경로를 선택
        best_u, best_trajectory = self.calc_control_and_trajectory(
            curPose, dw, goal, obstacles, terrain_polygons
        )

        target_state = best_trajectory[-1].tolist()[:3]
        # print(f"[DWA] Output target: {target_state}")

        return target_state

    def plan(self, current_pose, goal, obstacles, terrain_polygons):
        """
        내부적으로 사용되는 계획 메서드
        """
        dw = self.calc_dynamic_window(current_pose)
        best_u, best_trajectory = self.calc_control_and_trajectory(
            current_pose, dw, goal, obstacles, terrain_polygons
        )
        return best_u, best_trajectory

    def calc_dynamic_window(self, x):
        # 파라미터 안전 체크 및 기본값 설정
        min_speed = self.config.getConfiguration('min_speed')
        if min_speed is None:
            min_speed = 0.0

        max_speed = self.config.getConfiguration('max_speed')
        if max_speed is None:
            max_speed = 1.5

        max_yaw_rate = self.config.getConfiguration('max_yaw_rate')
        if max_yaw_rate is None:
            max_yaw_rate = 0.7

        max_accel = self.config.getConfiguration('max_accel')
        if max_accel is None:
            max_accel = 0.2

        dt = self.config.getConfiguration('dt')
        if dt is None:
            dt = 1.0

        max_delta_yaw_rate = self.config.getConfiguration('max_delta_yaw_rate')
        if max_delta_yaw_rate is None:
            max_delta_yaw_rate = 6.28

        # 속도와 각속도 제한 (고정 범위)
        Vs = [min_speed, max_speed, -max_yaw_rate, max_yaw_rate]

        # 동적 제한 (현재 속도 기반)
        Vd = [x[3] - max_accel * dt,
              x[3] + max_accel * dt,
              x[4] - max_delta_yaw_rate * dt,
              x[4] + max_delta_yaw_rate * dt]

        # 최종 Dynamic Window (두 제한의 교집합)
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw

    def calc_control_and_trajectory(self, x, dw, goal, ob, terrain_polygons):
        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        # 지형 인덱스 준비(맵 변경 시 1회만 재구성)
        self._ensure_terrain_index(terrain_polygons)

        # 해상도 파라미터 안전 체크 (해상도 높임 → 더 많은 경로 탐색 → 정확도 향상)
        v_resolution = self.config.getConfiguration(
            'v_resolution') or 0.1  # 0.1 → 0.05 (2배 정밀)
        yaw_rate_resolution = self.config.getConfiguration(
            'yaw_rate_resolution') or 0.017  # 0.017 → 0.01 (1.7배 정밀)

        # 비용 가중치/최대속도 캐싱 (목표 지향성 강화)
        to_goal_cost_gain = self.config.getConfiguration(
            'to_goal_cost_gain') or 2.0  # 1.0 → 2.0 (목표 지향 강화)
        speed_cost_gain = self.config.getConfiguration(
            'speed_cost_gain') or 0.5  # 1.0 → 0.5 (속도 우선순위 낮춤)
        obstacle_cost_gain = self.config.getConfiguration(
            'obstacle_cost_gain') or 1.5  # 1.0 → 1.5 (장애물 회피 강화)
        max_speed_config = self.config.getConfiguration('max_speed') or 1.5

        # 장애물 배열 정규화 및 KDTree 구성(가능 시)
        ob_array = ob
        if ob_array is None or (hasattr(ob_array, '__len__') and len(ob_array) == 0):
            ob_array = None
            ob_tree = None
        else:
            if isinstance(ob_array, dict):
                tmp = []
                for pos in ob_array.values():
                    if isinstance(pos, (list, tuple)) and len(pos) >= 2:
                        tmp.append([pos[0], pos[1]])
                ob_array = np.array(tmp) if tmp else None
            elif isinstance(ob_array, list):
                ob_array = np.array(ob_array) if len(ob_array) > 0 else None
            elif isinstance(ob_array, np.ndarray):
                if ob_array.size == 0:
                    ob_array = None
            else:
                ob_array = None

            if ob_array is not None and ob_array.ndim == 1:
                ob_array = ob_array.reshape(-1, 2)
            if ob_array is not None and ob_array.shape[1] >= 2 and _cKDTree is not None:
                try:
                    ob_tree = _cKDTree(ob_array[:, :2])
                except Exception:
                    ob_tree = None
            else:
                ob_tree = None

        # 모든 가능한 속도와 각속도 조합 탐색
        for v in np.arange(dw[0], dw[1], v_resolution):
            for yaw_rate in np.arange(dw[2], dw[3], yaw_rate_resolution):
                trajectory = self.predict_trajectory(x_init, v, yaw_rate)

                # 1) 가벼운 비용 먼저 계산 및 프루닝
                to_goal_cost = to_goal_cost_gain * \
                    self.calc_to_goal_cost(trajectory, goal)
                speed_cost = speed_cost_gain * \
                    (max_speed_config - trajectory[-1, 3])
                partial_cost = to_goal_cost + speed_cost
                if partial_cost >= min_cost:
                    continue

                # 2) 장애물 비용 (충돌 시 즉시 배제)
                if ob_array is not None:
                    ob_cost_raw = self.calc_obstacle_cost(
                        trajectory, ob_array, ob_tree)
                    if ob_cost_raw == float("Inf"):
                        continue
                    ob_cost = obstacle_cost_gain * ob_cost_raw
                    if partial_cost + ob_cost >= min_cost:
                        continue
                else:
                    ob_cost = 0.0

                # 3) 지형 비용 (교차 시 배제)
                terrain_cost = self.calc_terrain_cost(
                    trajectory, terrain_polygons)
                if terrain_cost == float("Inf"):
                    continue

                final_cost = partial_cost + ob_cost + terrain_cost

                # 최소 비용 궤적 선택
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, yaw_rate]
                    best_trajectory = trajectory

        return best_u, best_trajectory

    def predict_trajectory(self, x_init, v, yaw_rate):
        x = np.array(x_init)
        predict_time = self.config.getConfiguration('predict_time') or 3.0
        dt = self.config.getConfiguration('dt') or 1.0

        steps = int(math.floor(predict_time / dt)) + \
            1  # while time<=predict_time 반복 횟수
        n_rows = 1 + steps  # 초기 상태 포함
        trajectory = np.zeros((n_rows, len(x)), dtype=float)
        trajectory[0] = x

        for i in range(1, n_rows):
            x = self.motion(x, [v, yaw_rate], dt)
            trajectory[i] = x
        return trajectory

    def motion(self, x, u, dt):
        """차량 운동 모델"""
        x[2] += u[1] * dt  # yaw 업데이트
        x[0] += u[0] * math.cos(x[2]) * dt  # x 위치 업데이트
        x[1] += u[0] * math.sin(x[2]) * dt  # y 위치 업데이트
        x[3] = u[0]  # 선속도
        x[4] = u[1]  # 각속도
        return x

    def calc_terrain_cost(self, trajectory, terrain_polygons):
        """지형 비용 계산: 인플레이션 영역(충돌) + 소프트존 거리 비용"""
        # STRtree가 준비되어 있으면 빠른 경로 사용
        if self._use_shapely and self._inflated_tree is not None and len(self._inflated_geoms) > 0:
            try:
                # 궤적을 LineString으로 변환(한 번만 생성)
                coords = trajectory[:, :2]
                line = LineString([(float(x), float(y)) for x, y in coords])

                # 1) 충돌: 인플레이션된 폴리곤과 교차하면 즉시 무한대 비용
                for g in self._inflated_tree.query(line):
                    if line.intersects(g):
                        return float("Inf")

                # 2) 소프트존 비용: 소프트존 버퍼와 교차하는 후보에 대해서만 거리 비용 합산
                total_cost = 0.0
                if self.static_soft_zone > 0.0:
                    soft_buf = line.buffer(self.static_soft_zone)
                    for g in self._inflated_tree.query(soft_buf):
                        try:
                            d = float(line.distance(g))
                        except Exception:
                            continue
                        if d < self.static_soft_zone:
                            total_cost += self.terrain_cost_gain * \
                                (1.0 / (d + 1e-3))
                return total_cost
            except Exception:
                # 인덱스 경로가 실패하면 폴백
                pass

        # 폴백: 인덱스가 없을 때 기존 방식 사용
        if not terrain_polygons:
            return 0.0
        total_cost = 0.0
        for poly in terrain_polygons:
            try:
                polygon = poly if hasattr(poly, 'exterior') else Polygon(poly)
                inflated = polygon.buffer(self.static_inflation)
                min_clearance = float('inf')
                for xy in trajectory[:, :2]:
                    p = Point(float(xy[0]), float(xy[1]))
                    if inflated.contains(p):
                        return float("Inf")
                    d = p.distance(inflated)
                    if d < min_clearance:
                        min_clearance = d
                if min_clearance < self.static_soft_zone:
                    total_cost += self.terrain_cost_gain * \
                        (1.0 / (min_clearance + 1e-3))
            except Exception:
                continue
        return total_cost

    # ==== 정적 지형 인덱스 관리 ====
    def _ensure_terrain_index(self, terrain_polygons):
        """terrain_polygons 변경 시 1회만 인플레이션+STRtree 재구성"""
        if not self._use_shapely:
            return
        if not terrain_polygons:
            # 비우기
            self._inflated_tree = None
            self._inflated_geoms = []
            self._terrain_polys_ref = terrain_polygons
            self._terrain_first_last_id = None
            return

        # 변경 감지: 리스트 레퍼런스/길이/첫·마지막 요소 id
        needs_update = False
        if terrain_polygons is not self._terrain_polys_ref:
            needs_update = True
        else:
            try:
                latest_len = len(terrain_polygons)
                cached_len = len(self._inflated_geoms)
                if latest_len != cached_len:
                    needs_update = True
                elif latest_len > 0:
                    first_id = id(terrain_polygons[0])
                    last_id = id(terrain_polygons[-1])
                    if self._terrain_first_last_id != (first_id, last_id):
                        needs_update = True
            except Exception:
                needs_update = True

        if not needs_update and self._inflated_tree is not None:
            return

        # 인덱스 재구성
        self._build_terrain_index(terrain_polygons)

    def _build_terrain_index(self, terrain_polygons):
        from shapely.geometry import Polygon as ShapelyPolygon
        try:
            from shapely.strtree import STRtree
        except Exception:
            self._inflated_tree = None
            self._inflated_geoms = []
            return

        geoms = []
        for p in terrain_polygons:
            try:
                if hasattr(p, 'exterior'):
                    g = p
                else:
                    g = ShapelyPolygon(p)
                # 인플레이션 적용 후 저장
                geoms.append(g.buffer(self.static_inflation))
            except Exception:
                continue

        if len(geoms) == 0:
            self._inflated_tree = None
            self._inflated_geoms = []
        else:
            try:
                self._inflated_tree = STRtree(geoms)
                self._inflated_geoms = geoms
            except Exception:
                self._inflated_tree = None
                self._inflated_geoms = []

        # 변경 추적용 레퍼런스/ID 저장
        self._terrain_polys_ref = terrain_polygons
        try:
            if isinstance(terrain_polygons, (list, tuple)) and len(terrain_polygons) > 0:
                self._terrain_first_last_id = (
                    id(terrain_polygons[0]), id(terrain_polygons[-1]))
            else:
                self._terrain_first_last_id = None
        except Exception:
            self._terrain_first_last_id = None

    def calc_to_goal_cost(self, trajectory, goal):
        """목표까지의 방향 비용 계산"""
        if not goal or len(goal) < 2:
            return float("Inf")

        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        current_heading = trajectory[-1, 2]
        cost_angle = error_angle - current_heading

        # 각도를 [-π, π] 범위로 정규화
        cost_angle = (cost_angle + math.pi) % (2 * math.pi) - math.pi
        cost = min(abs(cost_angle), 2 * math.pi - abs(cost_angle))
        return cost

    def calc_obstacle_cost(self, trajectory, ob, ob_tree=None):
        """장애물 비용 계산 (KDTree 가속 + 다단계 검사)"""
        # 입력 정규화(이미 ndarray라고 가정하되, 안전 폴백 포함)
        if ob is None:
            return 0.0
        if isinstance(ob, np.ndarray):
            if ob.size == 0 or ob.ndim == 0:
                return 0.0
            if ob.ndim == 1:
                ob = ob.reshape(-1, 2)
            if ob.shape[1] < 2:
                return 0.0
        else:
            try:
                ob = np.array(ob)
            except Exception:
                return 0.0
            if ob.size == 0:
                return 0.0
            if ob.ndim == 1:
                ob = ob.reshape(-1, 2)
            if ob.shape[1] < 2:
                return 0.0

        # 다단계 충돌 검사(충돌 시 즉시 무한대)
        if self.multi_step_collision_check(trajectory, ob, ob_tree) == float("Inf"):
            return float("Inf")

        # 최근접 거리 계산: KDTree 있으면 사용, 없으면 브로드캐스팅
        try:
            if ob_tree is not None:
                dists, _ = ob_tree.query(trajectory[:, :2], k=1)
                min_r = float(np.min(dists))
            else:
                ox = ob[:, 0]
                oy = ob[:, 1]
                dx = trajectory[:, 0][:, np.newaxis] - ox[np.newaxis, :]
                dy = trajectory[:, 1][:, np.newaxis] - oy[np.newaxis, :]
                r = np.hypot(dx, dy)
                min_r = float(np.min(r))
        except Exception:
            ox = ob[:, 0]
            oy = ob[:, 1]
            dx = trajectory[:, 0][:, np.newaxis] - ox[np.newaxis, :]
            dy = trajectory[:, 1][:, np.newaxis] - oy[np.newaxis, :]
            r = np.hypot(dx, dy)
            min_r = float(np.min(r))

        if min_r <= 0:
            return float("Inf")

        # 거리 기반 비용(기존 로직 유지) - 프로필에서 설정 가능
        critical_distance = self.config.getConfiguration(
            'critical_distance') or 3.0
        warning_distance = self.config.getConfiguration(
            'warning_distance') or 6.0
        if min_r <= critical_distance:
            return 1000.0 / (min_r ** 2)
        elif min_r <= warning_distance:
            return 100.0 / min_r
        else:
            return 10.0 / min_r

    def multi_step_collision_check(self, trajectory, ob, ob_tree=None):
        """다단계 충돌 검사 - KDTree(있으면) 반경 질의로 후보 축소"""
        if ob is None or (hasattr(ob, 'size') and ob.size == 0):
            return 0.0

        # 궤적을 더 세밀하게 검사 (3개 지점마다 검사)
        check_indices = range(0, len(trajectory), 3)

        for i in check_indices:
            if i >= len(trajectory):
                break

            current_pos = trajectory[i]

            if self.robot_type == RobotType.rectangle:
                # 사각형 로봇 충돌 검사 (안전 마진 적용)
                yaw = current_pos[2]
                cos_yaw = np.cos(yaw)
                sin_yaw = np.sin(yaw)

                # 안전 마진을 적용한 더 큰 충돌 영역
                extended_length = self.robot_length / 2 + self.safety_margin
                extended_width = self.robot_width / 2 + self.safety_margin
                radius_bound = math.hypot(extended_length, extended_width)

                # KDTree 반경 질의(가능 시)로 후보 축소
                if ob_tree is not None:
                    idxs = ob_tree.query_ball_point(
                        [current_pos[0], current_pos[1]], r=radius_bound)
                    candidates = ob[idxs] if len(idxs) > 0 else []
                else:
                    candidates = ob

                for obstacle in candidates:
                    # 로컬 좌표계로 변환
                    dx = obstacle[0] - current_pos[0]
                    dy = obstacle[1] - current_pos[1]

                    local_x = dx * cos_yaw + dy * sin_yaw
                    local_y = -dx * sin_yaw + dy * cos_yaw

                    # 충돌 검사
                    if (abs(local_x) <= extended_length and
                            abs(local_y) <= extended_width):
                        return float("Inf")

            elif self.robot_type == RobotType.circle:
                # 원형 로봇 충돌 검사 (안전 마진 적용)
                extended_radius = self.robot_radius + self.safety_margin

                if ob_tree is not None:
                    idxs = ob_tree.query_ball_point(
                        [current_pos[0], current_pos[1]], r=extended_radius)
                    candidates = ob[idxs] if len(idxs) > 0 else []
                else:
                    candidates = ob

                for obstacle in candidates:
                    distance = np.hypot(obstacle[0] - current_pos[0],
                                        obstacle[1] - current_pos[1])
                    if distance <= extended_radius:
                        return float("Inf")

        return 0.0

    def _calc_min_obstacle_distance(self, curPose, obstacles):
        """현재 위치와 가장 가까운 장애물 사이의 거리 계산"""
        if obstacles is None or (hasattr(obstacles, '__len__') and len(obstacles) == 0):
            return float('inf')

        # 장애물 배열 정규화
        if isinstance(obstacles, dict):
            obs_positions = []
            for pos in obstacles.values():
                if isinstance(pos, (list, tuple)) and len(pos) >= 2:
                    obs_positions.append([pos[0], pos[1]])
            if not obs_positions:
                return float('inf')
            ob_array = np.array(obs_positions)
        elif isinstance(obstacles, list):
            if len(obstacles) == 0:
                return float('inf')
            ob_array = np.array(obstacles)
        elif isinstance(obstacles, np.ndarray):
            if obstacles.size == 0:
                return float('inf')
            ob_array = obstacles
        else:
            return float('inf')

        # 차원 확인 및 정규화
        if ob_array.ndim == 1:
            ob_array = ob_array.reshape(-1, 2)
        if ob_array.shape[1] < 2:
            return float('inf')

        # 현재 위치와 모든 장애물과의 거리 계산
        current_pos = np.array([curPose[0], curPose[1]])
        distances = np.linalg.norm(ob_array[:, :2] - current_pos, axis=1)

        return float(np.min(distances))
