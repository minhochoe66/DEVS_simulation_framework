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
        # 안전 마진. 설정 파일에서 조정할 수 있다
        self.safety_margin = config.getConfiguration('safety_margin') or 1.5
        # 정적 폴리곤 인플레이션, 완충 구역, 비용 가중치. 설정 파일에서 조정할 수 있다
        self.static_inflation = config.getConfiguration(
            'static_inflation') or 1.0
        self.static_soft_zone = config.getConfiguration(
            'static_soft_zone') or 2.0
        self.terrain_cost_gain = config.getConfiguration(
            'terrain_cost_gain') or 1.0

        # 인플레이션을 적용한 정적 지형 인덱스 캐시
        try:
            from shapely.strtree import STRtree  # noqa: F401
            self._use_shapely = True
        except Exception:
            self._use_shapely = False
        self._inflated_tree = None
        self._inflated_geoms = []  # 인플레이션된 shapely Polygon
        self._terrain_polys_ref = None  # 원본 참조
        self._terrain_first_last_id = None  # 제자리 변경 감지용

    def calc_dwa(self, curPose, goal, obstacles, terrain_polygons, **kwargs):
        # print(f"[DWA] Input - Pose: {curPose[:3]}, Goal: {goal}")
        # print(f"[DWA] Obstacles shape: {obstacles.shape if hasattr(obstacles, 'shape') else type(obstacles)}")

        # tracking_only 프로파일은 장애물 비용이 0이므로 따로 처리한다
        obstacle_cost_gain = self.config.getConfiguration(
            'obstacle_cost_gain') or 1.0

        # obstacle_cost_gain이 0이면 tracking_only 모드다
        if obstacle_cost_gain == 0.0:
            # 가장 가까운 장애물까지의 거리
            min_obstacle_distance = self._calc_min_obstacle_distance(
                curPose, obstacles)
            safety_tolerance = self.config.getConfiguration(
                'safety_tolerance') or 5.0

            # 장애물이 safety_tolerance 안으로 들어오면 그 자리에 선다
            if min_obstacle_distance < safety_tolerance:
                print(
                    f"[DWA] tracking_only mode: obstacle detected at {min_obstacle_distance:.2f}m, stopping at current position")
                # 현재 위치를 목표로 돌려주어 정지시킨다
                return [curPose[0], curPose[1], curPose[2]]

        # 동적 윈도우 계산
        dw = self.calc_dynamic_window(curPose)
        # print(f"[DWA] Dynamic Window: {dw}")

        # 최적 제어 입력과 궤적 선택
        best_u, best_trajectory = self.calc_control_and_trajectory(
            curPose, dw, goal, obstacles, terrain_polygons
        )

        target_state = best_trajectory[-1].tolist()[:3]
        # print(f"[DWA] Output target: {target_state}")

        return target_state

    def plan(self, current_pose, goal, obstacles, terrain_polygons):
        """동적 윈도우 안의 속도 후보를 평가하여 최적 궤적을 고른다."""
        dw = self.calc_dynamic_window(current_pose)
        best_u, best_trajectory = self.calc_control_and_trajectory(
            current_pose, dw, goal, obstacles, terrain_polygons
        )
        return best_u, best_trajectory

    def calc_dynamic_window(self, x):
        # 파라미터 기본값 보정
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

        # 속도와 각속도의 고정 한계
        Vs = [min_speed, max_speed, -max_yaw_rate, max_yaw_rate]

        # 현재 속도에서 한 스텝에 도달 가능한 범위
        Vd = [x[3] - max_accel * dt,
              x[3] + max_accel * dt,
              x[4] - max_delta_yaw_rate * dt,
              x[4] + max_delta_yaw_rate * dt]

        # 두 한계의 교집합이 동적 윈도우다
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw

    def calc_control_and_trajectory(self, x, dw, goal, ob, terrain_polygons):
        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        # 지형 인덱스는 맵이 바뀔 때만 다시 만든다
        self._ensure_terrain_index(terrain_polygons)

        # 동적 윈도우 샘플링 해상도. 값이 작을수록 후보가 많아지고 계산이 늘어난다
        v_resolution = self.config.getConfiguration(
            'v_resolution') or 0.1  # 설정이 없을 때의 기본값
        yaw_rate_resolution = self.config.getConfiguration(
            'yaw_rate_resolution') or 0.017  # 설정이 없을 때의 기본값

        # 비용 가중치와 최대 속도. 설정값이 있으면 그것을 쓴다
        to_goal_cost_gain = self.config.getConfiguration(
            'to_goal_cost_gain') or 2.0  # 설정이 없을 때의 기본값
        speed_cost_gain = self.config.getConfiguration(
            'speed_cost_gain') or 0.5  # 설정이 없을 때의 기본값
        obstacle_cost_gain = self.config.getConfiguration(
            'obstacle_cost_gain') or 1.5  # 설정이 없을 때의 기본값
        max_speed_config = self.config.getConfiguration('max_speed') or 1.5

        # 장애물 배열을 정규화하고 가능하면 KDTree를 만든다
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

        # 속도와 각속도의 모든 조합을 훑는다
        for v in np.arange(dw[0], dw[1], v_resolution):
            for yaw_rate in np.arange(dw[2], dw[3], yaw_rate_resolution):
                trajectory = self.predict_trajectory(x_init, v, yaw_rate)

                # 1) 값싼 비용부터 계산하고 가망 없는 후보를 쳐낸다
                to_goal_cost = to_goal_cost_gain * \
                    self.calc_to_goal_cost(trajectory, goal)
                speed_cost = speed_cost_gain * \
                    (max_speed_config - trajectory[-1, 3])
                partial_cost = to_goal_cost + speed_cost
                if partial_cost >= min_cost:
                    continue

                # 2) 장애물 비용. 충돌하면 즉시 버린다
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

                # 3) 지형 비용. 정적 장애물과 교차하면 버린다
                terrain_cost = self.calc_terrain_cost(
                    trajectory, terrain_polygons)
                if terrain_cost == float("Inf"):
                    continue

                final_cost = partial_cost + ob_cost + terrain_cost

                # 비용이 가장 낮은 궤적을 고른다
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
            1  # predict_time 동안의 스텝 수
        n_rows = 1 + steps  # 초기 상태를 포함한다
        trajectory = np.zeros((n_rows, len(x)), dtype=float)
        trajectory[0] = x

        for i in range(1, n_rows):
            x = self.motion(x, [v, yaw_rate], dt)
            trajectory[i] = x
        return trajectory

    def motion(self, x, u, dt):
        """차동 구동 운동 모델."""
        x[2] += u[1] * dt  # yaw
        x[0] += u[0] * math.cos(x[2]) * dt  # x
        x[1] += u[0] * math.sin(x[2]) * dt  # y
        x[3] = u[0]  # 선속도
        x[4] = u[1]  # 각속도
        return x

    def calc_terrain_cost(self, trajectory, terrain_polygons):
        """지형 비용. 인플레이션 영역과의 충돌과 소프트존 거리 비용을 더한다."""
        # STRtree가 있으면 빠른 경로를 쓴다
        if self._use_shapely and self._inflated_tree is not None and len(self._inflated_geoms) > 0:
            try:
                # 궤적을 LineString으로 한 번만 만든다
                coords = trajectory[:, :2]
                line = LineString([(float(x), float(y)) for x, y in coords])

                # 1) 인플레이션된 폴리곤과 교차하면 비용을 무한대로 둔다
                for g in self._inflated_tree.query(line):
                    if line.intersects(g):
                        return float("Inf")

                # 2) 소프트존에 걸친 후보만 거리 비용을 더한다
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
                # 인덱스 경로가 실패하면 아래로 폴백한다
                pass

        # 폴백: 인덱스 없이 직접 계산한다
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

    # ==== 정적 지형 인덱스 ====
    def _ensure_terrain_index(self, terrain_polygons):
        """terrain_polygons가 바뀔 때만 인플레이션과 STRtree를 다시 만든다."""
        if not self._use_shapely:
            return
        if not terrain_polygons:
            # 비운다
            self._inflated_tree = None
            self._inflated_geoms = []
            self._terrain_polys_ref = terrain_polygons
            self._terrain_first_last_id = None
            return

        # 리스트 참조, 길이, 첫 요소와 마지막 요소의 id로 변경을 감지한다
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
                # 인플레이션을 적용해 저장한다
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

        # 다음 변경 감지를 위해 참조와 id를 남긴다
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
        """목표 방향과의 오차 비용."""
        if not goal or len(goal) < 2:
            return float("Inf")

        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        current_heading = trajectory[-1, 2]
        cost_angle = error_angle - current_heading

        # 각도를 [-pi, pi]로 정규화한다
        cost_angle = (cost_angle + math.pi) % (2 * math.pi) - math.pi
        cost = min(abs(cost_angle), 2 * math.pi - abs(cost_angle))
        return cost

    def calc_obstacle_cost(self, trajectory, ob, ob_tree=None):
        """장애물 비용. KDTree로 후보를 좁힌 뒤 단계적으로 충돌을 검사한다."""
        # 입력 정규화. ndarray가 아니어도 처리한다
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

        # 단계적 충돌 검사. 충돌하면 비용은 무한대다
        if self.multi_step_collision_check(trajectory, ob, ob_tree) == float("Inf"):
            return float("Inf")

        # 최근접 거리. KDTree가 있으면 쓰고 없으면 브로드캐스팅한다
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

        # 거리 기반 비용. 설정 파일에서 조정할 수 있다
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
        """단계적 충돌 검사. KDTree 반경 질의로 후보를 줄인다."""
        if ob is None or (hasattr(ob, 'size') and ob.size == 0):
            return 0.0

        # 궤적을 3개 지점마다 검사한다
        check_indices = range(0, len(trajectory), 3)

        for i in check_indices:
            if i >= len(trajectory):
                break

            current_pos = trajectory[i]

            if self.robot_type == RobotType.rectangle:
                # 사각형 로봇의 충돌 검사. 안전 마진을 적용한다
                yaw = current_pos[2]
                cos_yaw = np.cos(yaw)
                sin_yaw = np.sin(yaw)

                # 안전 마진만큼 넓힌 충돌 영역
                extended_length = self.robot_length / 2 + self.safety_margin
                extended_width = self.robot_width / 2 + self.safety_margin
                radius_bound = math.hypot(extended_length, extended_width)

                # 가능하면 KDTree 반경 질의로 후보를 줄인다
                if ob_tree is not None:
                    idxs = ob_tree.query_ball_point(
                        [current_pos[0], current_pos[1]], r=radius_bound)
                    candidates = ob[idxs] if len(idxs) > 0 else []
                else:
                    candidates = ob

                for obstacle in candidates:
                    # 로봇 로컬 좌표계로 변환한다
                    dx = obstacle[0] - current_pos[0]
                    dy = obstacle[1] - current_pos[1]

                    local_x = dx * cos_yaw + dy * sin_yaw
                    local_y = -dx * sin_yaw + dy * cos_yaw

                    # 충돌 검사
                    if (abs(local_x) <= extended_length and
                            abs(local_y) <= extended_width):
                        return float("Inf")

            elif self.robot_type == RobotType.circle:
                # 원형 로봇의 충돌 검사. 안전 마진을 적용한다
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
        """현재 위치에서 가장 가까운 장애물까지의 거리."""
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

        # 차원 확인
        if ob_array.ndim == 1:
            ob_array = ob_array.reshape(-1, 2)
        if ob_array.shape[1] < 2:
            return float('inf')

        # 모든 장애물까지의 거리
        current_pos = np.array([curPose[0], curPose[1]])
        distances = np.linalg.norm(ob_array[:, :2] - current_pos, axis=1)

        return float(np.min(distances))
