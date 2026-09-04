from SimulationEngine.ClassicDEVS.DEVSAtomicModel import DEVSAtomicModel
from modeling.Message.MsgManeuverState import MsgManeuverState
from shapely.geometry import Point, Polygon, LineString
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as pltPolygon
import re
import time
from Algorithm.PathPlanning.global_path.RRT import RRT
from Algorithm.PathPlanning.global_path.A_star import A_star
from Algorithm.PathPlanning.global_path.PRM import PRM
from Algorithm.PathPlanning.global_path.Theta_star import Theta_star


class GlobalPlanner(DEVSAtomicModel):
    def __init__(self, ID, objConfiguration, globalVar, algorithm="RRT"):
        super().__init__(ID)

        self.objConfiguration = objConfiguration
        self.globalVar = globalVar
        self.algorithm = algorithm

        # 환경 정보
        self.map_width = 100
        self.map_height = 100
        self.obstacles = []

        self.AMR_Global_Planner_ID = ID

        # AMR 및 작업 정보
        self.amr_id = None
        self.amr_position = None
        self.amr_yaw = None
        self.amr_velocity = None
        self.task_id = None
        self.task_from = None  # TransportCommand의 From 위치 (Equipment IN)
        self.task_to = None    # TransportCommand의 To 위치 (Equipment IN)
        self.task_from_nodeID = None  # From Equipment의 NodeID
        self.task_to_nodeID = None    # To Equipment의 NodeID
        self.planned_path = []
        self.path = []
        self.start = None

        # Transport Phase 관리
        # "TO_FROM": From Equipment로 이동 중
        # "AT_FROM": From Equipment 도착 (픽업 대기)
        # "TO_DESTINATION": To Equipment로 이동 중
        # "AT_DESTINATION": To Equipment 도착 (하역 완료)
        self.transportPhase = None
        self.currentGoal = None  # 현재 단계의 목표 위치
        self.currentGoalNodeID = None  # 현재 단계의 목표 NodeID

        # 장애물 전처리 캐싱 (성능 향상)
        self.cached_obstacle_polygons = None  # Shapely Polygon 객체 캐시
        self.cached_expanded_polygons = None  # Buffer 적용된 Polygon 캐시
        self.cache_safety_margin = 3.0

        # 장애물 정보를 한 번만 전처리 (시뮬레이션 시작 시)
        self._preprocess_obstacles()

        # 상태 변수 정의
        self.addStateVariable("state", "INIT")

        # 입출력 포트 정의
        self.addInputPort("Task_I")           # 작업 정보 수신
        # FleetManagement로부터 단순 이동 명령 (jobID 없음)
        self.addInputPort("amrGoCommand")
        self.addInputPort("ManeuverState_I")  # AMR 위치 정보 수신
        self.addInputPort("Replan")           # 재계획 요청
        self.addInputPort("StopSim")          # 시뮬레이션 중지

        self.addOutputPort("GlobalWaypoint_O")  # Waypoint 전송
        self.addOutputPort("RequestManeuver")   # Maneuver 요청

    def funcExternalTransition(self, strPort, objEvent):
        state = self.getStateValue("state")

        # Docking 완료 신호 - 작업 완료 후 대기 상태로
        if strPort == "Docking_I":
            self.setStateValue("state", "WAIT")
            self.globalVar.printTerminal(
                f"[{self.getTime()}][GlobalPlanner({self.ID})] Docking/WaitingArea completed, now WAITING for next task"
            )
            return
        if strPort == "Undocking_I":
            self.setStateValue("state", "WAIT")
        # 🚨 amrGoCommand는 모든 상태에서 처리 (긴급 "비켜라" 명령)
        if strPort == "amrGoCommand":
            # FleetManagement로부터 단순 이동 명령 (jobID 없음)
            if self.ID.split('_', 1)[0] == objEvent['amrID'].split('_', 1)[0]:
                planner_vehicle_id = self.ID.split('_')[0]
                task_vehicle_id = objEvent['amrID'].split('_')[0] if isinstance(
                    objEvent, dict) and 'amrID' in objEvent else None

                if task_vehicle_id and planner_vehicle_id in task_vehicle_id:
                    action = objEvent.get('action', 'GO')

                    if action == 'GO_WAITING':
                        # WaitingArea로 이동
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] 🚨 EMERGENCY GO_WAITING Command (State: {state}) for AMR {objEvent['amrID']}"
                        )

                        # Task 정보 추출 (jobID 없음)
                        self.amr_id = objEvent['amrID']
                        self.task_id = None  # jobID 없음

                        # 목표 위치만 설정
                        target_x = objEvent['x']
                        target_y = objEvent['y']
                        areaID = objEvent.get('areaID', None)

                        # AMR 현재 위치를 시작점으로
                        vehicleInfo = self.globalVar.getVehicleInfoByID(
                            self.amr_id)
                        if vehicleInfo:
                            latest_coords = vehicleInfo.getCoordinates()
                            self.amr_position = (
                                latest_coords[0], latest_coords[1])

                        self.task_from = self.amr_position  # 현재 위치
                        self.task_to = (target_x, target_y)  # 목표 위치
                        self.task_from_nodeID = None  # NodeID 없음
                        self.task_to_nodeID = None  # NodeID 없음

                        # WAITING phase 설정
                        self.transportPhase = "WAITING"
                        self.currentGoal = self.task_to
                        self.currentGoalNodeID = areaID  # WaitingArea ID

                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] GO_WAITING Command: Move to WaitingArea {areaID} at ({target_x}, {target_y})"
                        )

                        # 경로 계획 모드로 전환
                        self.setStateValue("state", "GPP")
                        return  # 즉시 처리
                    else:
                        # 일반 GO 명령
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] 🚨 EMERGENCY GO Command (State: {state}) for AMR {objEvent['amrID']}"
                        )

                        # Task 정보 추출 (jobID 없음)
                        self.amr_id = objEvent['amrID']
                        self.task_id = None  # jobID 없음

                        # 목표 위치만 설정 (현재 위치 → 목표 위치)
                        target_x = objEvent['x']
                        target_y = objEvent['y']

                        # AMR 현재 위치를 시작점으로
                        vehicleInfo = self.globalVar.getVehicleInfoByID(
                            self.amr_id)
                        if vehicleInfo:
                            latest_coords = vehicleInfo.getCoordinates()
                            self.amr_position = (
                                latest_coords[0], latest_coords[1])

                        self.task_from = self.amr_position  # 현재 위치
                        self.task_to = (target_x, target_y)  # 목표 위치
                        self.task_from_nodeID = None  # NodeID 없음
                        self.task_to_nodeID = None  # NodeID 없음

                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] GO Command: Move to ({target_x}, {target_y})"
                        )
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] Current position: {self.amr_position}"
                        )

                        # 단순 이동 (Transport Phase 없음)
                        self.transportPhase = None
                        self.currentGoal = self.task_to
                        self.currentGoalNodeID = None

                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] GO Command - Simple movement (no job)"
                        )

                        # 경로 계획 모드로 전환
                        self.setStateValue("state", "GPP")
                        return  # 즉시 처리
                else:
                    # 다른 AMR의 작업이면 무시
                    self.continueTimeAdvance()
            return  # amrGoCommand 처리 완료

        # INIT 또는 WAIT 상태에서만 새로운 Task 받음
        if state == "INIT" or state == "WAIT":
            if strPort == "Task_I":
                if self.ID.split('_', 1)[0] == objEvent['amrID'].split('_', 1)[0]:
                    planner_vehicle_id = self.ID.split('_')[0]
                    task_vehicle_id = objEvent['amrID'].split('_')[0] if isinstance(
                        objEvent, dict) and 'amrID' in objEvent else None

                    if task_vehicle_id and planner_vehicle_id in task_vehicle_id:
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] Received TransportCommand for Job #{objEvent['jobID']}"
                        )

                        # Task 정보 추출
                        self.amr_id = objEvent['amrID']
                        self.task_id = objEvent['jobID']

                        # From-To 정보 저장 (위치 + NodeID)
                        fromPos = objEvent['fromPosition']
                        toPos = objEvent['toPosition']
                        self.task_from = (fromPos['x'], fromPos['y'])
                        self.task_to = (toPos['x'], toPos['y'])
                        self.task_from_nodeID = objEvent['fromNodeID']
                        self.task_to_nodeID = objEvent['toNodeID']

                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] Transport: From {self.task_from_nodeID} → To {self.task_to_nodeID}"
                        )
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] Coordinates: From {self.task_from} → To {self.task_to}"
                        )

                        # Transport action에 따라 단계 결정
                        action = objEvent.get('action', 'TRANSPORT')

                        if action == 'TRANSPORT_NEXT':
                            # 언도킹 후 다음 목적지로 이동 - 바로 TO_DESTINATION 단계
                            self.transportPhase = "TO_DESTINATION"
                            self.currentGoal = self.task_to
                            self.currentGoalNodeID = self.task_to_nodeID

                            # UNDOCKING 후 AMR 위치를 최신으로 업데이트
                            amrID = objEvent['amrID']
                            vehicleInfo = self.globalVar.getVehicleInfoByID(
                                amrID)
                            if vehicleInfo:
                                latest_coords = vehicleInfo.getCoordinates()
                                old_position = self.amr_position
                                self.amr_position = (
                                    latest_coords[0], latest_coords[1])
                                print(
                                    f"🗺️ [GLOBAL_PLANNER] {self.ID}: AMR position updated {old_position} → {self.amr_position}")
                                print(
                                    f"   📦 Received from GlobalVar for {amrID}, planning to {self.task_to}")
                                self.globalVar.printTerminal(
                                    f"[{self.getTime()}][GlobalPlanner({self.ID})] Updated AMR position to latest: {self.amr_position}"
                                )

                            self.globalVar.printTerminal(
                                f"[{self.getTime()}][GlobalPlanner({self.ID})] Phase: TO_DESTINATION - Moving to next destination (UNDOCKING complete)"
                            )
                        else:
                            # 일반 Transport - From Equipment로 이동 시작
                            self.transportPhase = "TO_FROM"
                            self.currentGoal = self.task_from
                            self.currentGoalNodeID = self.task_from_nodeID

                            self.globalVar.printTerminal(
                                f"[{self.getTime()}][GlobalPlanner({self.ID})] Phase: TO_FROM - Moving to pickup location"
                            )

                        # 경로 계획 모드로 전환
                        self.setStateValue("state", "GPP")
                    else:
                        # 다른 AMR의 작업이면 무시
                        self.continueTimeAdvance()
            elif strPort == "Replan":
                # LocalPlanner로부터 재계획 요청
                # Vehicle 번호 비교

                amr_base_id = objEvent.strID.split(
                    '_')[0] if hasattr(objEvent, 'strID') else None
                planner_base_id = self.ID.split('_')[0]

                if amr_base_id and amr_base_id == planner_base_id:
                    self.setStateValue("state", "GPP")
                    if hasattr(objEvent, 'dblPositionE') and hasattr(objEvent, 'dblPositionN'):
                        self.start = [objEvent.dblPositionE,
                                      objEvent.dblPositionN]
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][GlobalPlanner({self.ID})] Replan requested"
                    )

            elif strPort == "ManeuverState_I":
                # AMR 위치 정보 업데이트
                if self.ID.split('_', 1)[0] == objEvent.strID:
                    self.update_data(objEvent)
                else:
                    self.continueTimeAdvance()

        # PLAN 상태: 이동 중이므로 Task_I 무시, Replan과 ManeuverState_I만 처리
        elif state == "PLAN":
            if strPort == "Task_I":
                # 이동 중이므로 새로운 Task 무시
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][GlobalPlanner({self.ID})] BUSY - Ignoring new task (currently in PLAN state)"
                )
                self.continueTimeAdvance()

            elif strPort == "Replan":
                # LocalPlanner로부터 재계획 요청
                amr_base_id = objEvent.strID.split(
                    '_')[0] if hasattr(objEvent, 'strID') else None
                planner_base_id = self.ID.split('_')[0]

                if amr_base_id and amr_base_id == planner_base_id:
                    self.setStateValue("state", "GPP")
                    if hasattr(objEvent, 'dblPositionE') and hasattr(objEvent, 'dblPositionN'):
                        self.start = [objEvent.dblPositionE,
                                      objEvent.dblPositionN]
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][GlobalPlanner({self.ID})] Replan requested during PLAN"
                    )

            elif strPort == "ManeuverState_I":
                # AMR 위치 정보 업데이트
                if self.ID.split('_', 1)[0] == objEvent.strID:
                    self.update_data(objEvent)
                else:
                    self.continueTimeAdvance()

        # 기타 상태: ManeuverState_I만 처리
        elif strPort == "ManeuverState_I":
            # 모든 상태에서 위치 정보 업데이트
            if self.ID.split('_', 1)[0] == objEvent.strID:
                self.update_data(objEvent)
            else:
                self.continueTimeAdvance()

    def funcInternalTransition(self):
        state = self.getStateValue("state")

        if state == "GPP" and self.currentGoal is not None:
            # 경로 계획 수행
            if self.amr_position and self.currentGoal:
                # 현재 목표 노드와 시작점 노드를 제외한 장애물 정보 가져오기
                exclude_nodes = []

                # 목표 노드 제외 (WaitingArea 포함)
                if self.currentGoalNodeID:
                    exclude_nodes.append(self.currentGoalNodeID)
                    # WaitingArea 계열은 prefix만 동일해도 제외 보장
                    if isinstance(self.currentGoalNodeID, str) and self.currentGoalNodeID.startswith('WAITING_AREA'):
                        exclude_nodes.append(self.currentGoalNodeID)

                # 시작점 노드 제외 (현재 AMR이 있는 장비의 outputPort)
                if self.transportPhase == "TO_DESTINATION" and self.task_from_nodeID:
                    # TO_DESTINATION 단계에서는 이전 장비의 outputPort에서 시작
                    start_nodeID = self.task_from_nodeID.replace('_IN', '_OUT')
                    exclude_nodes.append(start_nodeID)
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][GlobalPlanner] Excluding start node {start_nodeID} from obstacles"
                    )

                start_time = time.time()
                # 경로 계획 실행 (캐시된 장애물 사용 - 최적화)
                self.planned_path = self.generate_path_with_cached_obstacles(
                    self.amr_position,
                    self.currentGoal,
                    exclude_nodes=exclude_nodes
                )
                end_time = time.time()
                elapsed = end_time - start_time
                self.globalVar.GlobalPlanner_algorithm_time += elapsed
                self.globalVar.GlobalPlanner_call_count += 1
                # self.globalVar.printTerminal(
                #     f"[{self.getTime()}][GlobalPlanner] GlobalPlanner algorithm time: {elapsed:.3f} seconds"
                # )
                phase_name = "FROM Equipment" if self.transportPhase == "TO_FROM" else "TO Equipment"
                print(
                    f"[{self.getTime()}][GlobalPlanner] Phase: {self.transportPhase} - Path to {phase_name} planned with {len(self.planned_path)} waypoints")

                if not self.planned_path:
                    # 경로를 찾지 못한 경우 직선 경로
                    print(
                        f"[GlobalPlanner] No path found to {phase_name}, using direct path")
                    self.planned_path = [
                        (self.currentGoal[0], self.currentGoal[1])]

                # 경로 전송 상태로 변경
                self.setStateValue("state", "SEND_PATH")

        elif state == "SEND_PATH":
            # 출력 후 PLAN 상태로 변경
            self.setStateValue("state", "PLAN")

    def funcOutput(self):
        state = self.getStateValue("state")

        if state == "SEND_PATH" and self.planned_path and len(self.planned_path) > 0:
            # 첫 번째 waypoint 전송
            first_waypoint = self.planned_path[0]
            x, y = first_waypoint[0], first_waypoint[1]

            phase_name = "FROM Equipment" if self.transportPhase == "TO_FROM" else "TO Equipment"
            print(
                f"[{self.getTime()}][GlobalPlanner] Sending waypoint: ({x}, {y}) - Phase: {phase_name}")

            # 메시지 객체 생성 (transportPhase + goalNodeID 포함)
            objRequestMessage = MsgManeuverState(
                self.ID.split('_', 1)[0] + '_maneuver_amr',
                x,  # dblPositionX
                y,  # dblPositionY
                path=self.planned_path,  # 전체 경로
                transportPhase=self.transportPhase,  # "TO_FROM" or "TO_DESTINATION"
                goalNodeID=self.currentGoalNodeID  # 현재 목표 NodeID
            )

            # 메시지 전송
            self.addOutputEvent("GlobalWaypoint_O", objRequestMessage)

    def funcTimeAdvance(self):
        state = self.getStateValue("state")

        if state == "INIT":
            return float('inf')  # 첫 Task 대기
        elif state == "WAIT":
            return float('inf')  # 다음 Task 대기
        elif state == "GPP":
            return 0  # 즉시 경로 계획 실행
        elif state == "SEND_PATH":
            return 0  # 즉시 경로 전송
        elif state == "PLAN":
            return float('inf')  # AMR 이동 중
        else:
            return 1

    def extract_vehicle_number(self, id_string):
        """Vehicle 뒤의 숫자 추출"""
        match = re.search(r'Vehicle(\d+)', id_string)
        if match:
            return match.group(1)
        return None

    def update_data(self, objEvent):
        """AMR 위치 및 상태 정보 업데이트"""
        # 위치 정보 업데이트
        if hasattr(objEvent, 'x') and hasattr(objEvent, 'y'):
            self.amr_position = (objEvent.x, objEvent.y)
            print(
                f"[{self.getTime()}][GlobalPlanner] AMR position updated: {self.amr_position}")

            if hasattr(objEvent, 'strID'):
                self.amr_id = objEvent.strID

            # 경로 계획 필요 여부 확인
            if self.getStateValue("state") == "GPP" and self.task_goal and not self.planned_path:
                print(
                    f"[{self.getTime()}][GlobalPlanner] Position received, ready for path planning")

        # 추가 정보 추출
        if hasattr(objEvent, 'yaw'):
            self.amr_yaw = objEvent.yaw
        if hasattr(objEvent, 'lin_vel'):
            self.amr_velocity = objEvent.lin_vel

    def _preprocess_obstacles(self):
        """
        시뮬레이션 시작 시 모든 장애물을 한 번만 전처리
        - Shapely Polygon 객체 생성 및 캐싱
        - Safety margin 적용된 확장 Polygon 생성
        """
        obstacles = self.globalVar.getObstacleInfo()

        # 1. 기본 Polygon 객체 생성
        obstacle_polygons = []
        for obs in obstacles:
            pos = obs['position']
            bbox = obs['boundingBox']
            node_id = obs.get('nodeID', None)

            # WaitingArea는 장애물로 취급하지 않음
            if node_id and isinstance(node_id, str) and node_id.startswith('WAITING_AREA'):
                continue

            x_min = pos['x'] - bbox['width'] / 2
            x_max = pos['x'] + bbox['width'] / 2
            y_min = pos['y'] - bbox['height'] / 2
            y_max = pos['y'] + bbox['height'] / 2

            polygon = Polygon([
                (x_min, y_min),
                (x_max, y_min),
                (x_max, y_max),
                (x_min, y_max)
            ])

            # nodeID도 함께 저장 (제외용)
            obstacle_polygons.append({
                'polygon': polygon,
                'nodeID': node_id
            })

        self.cached_obstacle_polygons = obstacle_polygons

        # 2. Safety margin 적용된 확장 Polygon 생성
        expanded_polygons = []
        for obs_data in obstacle_polygons:
            expanded = obs_data['polygon'].buffer(self.cache_safety_margin)
            expanded_polygons.append({
                'polygon': expanded,
                'nodeID': obs_data['nodeID']
            })

        self.cached_expanded_polygons = expanded_polygons

        print(
            f"[GlobalPlanner] Preprocessed {len(obstacle_polygons)} obstacles (cached)")

    def get_obstacle_polygons(self, exclude_nodes=None):
        """
        GlobalVar에서 장애물 정보 가져오기

        Args:
            exclude_nodes: 제외할 nodeID 리스트 (현재 목표 노드는 장애물에서 제외)
        """
        obstacles = self.globalVar.getObstacleInfo()
        obstacle_polygons = []

        if exclude_nodes is None:
            exclude_nodes = []

        for obs in obstacles:
            # 현재 목표 노드는 장애물에서 제외
            if 'nodeID' in obs and obs['nodeID'] in exclude_nodes:
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][GlobalPlanner] Excluding target node {obs['nodeID']} from obstacles"
                )
                continue

            # WaitingArea는 장애물로 취급하지 않음
            node_id = obs.get('nodeID', None)
            if node_id and isinstance(node_id, str) and node_id.startswith('WAITING_AREA'):
                continue

            pos = obs['position']
            bbox = obs['boundingBox']

            x_min = pos['x'] - bbox['width'] / 2
            x_max = pos['x'] + bbox['width'] / 2
            y_min = pos['y'] - bbox['height'] / 2
            y_max = pos['y'] + bbox['height'] / 2

            # 좌표 리스트로 변환 (Shapely Polygon 변환용)
            polygon_coords = [
                (x_min, y_min),
                (x_max, y_min),
                (x_max, y_max),
                (x_min, y_max)
            ]
            obstacle_polygons.append(polygon_coords)

        return obstacle_polygons

    def generate_path_with_cached_obstacles(self, start, goal, exclude_nodes=None):
        """
        캐시된 장애물로 경로 생성 (최적화 버전)
        """
        if exclude_nodes is None:
            exclude_nodes = []

        # 캐시된 확장 폴리곤 사용 (제외 노드 필터링)
        expanded_polygons = []
        for obs_data in self.cached_expanded_polygons:
            # 제외할 노드는 스킵
            if obs_data['nodeID'] and obs_data['nodeID'] in exclude_nodes:
                continue
            expanded_polygons.append(obs_data['polygon'])

        # 시작점이 포함된 확장 장애물은 제거하여 경로계획을 진행
        start_point = Point(start)
        cleaned_expanded = []
        for polygon in expanded_polygons:
            if not polygon.contains(start_point):
                cleaned_expanded.append(polygon)
        expanded_polygons = cleaned_expanded

        # ✅ 최적화: 먼저 직선 경로가 가능한지 체크 (매우 빠름!)
        if line_of_sight(start, goal, expanded_polygons):
            self.globalVar.printTerminal(
                f"[{self.getTime()}][GlobalPlanner] Direct path available, skipping pathfinding"
            )
            self.path = [goal]  # 목표점만 반환
            return self.path

        # 경로 계획 알고리즘 선택
        raw_path = None
        if self.algorithm == "A_star":
            raw_path = A_star(start, goal, expanded_polygons)
        elif self.algorithm == "RRT":
            raw_path = RRT(start, goal, expanded_polygons,
                           max_iter=1000, step_size=2.0, goal_sample_rate=15)
        elif self.algorithm == "PRM":
            raw_path = PRM(start, goal, expanded_polygons)
        elif self.algorithm == "Theta_star":
            raw_path = Theta_star(start, goal, expanded_polygons)
        else:
            # 기본: 직선 경로
            raw_path = [start, goal]

        if raw_path:
            # Line of Sight를 이용한 경로 단순화
            simplified_path = simplify_path_with_los(
                raw_path, expanded_polygons)

            # 시작점 제외 (첫 waypoint부터 전송)
            if len(simplified_path) == 1:
                self.path = simplified_path
            else:
                self.path = simplified_path[1:]

            # # 경로 시각화 (활성화)
            # self.plot_path_with_terrain(
            #     start, goal, simplified_path, expanded_polygons)
            return self.path
        else:
            print("[GlobalPlanner] No path found between start and goal")
            return []

    def generate_path(self, start, goal, terrain_polygons):
        """경로 생성 (기존 호환성 유지 - 하지만 캐시 사용)"""
        # 이 함수는 get_obstacle_polygons에서 호출되므로
        # exclude_nodes 정보를 terrain_polygons에서 유추해야 함
        # 하지만 더 나은 방법은 직접 exclude_nodes를 전달하는 것

        # terrain_polygons를 Shapely Polygon 객체로 변환
        polygons = [Polygon(p) for p in terrain_polygons]

        # 안전 마진 추가
        safety_margin = 3.0
        expanded_polygons = []
        for polygon in polygons:
            expanded_polygons.append(polygon.buffer(safety_margin))

        # 시작점이 장애물 내부에 있는지 확인
        start_point = Point(start)
        goal_point = Point(goal)

        # 시작점이 포함된 확장 장애물은 제거하여 경로계획을 진행
        cleaned_expanded = []
        for polygon in expanded_polygons:
            if not polygon.contains(start_point):
                cleaned_expanded.append(polygon)
        expanded_polygons = cleaned_expanded
        # goal_point는 체크하지 않음 - exclude_nodes로 이미 제외됨

        # 경로 계획 알고리즘 선택
        raw_path = None
        if self.algorithm == "A_star":
            raw_path = A_star(start, goal, expanded_polygons)
        elif self.algorithm == "RRT":
            raw_path = RRT(start, goal, expanded_polygons)
        elif self.algorithm == "PRM":
            raw_path = PRM(start, goal, expanded_polygons)
        elif self.algorithm == "Theta_star":
            raw_path = Theta_star(start, goal, expanded_polygons)
        else:
            # 기본: 직선 경로
            raw_path = [start, goal]

        if raw_path:
            # Line of Sight를 이용한 경로 단순화
            simplified_path = simplify_path_with_los(
                raw_path, expanded_polygons)

            # 시작점 제외 (첫 waypoint부터 전송)
            if len(simplified_path) == 1:
                self.path = simplified_path
            else:
                self.path = simplified_path[1:]

            # self.plot_path_with_terrain(
            #     start, goal, simplified_path, terrain_polygons)

            return self.path
        else:
            print("[GlobalPlanner] No path found between start and goal")
            return []

    def plot_path_with_terrain(self, start, goal, path, terrain_polygons):
        """경로 시각화 (Shapely Polygon 지원) - 전체 맵 크기 자동 계산"""
        fig, ax = plt.subplots(figsize=(14, 10))

        # 전체 맵 범위 계산을 위한 좌표 수집
        all_x = []
        all_y = []

        # 장애물 그리기 (Shapely Polygon → matplotlib Polygon 변환)
        for polygon in terrain_polygons:
            # Shapely Polygon인 경우 좌표 추출
            if isinstance(polygon, Polygon):
                coords = list(polygon.exterior.coords)
            else:
                # 이미 좌표 리스트인 경우 그대로 사용
                coords = polygon

            # 좌표 수집
            for coord in coords:
                all_x.append(coord[0])
                all_y.append(coord[1])

            poly_patch = pltPolygon(
                coords,
                closed=True,
                facecolor='gray',
                edgecolor='black',
                alpha=0.7
            )
            ax.add_patch(poly_patch)

        # 시작점/목표점
        ax.plot(start[0], start[1], 'go', markersize=10, label='Start')
        ax.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')
        all_x.extend([start[0], goal[0]])
        all_y.extend([start[1], goal[1]])

        # 경로
        if path:
            path_array = np.array(path)
            ax.plot(path_array[:, 0], path_array[:, 1],
                    'b-', linewidth=2, label='Path')
            all_x.extend(path_array[:, 0].tolist())
            all_y.extend(path_array[:, 1].tolist())

        # 전체 맵 범위 계산 (여유 공간 5.0 추가 - visualize.py와 동일)
        if all_x and all_y:
            min_x, max_x = min(all_x), max(all_x)
            min_y, max_y = min(all_y), max(all_y)
            margin = 5.0
            ax.set_xlim(min_x - margin, max_x + margin)
            ax.set_ylim(min_y - margin, max_y + margin)
        else:
            # 데이터가 없으면 기본값 사용
            ax.set_xlim(0, self.map_width)
            ax.set_ylim(0, self.map_height)
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.legend(loc='upper right')
        ax.set_title(f'Global Path Planning - {self.algorithm}')
        ax.grid(True, linestyle='--', alpha=0.3)
        ax.set_facecolor('#F5F5F5')

        plt.savefig(f'global_path_{self.getTime()}.png',
                    dpi=300, bbox_inches='tight')
        plt.close()


def simplify_path_with_los(path, terrain_polygons):
    """Line of Sight를 이용한 경로 단순화"""
    if not path or len(path) < 2:
        return path

    simplified_path = [path[0]]
    current_index = 0

    while current_index < len(path) - 1:
        last_visible = current_index + 1

        for next_index in range(current_index + 2, len(path)):
            if line_of_sight(path[current_index], path[next_index], terrain_polygons):
                last_visible = next_index
            else:
                break

        simplified_path.append(path[last_visible])
        current_index = last_visible

    if simplified_path[-1] != path[-1]:
        simplified_path.append(path[-1])

    return simplified_path


def line_of_sight(p1, p2, terrain_polygons):
    """두 점 사이에 장애물이 없는지 확인"""
    line = LineString([p1, p2])

    for polygon in terrain_polygons:
        # polygon이 이미 Shapely Polygon이면 그대로 사용
        if isinstance(polygon, Polygon):
            if line.intersects(polygon):
                return False
        else:
            # 좌표 리스트면 Polygon으로 변환
            if line.intersects(Polygon(polygon)):
                return False

    return True
