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

        # AMR과 작업 정보
        self.amr_id = None
        self.amr_position = None
        self.amr_yaw = None
        self.amr_velocity = None
        self.task_id = None
        self.task_from = None  # 운반 명령의 픽업 위치 (장비 IN)
        self.task_to = None    # 운반 명령의 목적지 위치 (장비 IN)
        self.task_from_nodeID = None  # 픽업 장비의 NodeID
        self.task_to_nodeID = None    # 목적지 장비의 NodeID
        self.planned_path = []
        self.path = []
        self.start = None

        # 운반 단계
        # TO_FROM:        픽업 장비로 이동 중
        # AT_FROM:        픽업 장비 도착, 적재 대기
        # TO_DESTINATION: 목적지 장비로 이동 중
        # AT_DESTINATION: 목적지 장비 도착, 하역 완료
        self.transportPhase = None
        self.currentGoal = None  # 현재 단계의 목표 위치
        self.currentGoalNodeID = None  # 현재 단계의 목표 NodeID

        # 장애물 전처리 캐시
        self.cached_obstacle_polygons = None  # Shapely Polygon 캐시
        self.cached_expanded_polygons = None  # 안전 마진을 적용한 Polygon 캐시
        self.cache_safety_margin = 3.0

        # 장애물은 시작 시 한 번만 전처리한다
        self._preprocess_obstacles()

        # 상태 변수
        self.addStateVariable("state", "INIT")

        # 포트
        self.addInputPort("Task_I")           # 작업 지시
        # 단순 이동 명령 (jobID 없음)
        self.addInputPort("amrGoCommand")
        self.addInputPort("ManeuverState_I")  # AMR 위치
        self.addInputPort("Replan")           # 재계획 요청
        self.addInputPort("StopSim")          # 시뮬레이션 중지

        self.addOutputPort("GlobalWaypoint_O")  # waypoint 전송
        self.addOutputPort("RequestManeuver")   # Maneuver 요청

    def funcExternalTransition(self, strPort, objEvent):
        state = self.getStateValue("state")

        # 도킹 완료. 작업을 마치고 대기 상태로 돌아간다
        if strPort == "Docking_I":
            self.setStateValue("state", "WAIT")
            self.globalVar.printTerminal(
                f"[{self.getTime()}][GlobalPlanner({self.ID})] Docking/WaitingArea completed, now WAITING for next task"
            )
            return
        if strPort == "Undocking_I":
            self.setStateValue("state", "WAIT")
        # amrGoCommand는 비켜 달라는 명령이므로 어느 상태에서든 받는다
        if strPort == "amrGoCommand":
            # FleetManagement가 보낸 단순 이동 명령 (jobID 없음)
            if self.ID.split('_', 1)[0] == objEvent['amrID'].split('_', 1)[0]:
                planner_vehicle_id = self.ID.split('_')[0]
                task_vehicle_id = objEvent['amrID'].split('_')[0] if isinstance(
                    objEvent, dict) and 'amrID' in objEvent else None

                if task_vehicle_id and planner_vehicle_id in task_vehicle_id:
                    action = objEvent.get('action', 'GO')

                    if action == 'GO_WAITING':
                        # 대기 구역으로 이동
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] 🚨 EMERGENCY GO_WAITING Command (State: {state}) for AMR {objEvent['amrID']}"
                        )

                        # 명령에서 목표를 읽는다
                        self.amr_id = objEvent['amrID']
                        self.task_id = None  # jobID 없음

                        # 목표 위치만 설정한다
                        target_x = objEvent['x']
                        target_y = objEvent['y']
                        areaID = objEvent.get('areaID', None)

                        # 현재 위치를 시작점으로 삼는다
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

                        # WAITING 단계로 둔다
                        self.transportPhase = "WAITING"
                        self.currentGoal = self.task_to
                        self.currentGoalNodeID = areaID  # WaitingArea ID

                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] GO_WAITING Command: Move to WaitingArea {areaID} at ({target_x}, {target_y})"
                        )

                        # 경로 계획 상태로 전이
                        self.setStateValue("state", "GPP")
                        return  # 즉시 처리
                    else:
                        # 일반 이동 명령
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] 🚨 EMERGENCY GO Command (State: {state}) for AMR {objEvent['amrID']}"
                        )

                        # 명령에서 목표를 읽는다
                        self.amr_id = objEvent['amrID']
                        self.task_id = None  # jobID 없음

                        # 현재 위치에서 목표 위치까지만 계획한다
                        target_x = objEvent['x']
                        target_y = objEvent['y']

                        # 현재 위치를 시작점으로 삼는다
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

                        # 단순 이동이라 운반 단계가 없다
                        self.transportPhase = None
                        self.currentGoal = self.task_to
                        self.currentGoalNodeID = None

                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] GO Command - Simple movement (no job)"
                        )

                        # 경로 계획 상태로 전이
                        self.setStateValue("state", "GPP")
                        return  # 즉시 처리
                else:
                    # 다른 AMR의 명령이면 무시한다
                    self.continueTimeAdvance()
            return  # amrGoCommand 처리 끝

        # 새 작업은 INIT과 WAIT에서만 받는다
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

                        # 작업 정보를 읽는다
                        self.amr_id = objEvent['amrID']
                        self.task_id = objEvent['jobID']

                        # 픽업과 목적지의 위치와 NodeID를 보관한다
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

                        # 운반 동작에 따라 시작 단계를 정한다
                        action = objEvent.get('action', 'TRANSPORT')

                        if action == 'TRANSPORT_NEXT':
                            # 언도킹 후에는 곧바로 TO_DESTINATION 단계로 간다
                            self.transportPhase = "TO_DESTINATION"
                            self.currentGoal = self.task_to
                            self.currentGoalNodeID = self.task_to_nodeID

                            # 언도킹 후 AMR 위치를 최신값으로 갱신한다
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
                            # 일반 운반은 픽업 장비로 먼저 간다
                            self.transportPhase = "TO_FROM"
                            self.currentGoal = self.task_from
                            self.currentGoalNodeID = self.task_from_nodeID

                            self.globalVar.printTerminal(
                                f"[{self.getTime()}][GlobalPlanner({self.ID})] Phase: TO_FROM - Moving to pickup location"
                            )

                        # 경로 계획 상태로 전이
                        self.setStateValue("state", "GPP")
                    else:
                        # 다른 AMR의 작업이면 무시한다
                        self.continueTimeAdvance()
            elif strPort == "Replan":
                # Local_Planner가 보낸 재계획 요청
                # 차량 번호를 비교한다

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
                # AMR 위치 갱신
                if self.ID.split('_', 1)[0] == objEvent.strID:
                    self.update_data(objEvent)
                else:
                    self.continueTimeAdvance()

        # PLAN 상태에서는 주행 중이므로 Replan과 ManeuverState_I만 받는다
        elif state == "PLAN":
            if strPort == "Task_I":
                # 주행 중에는 새 작업을 무시한다
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][GlobalPlanner({self.ID})] BUSY - Ignoring new task (currently in PLAN state)"
                )
                self.continueTimeAdvance()

            elif strPort == "Replan":
                # Local_Planner가 보낸 재계획 요청
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
                # AMR 위치 갱신
                if self.ID.split('_', 1)[0] == objEvent.strID:
                    self.update_data(objEvent)
                else:
                    self.continueTimeAdvance()

        # 그 밖의 상태에서는 ManeuverState_I만 받는다
        elif strPort == "ManeuverState_I":
            # 위치는 어느 상태에서든 갱신한다
            if self.ID.split('_', 1)[0] == objEvent.strID:
                self.update_data(objEvent)
            else:
                self.continueTimeAdvance()

    def funcInternalTransition(self):
        state = self.getStateValue("state")

        if state == "GPP" and self.currentGoal is not None:
            # 경로 계획
            if self.amr_position and self.currentGoal:
                # 목표 노드와 시작 노드는 장애물에서 뺀다
                exclude_nodes = []

                # 목표 노드 제외. 대기 구역도 포함한다
                if self.currentGoalNodeID:
                    exclude_nodes.append(self.currentGoalNodeID)
                    # 대기 구역은 접두어만 같아도 제외한다
                    if isinstance(self.currentGoalNodeID, str) and self.currentGoalNodeID.startswith('WAITING_AREA'):
                        exclude_nodes.append(self.currentGoalNodeID)

                # 시작 노드 제외. AMR이 서 있는 장비의 outputPort다
                if self.transportPhase == "TO_DESTINATION" and self.task_from_nodeID:
                    # TO_DESTINATION 단계에서는 이전 장비의 outputPort에서 출발한다
                    start_nodeID = self.task_from_nodeID.replace('_IN', '_OUT')
                    exclude_nodes.append(start_nodeID)
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][GlobalPlanner] Excluding start node {start_nodeID} from obstacles"
                    )

                start_time = time.time()
                # 캐시된 장애물로 경로를 계획한다
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
                    # 경로를 찾지 못하면 직선으로 잇는다
                    print(
                        f"[GlobalPlanner] No path found to {phase_name}, using direct path")
                    self.planned_path = [
                        (self.currentGoal[0], self.currentGoal[1])]

                # 경로 전송 상태로 전이
                self.setStateValue("state", "SEND_PATH")

        elif state == "SEND_PATH":
            # 출력한 뒤 PLAN으로 전이
            self.setStateValue("state", "PLAN")

    def funcOutput(self):
        state = self.getStateValue("state")

        if state == "SEND_PATH" and self.planned_path and len(self.planned_path) > 0:
            # 첫 waypoint를 보낸다
            first_waypoint = self.planned_path[0]
            x, y = first_waypoint[0], first_waypoint[1]

            phase_name = "FROM Equipment" if self.transportPhase == "TO_FROM" else "TO Equipment"
            print(
                f"[{self.getTime()}][GlobalPlanner] Sending waypoint: ({x}, {y}) - Phase: {phase_name}")

            # 운반 단계와 목표 NodeID를 함께 담아 보낸다
            objRequestMessage = MsgManeuverState(
                self.ID.split('_', 1)[0] + '_maneuver_amr',
                x,  # dblPositionX
                y,  # dblPositionY
                path=self.planned_path,  # 전체 경로
                transportPhase=self.transportPhase,  # "TO_FROM" or "TO_DESTINATION"
                goalNodeID=self.currentGoalNodeID  # 현재 목표 NodeID
            )

            # 전송
            self.addOutputEvent("GlobalWaypoint_O", objRequestMessage)

    def funcTimeAdvance(self):
        state = self.getStateValue("state")

        if state == "INIT":
            return float('inf')  # 첫 작업을 기다린다
        elif state == "WAIT":
            return float('inf')  # 다음 작업을 기다린다
        elif state == "GPP":
            return 0  # 즉시 경로를 계획한다
        elif state == "SEND_PATH":
            return 0  # 즉시 경로를 보낸다
        elif state == "PLAN":
            return float('inf')  # AMR이 주행 중이다
        else:
            return 1

    def extract_vehicle_number(self, id_string):
        """모델 ID에서 차량 번호를 뽑는다."""
        match = re.search(r'Vehicle(\d+)', id_string)
        if match:
            return match.group(1)
        return None

    def update_data(self, objEvent):
        """AMR의 위치와 상태를 갱신한다."""
        # 위치 갱신
        if hasattr(objEvent, 'x') and hasattr(objEvent, 'y'):
            self.amr_position = (objEvent.x, objEvent.y)
            print(
                f"[{self.getTime()}][GlobalPlanner] AMR position updated: {self.amr_position}")

            if hasattr(objEvent, 'strID'):
                self.amr_id = objEvent.strID

            # 재계획이 필요한지 확인한다
            if self.getStateValue("state") == "GPP" and self.task_goal and not self.planned_path:
                print(
                    f"[{self.getTime()}][GlobalPlanner] Position received, ready for path planning")

        # 부가 정보
        if hasattr(objEvent, 'yaw'):
            self.amr_yaw = objEvent.yaw
        if hasattr(objEvent, 'lin_vel'):
            self.amr_velocity = objEvent.lin_vel

    def _preprocess_obstacles(self):
        """시작 시 모든 장애물을 한 번만 전처리한다.

        Shapely Polygon을 만들어 캐시하고, 안전 마진을 적용한 확장
        Polygon도 함께 만들어 둔다.
        """
        obstacles = self.globalVar.getObstacleInfo()

        # 1. 기본 Polygon
        obstacle_polygons = []
        for obs in obstacles:
            pos = obs['position']
            bbox = obs['boundingBox']
            node_id = obs.get('nodeID', None)

            # 대기 구역은 장애물로 두지 않는다
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

            # 제외 판정을 위해 nodeID도 함께 둔다
            obstacle_polygons.append({
                'polygon': polygon,
                'nodeID': node_id
            })

        self.cached_obstacle_polygons = obstacle_polygons

        # 2. 안전 마진을 적용한 확장 Polygon
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
        """GlobalVar에서 장애물을 읽는다.

        exclude_nodes에 든 nodeID는 장애물에서 뺀다. 현재 목표 노드가
        장애물로 남으면 그 안으로 들어갈 수 없기 때문이다.
        """
        obstacles = self.globalVar.getObstacleInfo()
        obstacle_polygons = []

        if exclude_nodes is None:
            exclude_nodes = []

        for obs in obstacles:
            # 현재 목표 노드는 장애물에서 뺀다
            if 'nodeID' in obs and obs['nodeID'] in exclude_nodes:
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][GlobalPlanner] Excluding target node {obs['nodeID']} from obstacles"
                )
                continue

            # 대기 구역은 장애물로 두지 않는다
            node_id = obs.get('nodeID', None)
            if node_id and isinstance(node_id, str) and node_id.startswith('WAITING_AREA'):
                continue

            pos = obs['position']
            bbox = obs['boundingBox']

            x_min = pos['x'] - bbox['width'] / 2
            x_max = pos['x'] + bbox['width'] / 2
            y_min = pos['y'] - bbox['height'] / 2
            y_max = pos['y'] + bbox['height'] / 2

            # Shapely Polygon으로 넘기기 위해 좌표 리스트로 바꾼다
            polygon_coords = [
                (x_min, y_min),
                (x_max, y_min),
                (x_max, y_max),
                (x_min, y_max)
            ]
            obstacle_polygons.append(polygon_coords)

        return obstacle_polygons

    def generate_path_with_cached_obstacles(self, start, goal, exclude_nodes=None):
        """캐시된 장애물로 전역 경로를 만든다."""
        if exclude_nodes is None:
            exclude_nodes = []

        # 캐시된 확장 폴리곤을 쓰되 제외 노드는 거른다
        expanded_polygons = []
        for obs_data in self.cached_expanded_polygons:
            # 제외 노드는 건너뛴다
            if obs_data['nodeID'] and obs_data['nodeID'] in exclude_nodes:
                continue
            expanded_polygons.append(obs_data['polygon'])

        # 시작점을 품은 확장 장애물은 빼야 경로 계획이 가능하다
        start_point = Point(start)
        cleaned_expanded = []
        for polygon in expanded_polygons:
            if not polygon.contains(start_point):
                cleaned_expanded.append(polygon)
        expanded_polygons = cleaned_expanded

        # 직선으로 갈 수 있으면 탐색을 건너뛴다
        if line_of_sight(start, goal, expanded_polygons):
            self.globalVar.printTerminal(
                f"[{self.getTime()}][GlobalPlanner] Direct path available, skipping pathfinding"
            )
            self.path = [goal]  # 목표점만 반환한다
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
            # 기본값은 직선
            raw_path = [start, goal]

        if raw_path:
            # 직시 검사로 경로를 단순화한다
            simplified_path = simplify_path_with_los(
                raw_path, expanded_polygons)

            # 시작점은 빼고 첫 waypoint부터 보낸다
            if len(simplified_path) == 1:
                self.path = simplified_path
            else:
                self.path = simplified_path[1:]

            # # 경로 시각화
            # self.plot_path_with_terrain(
            #     start, goal, simplified_path, expanded_polygons)
            return self.path
        else:
            print("[GlobalPlanner] No path found between start and goal")
            return []

    def generate_path(self, start, goal, terrain_polygons):
        """전역 경로를 만든다. 캐시를 쓰지 않는 호환용 경로다."""
        # 이 함수는 get_obstacle_polygons를 거쳐 호출되므로 exclude_nodes를
        # terrain_polygons에서 유추해야 한다. 직접 넘기는 편이 낫다

        # terrain_polygons를 Shapely Polygon으로 바꾼다
        polygons = [Polygon(p) for p in terrain_polygons]

        # 안전 마진 적용
        safety_margin = 3.0
        expanded_polygons = []
        for polygon in polygons:
            expanded_polygons.append(polygon.buffer(safety_margin))

        # 시작점이 장애물 안에 있는지 확인한다
        start_point = Point(start)
        goal_point = Point(goal)

        # 시작점을 품은 확장 장애물은 빼야 경로 계획이 가능하다
        cleaned_expanded = []
        for polygon in expanded_polygons:
            if not polygon.contains(start_point):
                cleaned_expanded.append(polygon)
        expanded_polygons = cleaned_expanded
        # 목표점은 exclude_nodes에서 이미 빠졌으므로 검사하지 않는다

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
            # 기본값은 직선
            raw_path = [start, goal]

        if raw_path:
            # 직시 검사로 경로를 단순화한다
            simplified_path = simplify_path_with_los(
                raw_path, expanded_polygons)

            # 시작점은 빼고 첫 waypoint부터 보낸다
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
        """경로를 그린다. 맵 범위는 좌표에서 자동으로 계산한다."""
        fig, ax = plt.subplots(figsize=(14, 10))

        # 맵 범위를 계산하기 위해 좌표를 모은다
        all_x = []
        all_y = []

        # 장애물을 그린다
        for polygon in terrain_polygons:
            # Shapely Polygon이면 좌표를 꺼낸다
            if isinstance(polygon, Polygon):
                coords = list(polygon.exterior.coords)
            else:
                # 이미 좌표 리스트면 그대로 쓴다
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

        # 시작점과 목표점
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

        # 맵 범위. 여유 5.0은 visualize.py와 맞춘 값이다
        if all_x and all_y:
            min_x, max_x = min(all_x), max(all_x)
            min_y, max_y = min(all_y), max(all_y)
            margin = 5.0
            ax.set_xlim(min_x - margin, max_x + margin)
            ax.set_ylim(min_y - margin, max_y + margin)
        else:
            # 좌표가 없으면 기본값을 쓴다
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
    """직시 가능한 구간을 이어 붙여 경로를 단순화한다."""
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
    """두 점 사이에 장애물이 없는지 확인한다."""
    line = LineString([p1, p2])

    for polygon in terrain_polygons:
        # 이미 Shapely Polygon이면 그대로 쓴다
        if isinstance(polygon, Polygon):
            if line.intersects(polygon):
                return False
        else:
            # 좌표 리스트면 Polygon으로 바꾼다
            if line.intersects(Polygon(polygon)):
                return False

    return True
