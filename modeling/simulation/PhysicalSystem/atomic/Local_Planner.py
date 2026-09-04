from SimulationEngine.ClassicDEVS.DEVSAtomicModel import DEVSAtomicModel

from Algorithm.PathPlanning.Local_path.DWA import DWAPlanner
import numpy as np
from shapely.geometry import Point, Polygon
from modeling.Message.MsgManeuverState import MsgManeuverState
import math
import time
from modeling.simulation.PhysicalSystem import Vehicle


class LocalPlanner(DEVSAtomicModel):
    def __init__(self, ID, objConfiguration, globalVar):
        super().__init__(ID)

        # Configuration
        self.objConfiguration = objConfiguration
        self.globalVar = globalVar
        self.safety_tolerance = 8.0  # 다른 AMR 감지 거리 (8 m)
        self.collision_radius = 3.0  # 충돌 회피 반경 (3 m)

        # State variables
        self.goal = None
        self.curPose = [0, 0, 0, 0, 0]  # x, y, yaw, lin_vel, ang_vel
        self.obstacles = {}
        # {agent_id: {'pos': (x,y), 'vel': (vx,vy), 'time': t}}
        self.other_agents = {}

        # 운반 단계
        self.transportPhase = None  # "TO_FROM" or "TO_DESTINATION"
        self.path = []  # 전체 경로
        self.currentGoalNodeID = None  # 현재 목표 NodeID. 장애물 제외에 쓴다
        self.previousEquipmentID = None  # 직전 장비 ID. 언도킹 후 장애물 제외에 쓴다
        self.yaw_forced = False  # yaw 고정을 이미 적용했는지
        self.needs_yaw_force = False  # 언도킹 직후 yaw 고정이 필요한지

        # 정적 장애물 캐시
        self.cached_static_polygons = None  # 캐시된 Polygon
        self.cached_exclude_nodes = None  # 캐시를 만들 때 제외한 노드

        # Initialize DWA Planner
        self.dwa = DWAPlanner(objConfiguration)

        # Target state
        self.target_x = 0
        self.target_y = 0
        self.target_yaw = 0

        # Replan check variables
        self.replan_check_start_time = None
        self.replan_check_initial_distance = None
        self.replan_check_initial_position = None
        self.replan_check_threshold_time = 5.0  # 5초 동안 진전이 없으면 재계획한다
        self.replan_check_distance_improvement = 2.0  # 거리가 2 m 이상 줄어야 진전으로 본다
        self.replan_check_position_movement = 1.0  # 1 m 이상 움직여야 진전으로 본다

        # Logging variables
        self._last_pose_log_time = 0
        self._last_dwa_log_time = 0

        # 회피 상태
        self.is_in_avoidance = False  # 회피 모드 여부
        self.avoidance_clearance_threshold = self.safety_tolerance + \
            2.0  # 회피 해제 임계값. 히스테리시스를 준다

        # DEVS state
        self.addStateVariable("state", "WAIT")

        # Input Ports
        self.addInputPort("GlobalWaypoint_I")  # Global_Planner의 waypoint
        self.addInputPort("agent_pose_I")      # 다른 에이전트와 장애물 위치
        self.addInputPort("ManeuverState_I")   # 자기 위치
        self.addInputPort("OtherManeuverState_I")  # 다른 AMR 위치

        # Output Ports
        self.addOutputPort("RequestManeuver_O")  # Maneuver로 보내는 목표
        self.addOutputPort("Replan")             # Global_Planner로 보내는 재계획 요청
        self.addOutputPort("DeliveryComplete")   # 목적지 장비 도착, 하역 완료
        self.addOutputPort("Docking")            # Maneuver로 보내는 도킹 명령
        self.addOutputPort("EquipmentDocking")   # Equipment로 보내는 도킹 알림
        self.addOutputPort("UndockingComplete")  # FleetManagement로 보내는 언도킹 완료
        self.addOutputPort("Undocking_O")        # Maneuver로 보내는 언도킹 명령

    def funcExternalTransition(self, strPort, objEvent):
        state = self.getStateValue("state")

        if strPort == "GlobalWaypoint_I":
            # Global_Planner가 보낸 waypoint (MsgGoal)
            if hasattr(objEvent, 'strID') and objEvent.strID.split('_', 1)[0] == self.ID.split('_', 1)[0]:
                # 운반 명령에 따라 픽업 또는 목적지 장비의 IN 포트로 향한다
                self.goal = (objEvent.dblPositionX, objEvent.dblPositionY)

                # Global_Planner의 운반 단계를 받아 둔다
                self.transportPhase = objEvent.transportPhase if objEvent.transportPhase else None

                # 경로 저장
                self.path = objEvent.path if objEvent.path else []

                # 현재 목표 NodeID. 장애물 제외에 쓴다
                self.currentGoalNodeID = objEvent.goalNodeID if objEvent.goalNodeID else None

                self.globalVar.printTerminal(
                    f"[{self.getTime()}][LocalPlanner({self.ID})] Received waypoint: {self.goal}, Phase: {self.transportPhase}, NodeID: {self.currentGoalNodeID}"
                )

                # TO_DESTINATION이 아니면 previousEquipmentID를 비운다
                if self.transportPhase == "TO_FROM" or self.transportPhase == "WAITING":
                    self.previousEquipmentID = None
                    # TO_FROM과 WAITING 단계에서는 yaw를 고정할 필요가 없다
                    self.needs_yaw_force = False
                    self.yaw_forced = False

                # 새 목표를 받았으니 진전도 추적을 초기화한다
                self.reset_replan_check()

                # 새 전역 경로이므로 회피 모드를 푼다
                if self.is_in_avoidance:
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][LocalPlanner({self.ID})] 새로운 경로 수신! AVOIDANCE 모드 종료"
                    )
                    self.is_in_avoidance = False

                # 목표가 바뀌었으므로 정적 장애물 캐시를 버린다
                self.cached_static_polygons = None
                self.cached_exclude_nodes = None

                self.setStateValue("state", "PLAN")

        elif strPort == "ManeuverState_I":
            # 자기 위치 갱신
            event_id = objEvent.strID.split('_', 1)[0]
            my_id = self.ID.split('_', 1)[0]

            # 로그는 1초에 한 번만
            should_log = (self.getTime() - self._last_pose_log_time >= 1.0)

            if event_id == my_id:
                old_pose = self.curPose.copy()
                self.curPose[0] = objEvent.x
                self.curPose[1] = objEvent.y
                self.curPose[2] = objEvent.yaw
                self.curPose[3] = objEvent.lin_vel
                self.curPose[4] = objEvent.ang_vel

                if should_log:
                    print(
                        f"   ✅ UPDATED: ({old_pose[0]:.2f}, {old_pose[1]:.2f}) → ({self.curPose[0]:.2f}, {self.curPose[1]:.2f})")
                    print(f"   Current goal: {self.goal}")
                    if self.goal:
                        distance = np.linalg.norm(
                            np.array(self.goal) - np.array(self.curPose[:2]))
                        print(f"   Distance to goal: {distance:.2f}m")
                    self._last_pose_log_time = self.getTime()

                # 언도킹 직후 첫 PLAN에서만 yaw를 90도로 한 번 고정한다
                # 장비 언도킹과 대기 구역 언도킹을 모두 다룬다
                if state == "PLAN" and (self.needs_yaw_force or (self.previousEquipmentID and not self.yaw_forced)):
                    self.curPose[2] = np.pi / 2  # 90도로 고정
                    self.yaw_forced = True  # 한 번만 실행되도록 표시
                    self.needs_yaw_force = False  # 요청 소진
                    source = f"Equipment {self.previousEquipmentID}" if self.previousEquipmentID else "WaitingArea"
                    print(
                        f"   🔄 [ONCE] Forced yaw to 90° after UNDOCKING from {source}")

                self.continueTimeAdvance()
            else:
                if should_log:
                    print(f"   ❌ IGNORED: Not my message")
                self.continueTimeAdvance()

        elif strPort == "OtherManeuverState_I":
            # 다른 AMR의 위치. 자기 자신은 뺀다
            other_agent_id = objEvent.strID.split(
                '_')[0] if '_' in objEvent.strID else objEvent.strID
            my_id = self.ID.split('_', 1)[0]

            if other_agent_id != my_id:
                # MsgManeuverState는 dblPositionX와 dblPositionY를 쓴다
                current_time = self.getTime()
                other_pos = (objEvent.dblPositionX, objEvent.dblPositionY)

                # 이전 위치가 있으면 위치 변화로 속도를 구한다
                other_vel = (0.0, 0.0)  # 기본값
                if other_agent_id in self.other_agents:
                    prev_data = self.other_agents[other_agent_id]
                    prev_pos = prev_data['pos']
                    prev_time = prev_data['time']
                    dt = current_time - prev_time

                    if dt > 0:
                        # 위치 변화에서 실제 속도를 구한다
                        vx = (other_pos[0] - prev_pos[0]) / dt
                        vy = (other_pos[1] - prev_pos[1]) / dt
                        other_vel = (vx, vy)

                # 다른 AMR 정보 갱신
                self.other_agents[other_agent_id] = {
                    'pos': other_pos,
                    'vel': other_vel,
                    'time': current_time
                }

                # 거리, 속도, 전방 시야로 충돌 위험을 판단한다
                distance = self.check_agent_distance(other_agent_id)

                # 전방 약 120도 안에 있을 때만 장애물 후보로 본다
                fov_rad = 2.0 * np.pi / 3.0  # 120도
                rel_vec = np.array(other_pos) - np.array(self.curPose[:2])
                angle_to_other = math.atan2(rel_vec[1], rel_vec[0])
                rel_angle = (
                    angle_to_other - self.curPose[2] + math.pi) % (2 * math.pi) - math.pi
                in_front = abs(rel_angle) <= (fov_rad / 2.0)

                if distance < self.safety_tolerance and in_front:
                    # 가까우면 장애물로 등록한다
                    collision_risk = self.predict_collision_risk(
                        other_agent_id)

                    if collision_risk or distance < self.collision_radius:
                        # 충돌 위험이 있거나 매우 가까우면 장애물로 등록한다
                        self.obstacles[other_agent_id] = other_pos

                        if distance < self.collision_radius:
                            self.globalVar.printTerminal(
                                f"[{current_time}][LocalPlanner({self.ID})] ⚠️ 근접 경고: {other_agent_id} at {distance:.2f}m"
                            )

                        # 상태는 그대로 두고 회피 플래그만 세운다
                        if not self.is_in_avoidance:
                            self.globalVar.printTerminal(
                                f"[{current_time}][LocalPlanner({self.ID})] 🚨 장애물 감지! 회피 모드 ON (거리: {distance:.2f}m)"
                            )
                            self.is_in_avoidance = True
                    else:
                        # 위험이 없으면 장애물에서 뺀다
                        if other_agent_id in self.obstacles:
                            del self.obstacles[other_agent_id]
                else:
                    # 전방이 아니거나 안전 거리 밖이면 장애물에서 뺀다
                    if other_agent_id in self.obstacles:
                        del self.obstacles[other_agent_id]

            self.continueTimeAdvance()

        elif strPort == "UndockingComplete_I":
            # objEvent: [amrID, equipmentID, jobID, phase]
            if objEvent[0] == self.ID.split('_')[0]:
                # 이미 대기 구역에 있으면 중복 신호를 무시한다
                is_in_waiting_area = (self.currentGoalNodeID and
                                      self.currentGoalNodeID.startswith('WAITING_AREA') and
                                      self.transportPhase == "TO_DESTINATION")

                if is_in_waiting_area:
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][LocalPlanner({self.ID})] 🅿️ Ignoring UndockingComplete_I - Already in WaitingArea"
                    )
                    self.continueTimeAdvance()
                else:
                    self.setStateValue("state", "UNDOCKING")

    def funcInternalTransition(self):
        state = self.getStateValue("state")

        if state == "PLAN":
            if not self.goal:
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][LocalPlanner({self.ID})] No goal available"
                )
                self.setStateValue("state", "WAIT")
                return

            # 진전이 있는지 확인하고 필요하면 재계획한다
            if self.replan_check():
                self.setStateValue("state", "REPLAN")
                return

            distance_to_goal = self.check_distance()
            obstacle_distance = self.check_obstacle_distance()

            # 장애물이 충분히 멀어졌으면 회피 모드를 푼다
            if self.is_in_avoidance and obstacle_distance > self.avoidance_clearance_threshold:
                self.is_in_avoidance = False
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][LocalPlanner({self.ID})] ✅ 회피 모드 OFF! 장애물 거리: {obstacle_distance:.2f}m"
                )

            # 목표 도달 확인
            target_tolerance = self.objConfiguration.getConfiguration(
                'target_tolerance') or 5.0
            if distance_to_goal < target_tolerance:
                phase_str = self.transportPhase if self.transportPhase else 'Unknown'
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][LocalPlanner({self.ID})] Arrived at waypoint (Phase: {phase_str})"
                )

                # 도달했으면 진전도 추적을 초기화한다
                self.reset_replan_check()

                # 최종 목적지 좌표. 대기 구역과 장비를 함께 다룬다
                is_waiting_area_goal = (self.currentGoalNodeID and
                                        isinstance(self.currentGoalNodeID, str) and
                                        self.currentGoalNodeID.startswith('WAITING_AREA'))

                if is_waiting_area_goal:
                    waiting_area = self.globalVar.getWaitingAreaInfoByID(
                        self.currentGoalNodeID)
                    if waiting_area and 'x' in waiting_area.position and 'y' in waiting_area.position:
                        dest_x = waiting_area.position['x']
                        dest_y = waiting_area.position['y']
                    else:
                        # 폴백: 현재 목표나 현재 위치를 쓴다
                        dest_x = (
                            self.goal[0] if self.goal else self.curPose[0])
                        dest_y = (
                            self.goal[1] if self.goal else self.curPose[1])
                else:
                    equipmentInfo = self.globalVar.getEquipmentInfoByID(
                        self.currentGoalNodeID.split('_')[0])
                    equipmentInfo_inputPort = equipmentInfo.inputPort.get(
                        'position')
                    dest_x = equipmentInfo_inputPort['x']
                    dest_y = equipmentInfo_inputPort['y']

                # 거리 계산을 위해 numpy 배열로 바꾼다
                final_destination = np.array([dest_x, dest_y])
                current_position = np.array(self.curPose[:2])
                distance_to_final = np.linalg.norm(
                    final_destination - current_position)

                if distance_to_final < 5.0:
                    # 최종 목적지 도착이면 DOCKING으로 간다
                    if self.transportPhase == "TO_FROM":
                        # 픽업 장비 도착: 도킹 지시
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][LocalPlanner({self.ID})] 🎯 Arrived at FINAL destination (FROM Equipment)"
                        )
                        self.setStateValue("state", "DOCKING")

                    elif self.transportPhase == "TO_DESTINATION":
                        # 목적지 장비 도착: 도킹 지시
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][LocalPlanner({self.ID})] 🎯 Arrived at FINAL destination (TO Equipment)"
                        )
                        self.setStateValue("state", "DOCKING")
                    elif self.transportPhase == "WAITING":
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][LocalPlanner({self.ID})] 🅿️ Arrived at WaitingArea (FINAL) - Direct UNDOCKING"
                        )
                        self.setStateValue("state", "UNDOCKING")
                        return
                    else:
                        # transportPhase가 없거나 예상 밖이면 재계획한다
                        self.setStateValue("state", "REPLAN")
                    return
                else:
                    # 중간 waypoint 도달: 다음 waypoint로 간다
                    next_waypoint = self.get_next_waypoint()
                    if next_waypoint:
                        self.goal = next_waypoint
                        # 목표가 바뀌었으니 진전도 추적을 초기화한다
                        self.reset_replan_check()
                        # 정적 장애물 캐시도 버린다
                        self.cached_static_polygons = None
                        self.cached_exclude_nodes = None
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][LocalPlanner({self.ID})] ✓ Passed intermediate waypoint, moving to next: {self.goal}"
                        )
                    else:
                        # 다음 waypoint가 없으면 재계획한다
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][LocalPlanner({self.ID})] ⚠️ No next waypoint available, requesting replan"
                        )
                        self.setStateValue("state", "REPLAN")
                    return

            # DWA 실행. 회피 모드면 파라미터가 조정된 채로 돈다
            start_time = time.time()
            target_state = self.calc_dwa()
            end_time = time.time()
            elapsed = end_time - start_time
            self.globalVar.LocalPlanner_algorithm_time += elapsed
            self.globalVar.LocalPlanner_call_count += 1

            if target_state is None:
                # 궤적을 찾지 못하면 재계획을 요청한다
                mode_str = "AVOIDANCE" if self.is_in_avoidance else "PLAN"
                print(f"   ❌ DWA FAILED ({mode_str})")
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][LocalPlanner({self.ID})] DWA failed ({mode_str}), requesting replan"
                )
                self.setStateValue("state", "REPLAN")
                return

            # 목표 갱신
            self.target_x = target_state[0]
            self.target_y = target_state[1]
            self.target_yaw = target_state[2]

            # DWA 결과 로그. 1초에 한 번, 회피 여부를 함께 남긴다
            if self.getTime() - self._last_dwa_log_time >= 1.0:
                current_dist = np.linalg.norm(
                    np.array(self.goal) - np.array(self.curPose[:2]))
                target_dist = np.linalg.norm(
                    np.array(self.goal) - np.array([self.target_x, self.target_y]))

                mode_icon = "🔄" if self.is_in_avoidance else "✅"
                mode_str = "[AVOIDING]" if self.is_in_avoidance else "[NORMAL]"
                print(
                    f"   {mode_icon} DWA Result {mode_str}: ({self.target_x:.2f}, {self.target_y:.2f})")
                print(f"   Current distance to goal: {current_dist:.2f}m")
                print(f"   Target distance to goal: {target_dist:.2f}m")
                print(f"   Obstacle distance: {obstacle_distance:.2f}m")
                self._last_dwa_log_time = self.getTime()

            self.setStateValue("state", "SEND")

        elif state == "SEND":
            # 항상 PLAN으로 돌아간다. 회피는 플래그로만 관리한다
            self.setStateValue("state", "PLAN")

        elif state == "REPLAN":
            # 재계획을 요청하고 기다린다
            self.reset_replan_check()
            self.setStateValue("state", "WAIT")

        elif state == "DOCKING":
            # 장비는 도킹 후 jobExchange를 기다린다
            # 대기 구역은 여기로 오지 않고 곧바로 UNDOCKING으로 간다
            self.setStateValue("state", "WAIT")

        elif state == "UNDOCKING":
            # 대기 구역과 장비를 구분한다
            is_waiting_area = (self.currentGoalNodeID and
                               self.currentGoalNodeID.startswith('WAITING_AREA'))

            if is_waiting_area:
                # 대기 구역에서는 previousEquipmentID를 두지 않는다
                self.previousEquipmentID = None
                # 대기 구역에서는 멈추기만 하면 되므로 yaw를 고정하지 않는다
                self.needs_yaw_force = False
                self.yaw_forced = False  # 플래그 초기화

                # 대기 구역 언도킹 시 상태를 출력한다
                amrID = self.ID.split('_')[0]
                self.globalVar.printTerminal(
                    f"\n{'='*80}\n"
                    f"[{self.getTime()}][LocalPlanner({self.ID})] 🅿️ WaitingArea UNDOCKING - Vehicle 상태 전체 출력\n"
                    f"{'='*80}"
                )
                print(f"📋 Vehicle ID: {amrID}")
                print(f"   LocalPlanner ID: {self.ID}")
                print(f"   Current State: {self.getStateValue('state')}")
                print(f"   Transport Phase: {self.transportPhase}")
                print(f"   Current Goal NodeID: {self.currentGoalNodeID}")
                print(f"   Previous Equipment ID: {self.previousEquipmentID}")
                print(f"   Needs Yaw Force: {self.needs_yaw_force}")
                print(f"   Yaw Forced: {self.yaw_forced}")
                print(
                    f"   Current Pose: x={self.curPose[0]:.2f}, y={self.curPose[1]:.2f}, yaw={self.curPose[2]:.4f} ({math.degrees(self.curPose[2]):.2f}°), lin_vel={self.curPose[3]:.2f}, ang_vel={self.curPose[4]:.4f}")
                print(f"   Current Goal: {self.goal}")
                print(f"   Path Length: {len(self.path) if self.path else 0}")
                print(f"   Is In Avoidance: {self.is_in_avoidance}")
                print(f"   Obstacles Count: {len(self.obstacles)}")
                print(f"   Other Agents Count: {len(self.other_agents)}")
                if self.path:
                    print(
                        f"   Path Preview: First={self.path[0] if len(self.path) > 0 else 'N/A'}, Last={self.path[-1] if len(self.path) > 0 else 'N/A'}")
                if self.goal:
                    distance = np.linalg.norm(
                        np.array(self.goal) - np.array(self.curPose[:2]))
                    print(f"   Distance to Goal: {distance:.2f}m")
                print(f"{'='*80}\n")
            else:
                # Equipment undocking
                equipmentID = self.currentGoalNodeID.split(
                    '_')[0] if self.currentGoalNodeID else None

                # 직전 장비 ID. DWA에서 장애물 제외에 쓴다
                self.previousEquipmentID = equipmentID
                # yaw 고정 플래그 초기화
                self.needs_yaw_force = True
                self.yaw_forced = False

            # 정적 장애물 캐시 무효화
            self.cached_static_polygons = None
            self.cached_exclude_nodes = None

            self.setStateValue("state", "Undocking_to_fleetmanagement")

        elif state == "Undocking_to_fleetmanagement":
            self.setStateValue("state", "WAIT")

    def funcOutput(self):
        state = self.getStateValue("state")

        if state == "SEND":
            # Maneuver로 목표를 보낸다
            objRequestMessage = MsgManeuverState(
                self.ID,
                self.target_x,
                self.target_y
            )
            self.addOutputEvent("RequestManeuver_O", objRequestMessage)
            self.globalVar.printTerminal(
                f"[{self.getTime()}][LocalPlanner({self.ID})] Sending target: ({self.target_x:.2f}, {self.target_y:.2f})"
            )

        elif state == "REPLAN":
            # Global_Planner로 재계획을 요청한다
            done_message = MsgManeuverState(
                self.ID,
                self.curPose[0],
                self.curPose[1]
            )
            self.addOutputEvent("Replan", done_message)
            self.globalVar.printTerminal(
                f"[{self.getTime()}][LocalPlanner({self.ID})] Requesting replan from Global Planner"
            )

        elif state == "DOCKING":
            # Maneuver로 도킹 명령을 보낸다
            docking_x = self.goal[0] if self.goal else self.curPose[0]
            docking_y = self.goal[1] if self.goal else self.curPose[1]
            docking_message = MsgManeuverState(
                self.ID,
                docking_x,
                docking_y
            )
            self.addOutputEvent("Docking_O", docking_message)

            # Equipment로 도킹을 알린다. 단계 정보를 함께 담는다
            # [amrID, equipmentID, phase]
            vehicle_id = self.ID.split('_', 1)[0]
            # currentGoalNodeID(예: "A-1_IN")에서 equipmentID를 뽑는다
            equipment_id = self.currentGoalNodeID.split(
                '_')[0] if self.currentGoalNodeID and '_' in self.currentGoalNodeID else None
            if equipment_id and self.transportPhase:
                docking_notify = [vehicle_id,
                                  equipment_id, self.transportPhase]
                self.addOutputEvent("EquipmentDocking", docking_notify)
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][LocalPlanner({self.ID})] 🚏 Docking at {equipment_id} (phase: {self.transportPhase})"
                )
            else:
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][LocalPlanner({self.ID})] 🚏 Docking command at ({docking_x:.2f}, {docking_y:.2f})"
                )
            return True
        elif state == "UNDOCKING":
            amrID = self.ID.split('_')[0]

            # 대기 구역과 장비를 구분한다
            # 대기 구역 ID는 "WAITING_AREA_xxx" 형태로 밑줄이 둘 이상이다
            # 장비 ID는 "A-1_IN"이나 "A-1_OUT" 형태로 밑줄이 하나다
            is_waiting_area = (self.currentGoalNodeID and
                               self.currentGoalNodeID.startswith('WAITING_AREA'))

            # 디버그 로그
            print(f"🔍 [UNDOCKING_DEBUG] {amrID}:")
            print(f"   currentGoalNodeID: {self.currentGoalNodeID}")
            print(f"   transportPhase: {self.transportPhase}")
            print(f"   is_waiting_area: {is_waiting_area}")
            print(
                f"   Has _IN: {'_IN' in self.currentGoalNodeID if self.currentGoalNodeID else 'N/A'}")
            print(
                f"   Has _OUT: {'_OUT' in self.currentGoalNodeID if self.currentGoalNodeID else 'N/A'}")

            if not is_waiting_area:
                # 장비 언도킹. Maneuver로 명령을 보낸다
                equipmentID = self.currentGoalNodeID.split(
                    '_')[0] if self.currentGoalNodeID else None

                # 장비의 출력 포트 좌표
                Equipment = self.globalVar.getEquipmentInfoByID(equipmentID)
                Out_pos = Equipment.outputPort.get('position')

                print(
                    f"🗺️ [LOCAL_UNDOCKING_OUTPUT] {amrID}: Sending Undocking message")
                print(f"   My ID: {self.ID}")
                print(f"   AMR ID: {amrID}")
                print(f"   Equipment: {equipmentID}")
                print(f"   Target outputPort: {Out_pos}")

                # 좌표를 함께 보내면 Maneuver가 그 자리로 옮긴다
                undocking_message = MsgManeuverState(
                    self.ID.replace('_LPP', '_maneuver'),
                    Out_pos['x'],
                    Out_pos['y']
                )
                self.addOutputEvent("Undocking_O", undocking_message)
            else:
                # 대기 구역에서는 현재 위치에 그대로 선다
                vehicleInfo = self.globalVar.getVehicleInfoByID(amrID)
                if vehicleInfo:
                    current_coords = vehicleInfo.getCoordinates()
                    current_x = current_coords[0]
                    current_y = current_coords[1]
                else:
                    # 차량 정보가 없으면 대기 구역 좌표를 쓴다
                    waitingArea = self.globalVar.getWaitingAreaInfoByID(
                        self.currentGoalNodeID)
                    if waitingArea:
                        current_x = waitingArea.position['x']
                        current_y = waitingArea.position['y']
                    else:
                        current_x = self.goal[0] if self.goal else 0
                        current_y = self.goal[1] if self.goal else 0

                print(
                    f"🅿️ [LOCAL_WAITING_STOP] {amrID}: Sending stop at WaitingArea")
                print(f"   WaitingArea: {self.currentGoalNodeID}")
                print(f"   Stop position: ({current_x}, {current_y})")

                # Maneuver에 현재 위치를 유지하라고 보낸다
                stop_message = MsgManeuverState(
                    self.ID.replace('_LPP', '_maneuver'),
                    current_x,
                    current_y
                )
                self.addOutputEvent("Undocking_O", stop_message)

        elif state == "Undocking_to_fleetmanagement":
            amrID = self.ID.split('_')[0]

            # 대기 구역과 장비를 구분한다
            is_waiting_area = (self.currentGoalNodeID and
                               self.currentGoalNodeID.startswith('WAITING_AREA'))

            if is_waiting_area:
                # 대기 구역이면 areaID를 보낸다
                locationID = self.currentGoalNodeID
            else:
                # 장비면 equipmentID를 보낸다
                locationID = self.currentGoalNodeID.split(
                    '_')[0] if self.currentGoalNodeID else None

            jobID = self.globalVar.getVehicleInfoByID(
                amrID).intJobID if self.globalVar.getVehicleInfoByID(amrID) else None

            self.addOutputEvent("UndockingComplete_O", [
                                amrID, locationID, jobID, self.transportPhase])
            return True
        else:
            return False

    def funcTimeAdvance(self):
        state = self.getStateValue("state")

        if state == "WAIT":
            return float('inf')
        elif state in ["SEND", "REPLAN"]:
            return 0  # 즉시 출력
        elif state == "PLAN":
            return 0.1  # 100 ms마다 계획한다
        elif state == "UNDOCKING":
            return 1.0
        elif state == "Undocking_to_fleetmanagement":
            return 0
        else:
            return 1.0

    def calc_dwa(self):
        """DWA로 지역 경로를 계획한다."""
        if not self.goal:
            return None

        # 동적 장애물
        obstacle_positions = list(self.obstacles.values())

        # 정적 장애물. 현재 목표 노드는 뺀다
        static_polygons = self.get_static_obstacles_for_dwa()

        # 목표까지의 거리
        current_pos = np.array(self.curPose[:2])
        goal_pos = np.array(self.goal)
        distance_to_goal = np.linalg.norm(goal_pos - current_pos)

        # DWA 파라미터
        dwa_params = {
            'max_speed': 1.5,
            'max_yawrate': np.pi / 3,
            'safety_margin': 2.0
        }

        # 목표에 가까우면 감속한다
        if distance_to_goal < 5.0:
            dwa_params['max_speed'] *= 0.7

        # 회피 모드에서는 파라미터를 보수적으로 바꾼다
        if self.is_in_avoidance:
            dwa_params['max_speed'] *= 0.8      # 속도를 낮춘다
            dwa_params['max_yawrate'] *= 1.5    # 회전을 더 허용한다
            dwa_params['safety_margin'] *= 1.2  # 안전 마진을 넓힌다

        # 정적 장애물까지 넣어 DWA를 실행한다
        target_state = self.dwa.calc_dwa(
            self.curPose,
            self.goal,
            np.array(obstacle_positions),  # 동적 장애물
            static_polygons,  # 정적 장애물, 현재 목표 제외
            **dwa_params
        )

        if target_state is not None:
            return [target_state[0], target_state[1], target_state[2]]
        else:
            self.globalVar.printTerminal(
                f"[{self.getTime()}][LocalPlanner({self.ID})] DWA calculation failed"
            )
            return None

    def get_static_obstacles_for_dwa(self):
        """DWA에 넘길 정적 장애물을 모은다.

        현재 목표 노드와 직전 장비는 제외한다. 결과는 제외 노드가 같은
        동안 캐시하여 재사용한다.
        """

        # 제외할 노드 목록
        exclude_nodes = []
        if self.currentGoalNodeID:
            exclude_nodes.append(self.currentGoalNodeID)
        if self.previousEquipmentID:
            start_nodeID = self.previousEquipmentID + '_OUT'
            exclude_nodes.append(start_nodeID)

        # 제외 노드가 같으면 캐시를 그대로 쓴다
        if (self.cached_static_polygons is not None and
                self.cached_exclude_nodes == exclude_nodes):
            # 캐시 재사용
            return self.cached_static_polygons

        # 캐시가 없거나 무효하면 새로 만든다
        if self.previousEquipmentID and self.cached_exclude_nodes != exclude_nodes:
            self.globalVar.printTerminal(
                f"[{self.getTime()}][LocalPlanner] Rebuilding static obstacles cache (exclude: {exclude_nodes})"
            )

        obstacles = self.globalVar.getObstacleInfo()
        static_polygons = []

        # 현재 위치와 목표 위치를 미리 구해 둔다
        current_point = Point(self.curPose[0], self.curPose[1])
        goal_point = Point(self.goal[0], self.goal[1]) if self.goal else None

        for obs in obstacles:
            # 현재 목표 노드는 장애물에서 뺀다
            if 'nodeID' in obs and obs['nodeID'] in exclude_nodes:
                continue

            pos = obs['position']
            bbox = obs['boundingBox']

            x_min = pos['x'] - bbox['width'] / 2
            x_max = pos['x'] + bbox['width'] / 2
            y_min = pos['y'] - bbox['height'] / 2
            y_max = pos['y'] + bbox['height'] / 2

            # Polygon 좌표
            polygon = Polygon([
                (x_min, y_min),
                (x_max, y_min),
                (x_max, y_max),
                (x_min, y_max)
            ])

            # 현재 위치를 품은 폴리곤은 빼야 로봇이 빠져나올 수 있다
            if polygon.contains(current_point):
                continue

            # 목표를 품은 폴리곤도 빼야 안으로 들어갈 수 있다
            if goal_point and polygon.contains(goal_point):
                continue

            static_polygons.append(polygon)

        # 캐시에 넣는다
        self.cached_static_polygons = static_polygons
        self.cached_exclude_nodes = exclude_nodes.copy()

        return static_polygons

    def check_distance(self):
        """현재 위치에서 목표까지의 거리."""
        if not self.curPose or not self.goal:
            return float('inf')

        current_position = np.array(self.curPose[:2])
        goal_position = np.array(self.goal)
        distance = np.linalg.norm(goal_position - current_position)

        return distance

    def check_obstacle_distance(self):
        """현재 위치에서 가장 가까운 장애물까지의 거리."""
        if not self.curPose or not self.obstacles:
            return float('inf')

        current_position = np.array(self.curPose[:2])
        min_distance = float('inf')

        for obstacle_pos in self.obstacles.values():
            obstacle_position = np.array(obstacle_pos)
            distance = np.linalg.norm(obstacle_position - current_position)
            if distance < min_distance:
                min_distance = distance

        return min_distance

    def check_agent_distance(self, agent_id):
        """다른 에이전트까지의 거리."""
        if not self.curPose or agent_id not in self.other_agents:
            return float('inf')

        current_position = np.array(self.curPose[:2])
        agent_data = self.other_agents[agent_id]
        agent_position = np.array(agent_data['pos'])
        return np.linalg.norm(agent_position - current_position)

    def predict_collision_risk(self, agent_id):
        """상대 속도를 이용해 다른 AMR과의 충돌 위험을 예측한다."""
        if agent_id not in self.other_agents:
            return False

        agent_data = self.other_agents[agent_id]
        my_pos = np.array(self.curPose[:2])
        my_vel = np.array([self.curPose[3] * np.cos(self.curPose[2]),
                          self.curPose[3] * np.sin(self.curPose[2])])

        other_pos = np.array(agent_data['pos'])
        other_vel = np.array(agent_data['vel'][:2]) if len(
            agent_data['vel']) >= 2 else np.array([0.0, 0.0])

        # 상대 위치와 상대 속도
        rel_pos = other_pos - my_pos
        rel_vel = other_vel - my_vel

        # 거리
        distance = np.linalg.norm(rel_pos)

        # 아주 가까우면 무조건 위험으로 본다
        if distance < self.collision_radius:
            return True

        # 상대 속도가 거의 없으면 위험하지 않다
        rel_speed = np.linalg.norm(rel_vel)
        if rel_speed < 0.1:
            return False

        # 예측 구간 3초
        prediction_time = 3.0

        # 미래 위치를 외삽한다
        my_future_pos = my_pos + my_vel * prediction_time
        other_future_pos = other_pos + other_vel * prediction_time

        # 미래 거리
        future_distance = np.linalg.norm(other_future_pos - my_future_pos)

        # 미래에 더 가까워지면 위험으로 본다
        if future_distance < self.collision_radius:
            return True

        # 내적으로 접근 여부를 확인한다
        # 상대 위치와 상대 속도의 내적이 음수면 접근 중이다
        if distance > 0:
            approaching = np.dot(rel_pos, rel_vel) < 0

            # 접근 중이고 가까우면 위험이다
            if approaching and distance < self.safety_tolerance * 0.7:
                return True

        return False

    def get_next_waypoint(self):
        """경로에서 다음 waypoint를 가져온다."""
        if not self.path or len(self.path) <= 1:
            return None

        # 현재 목표와 가장 가까운 waypoint의 인덱스
        current_idx = self.find_current_waypoint_index()

        if current_idx is not None and current_idx < len(self.path) - 1:
            # 그 다음 waypoint
            next_wp = self.path[current_idx + 1]
            return (next_wp[0], next_wp[1])

        return None

    def find_current_waypoint_index(self):
        """현재 목표가 경로의 몇 번째 waypoint인지 찾는다."""
        if not self.path or not self.goal:
            return None

        goal_array = np.array(self.goal)

        for i, waypoint in enumerate(self.path):
            wp_array = np.array([waypoint[0], waypoint[1]])
            distance = np.linalg.norm(goal_array - wp_array)

            # 거리가 1.0 미만이면 같은 지점으로 본다
            if distance < 1.0:
                return i

        # 찾지 못하면 첫 waypoint로 본다
        return 0

    def replan_check(self):
        """목표까지의 거리가 줄고 있는지 보고 재계획이 필요한지 판단한다.

        일정 시간 동안 거리도 위치도 거의 변하지 않으면 교착이나 진동으로
        보고 True를 반환한다.
        """
        current_time = self.getTime()
        current_position = np.array(self.curPose[:2])
        current_distance = self.check_distance()

        # 추적 시작
        if self.replan_check_start_time is None:
            self.replan_check_start_time = current_time
            self.replan_check_initial_distance = current_distance
            self.replan_check_initial_position = current_position.copy()
            return False

        # 경과 시간
        elapsed_time = current_time - self.replan_check_start_time

        # 임계 시간이 지났는지 확인한다
        if elapsed_time >= self.replan_check_threshold_time:
            # 거리 개선량
            distance_improved = self.replan_check_initial_distance - current_distance

            # 이동량
            position_movement = np.linalg.norm(
                current_position - self.replan_check_initial_position
            )

            # 진전 판단
            needs_replan = False
            reason = ""

            # 조건 1: 거리가 충분히 줄지 않았다
            if distance_improved < self.replan_check_distance_improvement:
                needs_replan = True
                reason = f"거리 개선 부족 (개선: {distance_improved:.2f}m < 필요: {self.replan_check_distance_improvement}m)"

            # 조건 2: 위치가 거의 그대로다. 제자리에서 맴돈다
            if position_movement < self.replan_check_position_movement:
                needs_replan = True
                reason += f" / 위치 이동 부족 (이동: {position_movement:.2f}m < 필요: {self.replan_check_position_movement}m)"

            if needs_replan:
                self.globalVar.printTerminal(
                    f"[{current_time}][LocalPlanner({self.ID})] ⚠️ 재계획 필요: {reason}"
                )
                self.globalVar.printTerminal(
                    f"   경과시간: {elapsed_time:.1f}초, 초기거리: {self.replan_check_initial_distance:.2f}m → 현재거리: {current_distance:.2f}m"
                )
                # 상태 초기화
                self.reset_replan_check()
                return True
            else:
                # 충분히 진전했으면 기준점을 다시 잡는다
                self.globalVar.printTerminal(
                    f"[{current_time}][LocalPlanner({self.ID})] ✅ 진전 있음: 거리 {distance_improved:.2f}m 개선, {position_movement:.2f}m 이동"
                )
                self.reset_replan_check()
                return False

        return False

    def reset_replan_check(self):
        """진전도 추적 상태를 초기화한다."""
        self.replan_check_start_time = None
        self.replan_check_initial_distance = None
        self.replan_check_initial_position = None
