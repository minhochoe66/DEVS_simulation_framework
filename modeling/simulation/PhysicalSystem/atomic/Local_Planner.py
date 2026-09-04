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
        self.safety_tolerance = 8.0  # peer detection range, 8 m
        self.collision_radius = 3.0  # collision-avoidance radius, 3 m

        # State variables
        self.goal = None
        self.curPose = [0, 0, 0, 0, 0]  # x, y, yaw, lin_vel, ang_vel
        self.obstacles = {}
        # {agent_id: {'pos': (x,y), 'vel': (vx,vy), 'time': t}}
        self.other_agents = {}

        # Transport phase
        self.transportPhase = None  # "TO_FROM" or "TO_DESTINATION"
        self.path = []  # the whole path
        self.currentGoalNodeID = None  # current goal node ID, used to exclude it from the obstacles
        self.previousEquipmentID = None  # previous machine ID, excluded from the obstacles after undocking
        self.yaw_forced = False  # whether the yaw has already been pinned
        self.needs_yaw_force = False  # whether the yaw needs pinning, straight after undocking

        # Cached static obstacles
        self.cached_static_polygons = None  # the cached polygons
        self.cached_exclude_nodes = None  # the nodes excluded when the cache was built

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
        self.replan_check_threshold_time = 5.0  # replan after 5 s without progress
        self.replan_check_distance_improvement = 2.0  # progress means closing at least 2 m
        self.replan_check_position_movement = 1.0  # and moving at least 1 m

        # Logging variables
        self._last_pose_log_time = 0
        self._last_dwa_log_time = 0

        # Avoidance state
        self.is_in_avoidance = False  # whether avoidance is active
        self.avoidance_clearance_threshold = self.safety_tolerance + \
            2.0  # release threshold, giving the mode hysteresis

        # DEVS state
        self.addStateVariable("state", "WAIT")

        # Input Ports
        self.addInputPort("GlobalWaypoint_I")  # waypoints from Global_Planner
        self.addInputPort("agent_pose_I")      # poses of peers and obstacles
        self.addInputPort("ManeuverState_I")   # this robot's pose
        self.addInputPort("OtherManeuverState_I")  # peer robot poses

        # Output Ports
        self.addOutputPort("RequestManeuver_O")  # goal sent to Maneuver
        self.addOutputPort("Replan")             # replan request to Global_Planner
        self.addOutputPort("DeliveryComplete")   # arrived at the destination machine, unloaded
        self.addOutputPort("Docking")            # docking command to Maneuver
        self.addOutputPort("EquipmentDocking")   # docking notice to Equipment
        self.addOutputPort("UndockingComplete")  # undocking complete, to FleetManagement
        self.addOutputPort("Undocking_O")        # undocking command to Maneuver

    def funcExternalTransition(self, strPort, objEvent):
        state = self.getStateValue("state")

        if strPort == "GlobalWaypoint_I":
            # waypoints from Global_Planner, as a MsgGoal
            if hasattr(objEvent, 'strID') and objEvent.strID.split('_', 1)[0] == self.ID.split('_', 1)[0]:
                # head for the IN port of the pickup or destination machine, per the transport command
                self.goal = (objEvent.dblPositionX, objEvent.dblPositionY)

                # take the transport phase from Global_Planner
                self.transportPhase = objEvent.transportPhase if objEvent.transportPhase else None

                # store the path
                self.path = objEvent.path if objEvent.path else []

                # current goal node ID, used to exclude it from the obstacles
                self.currentGoalNodeID = objEvent.goalNodeID if objEvent.goalNodeID else None

                self.globalVar.printTerminal(
                    f"[{self.getTime()}][LocalPlanner({self.ID})] Received waypoint: {self.goal}, Phase: {self.transportPhase}, NodeID: {self.currentGoalNodeID}"
                )

                # outside TO_DESTINATION, clear previousEquipmentID
                if self.transportPhase == "TO_FROM" or self.transportPhase == "WAITING":
                    self.previousEquipmentID = None
                    # the TO_FROM and WAITING phases need no yaw pinning
                    self.needs_yaw_force = False
                    self.yaw_forced = False

                # a new goal resets the progress tracking
                self.reset_replan_check()

                # a new global path clears avoidance mode
                if self.is_in_avoidance:
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][LocalPlanner({self.ID})] 새로운 경로 수신! AVOIDANCE 모드 종료"
                    )
                    self.is_in_avoidance = False

                # the goal moved, so the static obstacle cache is stale
                self.cached_static_polygons = None
                self.cached_exclude_nodes = None

                self.setStateValue("state", "PLAN")

        elif strPort == "ManeuverState_I":
            # update this robot's pose
            event_id = objEvent.strID.split('_', 1)[0]
            my_id = self.ID.split('_', 1)[0]

            # log at most once per second
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

                # pin the yaw to 90 degrees once, on the first PLAN after undocking
                # covers undocking from a machine and from a waiting area
                if state == "PLAN" and (self.needs_yaw_force or (self.previousEquipmentID and not self.yaw_forced)):
                    self.curPose[2] = np.pi / 2  # pin to 90 degrees
                    self.yaw_forced = True  # mark it done, so it runs only once
                    self.needs_yaw_force = False  # request consumed
                    source = f"Equipment {self.previousEquipmentID}" if self.previousEquipmentID else "WaitingArea"
                    print(
                        f"   🔄 [ONCE] Forced yaw to 90° after UNDOCKING from {source}")

                self.continueTimeAdvance()
            else:
                if should_log:
                    print(f"   ❌ IGNORED: Not my message")
                self.continueTimeAdvance()

        elif strPort == "OtherManeuverState_I":
            # peer poses, excluding this robot
            other_agent_id = objEvent.strID.split(
                '_')[0] if '_' in objEvent.strID else objEvent.strID
            my_id = self.ID.split('_', 1)[0]

            if other_agent_id != my_id:
                # MsgManeuverState carries dblPositionX and dblPositionY
                current_time = self.getTime()
                other_pos = (objEvent.dblPositionX, objEvent.dblPositionY)

                # with a previous pose, derive the speed from the displacement
                other_vel = (0.0, 0.0)  # default
                if other_agent_id in self.other_agents:
                    prev_data = self.other_agents[other_agent_id]
                    prev_pos = prev_data['pos']
                    prev_time = prev_data['time']
                    dt = current_time - prev_time

                    if dt > 0:
                        # actual speed, from the displacement
                        vx = (other_pos[0] - prev_pos[0]) / dt
                        vy = (other_pos[1] - prev_pos[1]) / dt
                        other_vel = (vx, vy)

                # update the peer record
                self.other_agents[other_agent_id] = {
                    'pos': other_pos,
                    'vel': other_vel,
                    'time': current_time
                }

                # collision risk is judged from distance, speed and field of view
                distance = self.check_agent_distance(other_agent_id)

                # only peers within about 120 degrees ahead count as obstacles
                fov_rad = 2.0 * np.pi / 3.0  # 120 degrees
                rel_vec = np.array(other_pos) - np.array(self.curPose[:2])
                angle_to_other = math.atan2(rel_vec[1], rel_vec[0])
                rel_angle = (
                    angle_to_other - self.curPose[2] + math.pi) % (2 * math.pi) - math.pi
                in_front = abs(rel_angle) <= (fov_rad / 2.0)

                if distance < self.safety_tolerance and in_front:
                    # register it as an obstacle when close
                    collision_risk = self.predict_collision_risk(
                        other_agent_id)

                    if collision_risk or distance < self.collision_radius:
                        # register it when a collision is predicted, or it is very close
                        self.obstacles[other_agent_id] = other_pos

                        if distance < self.collision_radius:
                            self.globalVar.printTerminal(
                                f"[{current_time}][LocalPlanner({self.ID})] ⚠️ 근접 경고: {other_agent_id} at {distance:.2f}m"
                            )

                        # set the avoidance flag without changing state
                        if not self.is_in_avoidance:
                            self.globalVar.printTerminal(
                                f"[{current_time}][LocalPlanner({self.ID})] 🚨 장애물 감지! 회피 모드 ON (거리: {distance:.2f}m)"
                            )
                            self.is_in_avoidance = True
                    else:
                        # no risk: drop it from the obstacles
                        if other_agent_id in self.obstacles:
                            del self.obstacles[other_agent_id]
                else:
                    # behind, or beyond the safe distance: drop it from the obstacles
                    if other_agent_id in self.obstacles:
                        del self.obstacles[other_agent_id]

            self.continueTimeAdvance()

        elif strPort == "UndockingComplete_I":
            # objEvent: [amrID, equipmentID, jobID, phase]
            if objEvent[0] == self.ID.split('_')[0]:
                # already in the waiting area: ignore the repeated signal
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

            # check for progress, and replan if there is none
            if self.replan_check():
                self.setStateValue("state", "REPLAN")
                return

            distance_to_goal = self.check_distance()
            obstacle_distance = self.check_obstacle_distance()

            # release avoidance mode once the obstacles are far enough away
            if self.is_in_avoidance and obstacle_distance > self.avoidance_clearance_threshold:
                self.is_in_avoidance = False
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][LocalPlanner({self.ID})] ✅ 회피 모드 OFF! 장애물 거리: {obstacle_distance:.2f}m"
                )

            # have we reached the goal?
            target_tolerance = self.objConfiguration.getConfiguration(
                'target_tolerance') or 5.0
            if distance_to_goal < target_tolerance:
                phase_str = self.transportPhase if self.transportPhase else 'Unknown'
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][LocalPlanner({self.ID})] Arrived at waypoint (Phase: {phase_str})"
                )

                # on arrival, reset the progress tracking
                self.reset_replan_check()

                # final destination, handling waiting areas and machines alike
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
                        # fallback: the current goal, or the current pose
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

                # convert to numpy arrays for the distance computation
                final_destination = np.array([dest_x, dest_y])
                current_position = np.array(self.curPose[:2])
                distance_to_final = np.linalg.norm(
                    final_destination - current_position)

                if distance_to_final < 5.0:
                    # reaching the final destination leads to DOCKING
                    if self.transportPhase == "TO_FROM":
                        # at the pickup machine: order docking
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][LocalPlanner({self.ID})] 🎯 Arrived at FINAL destination (FROM Equipment)"
                        )
                        self.setStateValue("state", "DOCKING")

                    elif self.transportPhase == "TO_DESTINATION":
                        # at the destination machine: order docking
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
                        # an absent or unexpected transportPhase triggers a replan
                        self.setStateValue("state", "REPLAN")
                    return
                else:
                    # an intermediate waypoint is reached: move to the next
                    next_waypoint = self.get_next_waypoint()
                    if next_waypoint:
                        self.goal = next_waypoint
                        # the goal moved, so reset the progress tracking
                        self.reset_replan_check()
                        # and invalidate the static obstacle cache
                        self.cached_static_polygons = None
                        self.cached_exclude_nodes = None
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][LocalPlanner({self.ID})] ✓ Passed intermediate waypoint, moving to next: {self.goal}"
                        )
                    else:
                        # no next waypoint: replan
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][LocalPlanner({self.ID})] ⚠️ No next waypoint available, requesting replan"
                        )
                        self.setStateValue("state", "REPLAN")
                    return

            # run DWA; in avoidance mode it runs with adjusted parameters
            start_time = time.time()
            target_state = self.calc_dwa()
            end_time = time.time()
            elapsed = end_time - start_time
            self.globalVar.LocalPlanner_algorithm_time += elapsed
            self.globalVar.LocalPlanner_call_count += 1

            if target_state is None:
                # no feasible trajectory: ask for a replan
                mode_str = "AVOIDANCE" if self.is_in_avoidance else "PLAN"
                print(f"   ❌ DWA FAILED ({mode_str})")
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][LocalPlanner({self.ID})] DWA failed ({mode_str}), requesting replan"
                )
                self.setStateValue("state", "REPLAN")
                return

            # update the target
            self.target_x = target_state[0]
            self.target_y = target_state[1]
            self.target_yaw = target_state[2]

            # log the DWA result once per second, noting whether avoidance is on
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
            # always return to PLAN; avoidance is carried by the flag alone
            self.setStateValue("state", "PLAN")

        elif state == "REPLAN":
            # request a replan and wait
            self.reset_replan_check()
            self.setStateValue("state", "WAIT")

        elif state == "DOCKING":
            # at a machine, wait for jobExchange after docking
            # a waiting area never reaches here; it goes straight to UNDOCKING
            self.setStateValue("state", "WAIT")

        elif state == "UNDOCKING":
            # distinguish a waiting area from a machine
            is_waiting_area = (self.currentGoalNodeID and
                               self.currentGoalNodeID.startswith('WAITING_AREA'))

            if is_waiting_area:
                # a waiting area sets no previousEquipmentID
                self.previousEquipmentID = None
                # in a waiting area the robot only has to stop, so the yaw is left alone
                self.needs_yaw_force = False
                self.yaw_forced = False  # clear the flag

                # trace the state when undocking from a waiting area
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

                # previous machine ID, excluded from the DWA obstacles
                self.previousEquipmentID = equipmentID
                # arm the yaw pinning
                self.needs_yaw_force = True
                self.yaw_forced = False

            # invalidate the static obstacle cache
            self.cached_static_polygons = None
            self.cached_exclude_nodes = None

            self.setStateValue("state", "Undocking_to_fleetmanagement")

        elif state == "Undocking_to_fleetmanagement":
            self.setStateValue("state", "WAIT")

    def funcOutput(self):
        state = self.getStateValue("state")

        if state == "SEND":
            # send the target to Maneuver
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
            # ask Global_Planner to replan
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
            # send the docking command to Maneuver
            docking_x = self.goal[0] if self.goal else self.curPose[0]
            docking_y = self.goal[1] if self.goal else self.curPose[1]
            docking_message = MsgManeuverState(
                self.ID,
                docking_x,
                docking_y
            )
            self.addOutputEvent("Docking_O", docking_message)

            # tell Equipment about the docking, including the phase
            # [amrID, equipmentID, phase]
            vehicle_id = self.ID.split('_', 1)[0]
            # derive the equipment ID from currentGoalNodeID, e.g. "A-1_IN"
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

            # distinguish a waiting area from a machine
            # a waiting-area ID looks like "WAITING_AREA_xxx", with two or more underscores
            # a machine ID looks like "A-1_IN" or "A-1_OUT", with one underscore
            is_waiting_area = (self.currentGoalNodeID and
                               self.currentGoalNodeID.startswith('WAITING_AREA'))

            # debug trace
            print(f"🔍 [UNDOCKING_DEBUG] {amrID}:")
            print(f"   currentGoalNodeID: {self.currentGoalNodeID}")
            print(f"   transportPhase: {self.transportPhase}")
            print(f"   is_waiting_area: {is_waiting_area}")
            print(
                f"   Has _IN: {'_IN' in self.currentGoalNodeID if self.currentGoalNodeID else 'N/A'}")
            print(
                f"   Has _OUT: {'_OUT' in self.currentGoalNodeID if self.currentGoalNodeID else 'N/A'}")

            if not is_waiting_area:
                # undocking from a machine: send the command to Maneuver
                equipmentID = self.currentGoalNodeID.split(
                    '_')[0] if self.currentGoalNodeID else None

                # the machine's output-port coordinates
                Equipment = self.globalVar.getEquipmentInfoByID(equipmentID)
                Out_pos = Equipment.outputPort.get('position')

                print(
                    f"🗺️ [LOCAL_UNDOCKING_OUTPUT] {amrID}: Sending Undocking message")
                print(f"   My ID: {self.ID}")
                print(f"   AMR ID: {amrID}")
                print(f"   Equipment: {equipmentID}")
                print(f"   Target outputPort: {Out_pos}")

                # Maneuver moves the robot to the coordinates carried in the command
                undocking_message = MsgManeuverState(
                    self.ID.replace('_LPP', '_maneuver'),
                    Out_pos['x'],
                    Out_pos['y']
                )
                self.addOutputEvent("Undocking_O", undocking_message)
            else:
                # in a waiting area the robot simply holds its position
                vehicleInfo = self.globalVar.getVehicleInfoByID(amrID)
                if vehicleInfo:
                    current_coords = vehicleInfo.getCoordinates()
                    current_x = current_coords[0]
                    current_y = current_coords[1]
                else:
                    # with no vehicle record, fall back to the waiting-area coordinates
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

                # tell Maneuver to hold the current position
                stop_message = MsgManeuverState(
                    self.ID.replace('_LPP', '_maneuver'),
                    current_x,
                    current_y
                )
                self.addOutputEvent("Undocking_O", stop_message)

        elif state == "Undocking_to_fleetmanagement":
            amrID = self.ID.split('_')[0]

            # distinguish a waiting area from a machine
            is_waiting_area = (self.currentGoalNodeID and
                               self.currentGoalNodeID.startswith('WAITING_AREA'))

            if is_waiting_area:
                # a waiting area reports its areaID
                locationID = self.currentGoalNodeID
            else:
                # a machine reports its equipmentID
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
            return 0  # emit at once
        elif state == "PLAN":
            return 0.1  # replan every 100 ms
        elif state == "UNDOCKING":
            return 1.0
        elif state == "Undocking_to_fleetmanagement":
            return 0
        else:
            return 1.0

    def calc_dwa(self):
        """Plan the local motion with DWA."""
        if not self.goal:
            return None

        # dynamic obstacles
        obstacle_positions = list(self.obstacles.values())

        # static obstacles, minus the current goal node
        static_polygons = self.get_static_obstacles_for_dwa()

        # distance to the goal
        current_pos = np.array(self.curPose[:2])
        goal_pos = np.array(self.goal)
        distance_to_goal = np.linalg.norm(goal_pos - current_pos)

        # DWA parameters
        dwa_params = {
            'max_speed': 1.5,
            'max_yawrate': np.pi / 3,
            'safety_margin': 2.0
        }

        # slow down near the goal
        if distance_to_goal < 5.0:
            dwa_params['max_speed'] *= 0.7

        # in avoidance mode the parameters are made more conservative
        if self.is_in_avoidance:
            dwa_params['max_speed'] *= 0.8      # lower the speed
            dwa_params['max_yawrate'] *= 1.5    # allow more turning
            dwa_params['safety_margin'] *= 1.2  # widen the safety margin

        # run DWA, static obstacles included
        target_state = self.dwa.calc_dwa(
            self.curPose,
            self.goal,
            np.array(obstacle_positions),  # dynamic obstacles
            static_polygons,  # static obstacles, current goal excluded
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
        """Collect the static obstacles for DWA.

        The current goal and the previous machine are excluded. The result is
        cached and reused for as long as the exclusion set is unchanged.
        """

        # build the exclusion list
        exclude_nodes = []
        if self.currentGoalNodeID:
            exclude_nodes.append(self.currentGoalNodeID)
        if self.previousEquipmentID:
            start_nodeID = self.previousEquipmentID + '_OUT'
            exclude_nodes.append(start_nodeID)

        # the cache is valid while the exclusion set is unchanged
        if (self.cached_static_polygons is not None and
                self.cached_exclude_nodes == exclude_nodes):
            # reuse the cache
            return self.cached_static_polygons

        # no cache, or a stale one: rebuild
        if self.previousEquipmentID and self.cached_exclude_nodes != exclude_nodes:
            self.globalVar.printTerminal(
                f"[{self.getTime()}][LocalPlanner] Rebuilding static obstacles cache (exclude: {exclude_nodes})"
            )

        obstacles = self.globalVar.getObstacleInfo()
        static_polygons = []

        # current pose and goal, computed once
        current_point = Point(self.curPose[0], self.curPose[1])
        goal_point = Point(self.goal[0], self.goal[1]) if self.goal else None

        for obs in obstacles:
            # the current goal is not treated as an obstacle
            if 'nodeID' in obs and obs['nodeID'] in exclude_nodes:
                continue

            pos = obs['position']
            bbox = obs['boundingBox']

            x_min = pos['x'] - bbox['width'] / 2
            x_max = pos['x'] + bbox['width'] / 2
            y_min = pos['y'] - bbox['height'] / 2
            y_max = pos['y'] + bbox['height'] / 2

            # polygon coordinates
            polygon = Polygon([
                (x_min, y_min),
                (x_max, y_min),
                (x_max, y_max),
                (x_min, y_max)
            ])

            # drop the polygon containing the robot, or it can never escape
            if polygon.contains(current_point):
                continue

            # drop the polygon containing the goal, or it can never be entered
            if goal_point and polygon.contains(goal_point):
                continue

            static_polygons.append(polygon)

        # store in the cache
        self.cached_static_polygons = static_polygons
        self.cached_exclude_nodes = exclude_nodes.copy()

        return static_polygons

    def check_distance(self):
        """Distance from the current pose to the goal."""
        if not self.curPose or not self.goal:
            return float('inf')

        current_position = np.array(self.curPose[:2])
        goal_position = np.array(self.goal)
        distance = np.linalg.norm(goal_position - current_position)

        return distance

    def check_obstacle_distance(self):
        """Distance from the current pose to the nearest obstacle."""
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
        """Distance to another agent."""
        if not self.curPose or agent_id not in self.other_agents:
            return float('inf')

        current_position = np.array(self.curPose[:2])
        agent_data = self.other_agents[agent_id]
        agent_position = np.array(agent_data['pos'])
        return np.linalg.norm(agent_position - current_position)

    def predict_collision_risk(self, agent_id):
        """Predict a collision with a peer robot from the relative velocity."""
        if agent_id not in self.other_agents:
            return False

        agent_data = self.other_agents[agent_id]
        my_pos = np.array(self.curPose[:2])
        my_vel = np.array([self.curPose[3] * np.cos(self.curPose[2]),
                          self.curPose[3] * np.sin(self.curPose[2])])

        other_pos = np.array(agent_data['pos'])
        other_vel = np.array(agent_data['vel'][:2]) if len(
            agent_data['vel']) >= 2 else np.array([0.0, 0.0])

        # relative position and velocity
        rel_pos = other_pos - my_pos
        rel_vel = other_vel - my_vel

        # distance
        distance = np.linalg.norm(rel_pos)

        # very close is always a risk
        if distance < self.collision_radius:
            return True

        # with almost no relative motion there is no risk
        rel_speed = np.linalg.norm(rel_vel)
        if rel_speed < 0.1:
            return False

        # a 3 s prediction horizon
        prediction_time = 3.0

        # extrapolate the future positions
        my_future_pos = my_pos + my_vel * prediction_time
        other_future_pos = other_pos + other_vel * prediction_time

        # the future distance
        future_distance = np.linalg.norm(other_future_pos - my_future_pos)

        # closing in the future counts as a risk
        if future_distance < self.collision_radius:
            return True

        # the dot product tells us whether they are approaching
        # a negative dot product means the two are closing
        if distance > 0:
            approaching = np.dot(rel_pos, rel_vel) < 0

            # closing and near: a risk
            if approaching and distance < self.safety_tolerance * 0.7:
                return True

        return False

    def get_next_waypoint(self):
        """Take the next waypoint from the path."""
        if not self.path or len(self.path) <= 1:
            return None

        # index of the waypoint closest to the current goal
        current_idx = self.find_current_waypoint_index()

        if current_idx is not None and current_idx < len(self.path) - 1:
            # the one after it
            next_wp = self.path[current_idx + 1]
            return (next_wp[0], next_wp[1])

        return None

    def find_current_waypoint_index(self):
        """Find which waypoint of the path the current goal corresponds to."""
        if not self.path or not self.goal:
            return None

        goal_array = np.array(self.goal)

        for i, waypoint in enumerate(self.path):
            wp_array = np.array([waypoint[0], waypoint[1]])
            distance = np.linalg.norm(goal_array - wp_array)

            # closer than 1.0 counts as the same point
            if distance < 1.0:
                return i

        # not found: assume the first waypoint
        return 0

    def replan_check(self):
        """Decide whether to replan, by tracking progress towards the goal.

        Returns True when neither the distance nor the position has changed
        appreciably for long enough, which indicates a deadlock or oscillation.
        """
        current_time = self.getTime()
        current_position = np.array(self.curPose[:2])
        current_distance = self.check_distance()

        # start tracking
        if self.replan_check_start_time is None:
            self.replan_check_start_time = current_time
            self.replan_check_initial_distance = current_distance
            self.replan_check_initial_position = current_position.copy()
            return False

        # elapsed time
        elapsed_time = current_time - self.replan_check_start_time

        # has the threshold been passed?
        if elapsed_time >= self.replan_check_threshold_time:
            # how much closer we got
            distance_improved = self.replan_check_initial_distance - current_distance

            # how far we moved
            position_movement = np.linalg.norm(
                current_position - self.replan_check_initial_position
            )

            # progress test
            needs_replan = False
            reason = ""

            # condition 1: the distance did not close enough
            if distance_improved < self.replan_check_distance_improvement:
                needs_replan = True
                reason = f"거리 개선 부족 (개선: {distance_improved:.2f}m < 필요: {self.replan_check_distance_improvement}m)"

            # condition 2: the robot barely moved, so it is circling in place
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
                # reset the tracking
                self.reset_replan_check()
                return True
            else:
                # enough progress: re-anchor the tracking
                self.globalVar.printTerminal(
                    f"[{current_time}][LocalPlanner({self.ID})] ✅ 진전 있음: 거리 {distance_improved:.2f}m 개선, {position_movement:.2f}m 이동"
                )
                self.reset_replan_check()
                return False

        return False

    def reset_replan_check(self):
        """Reset the progress tracking."""
        self.replan_check_start_time = None
        self.replan_check_initial_distance = None
        self.replan_check_initial_position = None
