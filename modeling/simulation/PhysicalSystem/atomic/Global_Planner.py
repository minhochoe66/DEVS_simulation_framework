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

        # Environment
        self.map_width = 100
        self.map_height = 100
        self.obstacles = []

        self.AMR_Global_Planner_ID = ID

        # Robot and job state
        self.amr_id = None
        self.amr_position = None
        self.amr_yaw = None
        self.amr_velocity = None
        self.task_id = None
        self.task_from = None  # pickup position from the transport command (equipment IN)
        self.task_to = None    # destination position from the transport command (equipment IN)
        self.task_from_nodeID = None  # node ID of the pickup machine
        self.task_to_nodeID = None    # node ID of the destination machine
        self.planned_path = []
        self.path = []
        self.start = None

        # Transport phase
        # TO_FROM:        driving to the pickup machine
        # AT_FROM:        at the pickup machine, waiting to load
        # TO_DESTINATION: driving to the destination machine
        # AT_DESTINATION: at the destination machine, unloaded
        self.transportPhase = None
        self.currentGoal = None  # goal position of the current phase
        self.currentGoalNodeID = None  # goal node ID of the current phase

        # Cached obstacle geometry
        self.cached_obstacle_polygons = None  # cached shapely polygons
        self.cached_expanded_polygons = None  # cached polygons with the safety margin applied
        self.cache_safety_margin = 3.0

        # the obstacles are preprocessed once, at start-up
        self._preprocess_obstacles()

        # State variable
        self.addStateVariable("state", "INIT")

        # Ports
        self.addInputPort("Task_I")           # transport order
        # bare move command, no jobID
        self.addInputPort("amrGoCommand")
        self.addInputPort("ManeuverState_I")  # AMR pose
        self.addInputPort("Replan")           # replan request
        self.addInputPort("StopSim")          # stop the simulation

        self.addOutputPort("GlobalWaypoint_O")  # emit the waypoints
        self.addOutputPort("RequestManeuver")   # request to Maneuver

    def funcExternalTransition(self, strPort, objEvent):
        state = self.getStateValue("state")

        # docking complete: the job is done, go back to waiting
        if strPort == "Docking_I":
            self.setStateValue("state", "WAIT")
            self.globalVar.printTerminal(
                f"[{self.getTime()}][GlobalPlanner({self.ID})] Docking/WaitingArea completed, now WAITING for next task"
            )
            return
        if strPort == "Undocking_I":
            self.setStateValue("state", "WAIT")
        # amrGoCommand is a give-way order, accepted in any state
        if strPort == "amrGoCommand":
            # bare move command from FleetManagement, with no jobID
            if self.ID.split('_', 1)[0] == objEvent['amrID'].split('_', 1)[0]:
                planner_vehicle_id = self.ID.split('_')[0]
                task_vehicle_id = objEvent['amrID'].split('_')[0] if isinstance(
                    objEvent, dict) and 'amrID' in objEvent else None

                if task_vehicle_id and planner_vehicle_id in task_vehicle_id:
                    action = objEvent.get('action', 'GO')

                    if action == 'GO_WAITING':
                        # head for a waiting area
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] 🚨 EMERGENCY GO_WAITING Command (State: {state}) for AMR {objEvent['amrID']}"
                        )

                        # read the goal out of the command
                        self.amr_id = objEvent['amrID']
                        self.task_id = None  # no jobID

                        # only the goal position is set
                        target_x = objEvent['x']
                        target_y = objEvent['y']
                        areaID = objEvent.get('areaID', None)

                        # plan from the robot's current position
                        vehicleInfo = self.globalVar.getVehicleInfoByID(
                            self.amr_id)
                        if vehicleInfo:
                            latest_coords = vehicleInfo.getCoordinates()
                            self.amr_position = (
                                latest_coords[0], latest_coords[1])

                        self.task_from = self.amr_position  # current position
                        self.task_to = (target_x, target_y)  # goal position
                        self.task_from_nodeID = None  # no node ID
                        self.task_to_nodeID = None  # no node ID

                        # enter the WAITING phase
                        self.transportPhase = "WAITING"
                        self.currentGoal = self.task_to
                        self.currentGoalNodeID = areaID  # WaitingArea ID

                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] GO_WAITING Command: Move to WaitingArea {areaID} at ({target_x}, {target_y})"
                        )

                        # switch to planning
                        self.setStateValue("state", "GPP")
                        return  # act immediately
                    else:
                        # an ordinary move command
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] 🚨 EMERGENCY GO Command (State: {state}) for AMR {objEvent['amrID']}"
                        )

                        # read the goal out of the command
                        self.amr_id = objEvent['amrID']
                        self.task_id = None  # no jobID

                        # plan only from here to the goal
                        target_x = objEvent['x']
                        target_y = objEvent['y']

                        # plan from the robot's current position
                        vehicleInfo = self.globalVar.getVehicleInfoByID(
                            self.amr_id)
                        if vehicleInfo:
                            latest_coords = vehicleInfo.getCoordinates()
                            self.amr_position = (
                                latest_coords[0], latest_coords[1])

                        self.task_from = self.amr_position  # current position
                        self.task_to = (target_x, target_y)  # goal position
                        self.task_from_nodeID = None  # no node ID
                        self.task_to_nodeID = None  # no node ID

                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] GO Command: Move to ({target_x}, {target_y})"
                        )
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] Current position: {self.amr_position}"
                        )

                        # a bare move has no transport phase
                        self.transportPhase = None
                        self.currentGoal = self.task_to
                        self.currentGoalNodeID = None

                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][GlobalPlanner({self.ID})] GO Command - Simple movement (no job)"
                        )

                        # switch to planning
                        self.setStateValue("state", "GPP")
                        return  # act immediately
                else:
                    # ignore commands addressed to another AMR
                    self.continueTimeAdvance()
            return  # amrGoCommand handled

        # a new job is accepted only in INIT or WAIT
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

                        # read the job
                        self.amr_id = objEvent['amrID']
                        self.task_id = objEvent['jobID']

                        # store the pickup and destination, both position and node ID
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

                        # the transport action decides which phase to start in
                        action = objEvent.get('action', 'TRANSPORT')

                        if action == 'TRANSPORT_NEXT':
                            # after undocking, go straight to the TO_DESTINATION phase
                            self.transportPhase = "TO_DESTINATION"
                            self.currentGoal = self.task_to
                            self.currentGoalNodeID = self.task_to_nodeID

                            # refresh the robot pose after undocking
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
                            # an ordinary transport heads for the pickup machine first
                            self.transportPhase = "TO_FROM"
                            self.currentGoal = self.task_from
                            self.currentGoalNodeID = self.task_from_nodeID

                            self.globalVar.printTerminal(
                                f"[{self.getTime()}][GlobalPlanner({self.ID})] Phase: TO_FROM - Moving to pickup location"
                            )

                        # switch to planning
                        self.setStateValue("state", "GPP")
                    else:
                        # ignore jobs addressed to another AMR
                        self.continueTimeAdvance()
            elif strPort == "Replan":
                # replan request from Local_Planner
                # compare the vehicle numbers

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
                # update the AMR pose
                if self.ID.split('_', 1)[0] == objEvent.strID:
                    self.update_data(objEvent)
                else:
                    self.continueTimeAdvance()

        # while in PLAN the robot is driving, so only Replan and ManeuverState_I are handled
        elif state == "PLAN":
            if strPort == "Task_I":
                # a new job is ignored while driving
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][GlobalPlanner({self.ID})] BUSY - Ignoring new task (currently in PLAN state)"
                )
                self.continueTimeAdvance()

            elif strPort == "Replan":
                # replan request from Local_Planner
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
                # update the AMR pose
                if self.ID.split('_', 1)[0] == objEvent.strID:
                    self.update_data(objEvent)
                else:
                    self.continueTimeAdvance()

        # in any other state, only ManeuverState_I is handled
        elif strPort == "ManeuverState_I":
            # the pose is updated in every state
            if self.ID.split('_', 1)[0] == objEvent.strID:
                self.update_data(objEvent)
            else:
                self.continueTimeAdvance()

    def funcInternalTransition(self):
        state = self.getStateValue("state")

        if state == "GPP" and self.currentGoal is not None:
            # plan the path
            if self.amr_position and self.currentGoal:
                # exclude the goal and start nodes from the obstacle set
                exclude_nodes = []

                # exclude the goal node, waiting areas included
                if self.currentGoalNodeID:
                    exclude_nodes.append(self.currentGoalNodeID)
                    # for waiting areas, a matching prefix is enough to exclude
                    if isinstance(self.currentGoalNodeID, str) and self.currentGoalNodeID.startswith('WAITING_AREA'):
                        exclude_nodes.append(self.currentGoalNodeID)

                # exclude the start node: the output port of the machine the robot sits at
                if self.transportPhase == "TO_DESTINATION" and self.task_from_nodeID:
                    # in TO_DESTINATION the robot departs from the previous machine's output port
                    start_nodeID = self.task_from_nodeID.replace('_IN', '_OUT')
                    exclude_nodes.append(start_nodeID)
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][GlobalPlanner] Excluding start node {start_nodeID} from obstacles"
                    )

                start_time = time.time()
                # plan against the cached obstacles
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
                    # with no path found, fall back to a straight line
                    print(
                        f"[GlobalPlanner] No path found to {phase_name}, using direct path")
                    self.planned_path = [
                        (self.currentGoal[0], self.currentGoal[1])]

                # switch to emitting the path
                self.setStateValue("state", "SEND_PATH")

        elif state == "SEND_PATH":
            # after the output, move to PLAN
            self.setStateValue("state", "PLAN")

    def funcOutput(self):
        state = self.getStateValue("state")

        if state == "SEND_PATH" and self.planned_path and len(self.planned_path) > 0:
            # emit the first waypoint
            first_waypoint = self.planned_path[0]
            x, y = first_waypoint[0], first_waypoint[1]

            phase_name = "FROM Equipment" if self.transportPhase == "TO_FROM" else "TO Equipment"
            print(
                f"[{self.getTime()}][GlobalPlanner] Sending waypoint: ({x}, {y}) - Phase: {phase_name}")

            # the message carries the transport phase and the goal node ID
            objRequestMessage = MsgManeuverState(
                self.ID.split('_', 1)[0] + '_maneuver_amr',
                x,  # dblPositionX
                y,  # dblPositionY
                path=self.planned_path,  # the whole path
                transportPhase=self.transportPhase,  # "TO_FROM" or "TO_DESTINATION"
                goalNodeID=self.currentGoalNodeID  # current goal node ID
            )

            # emit
            self.addOutputEvent("GlobalWaypoint_O", objRequestMessage)

    def funcTimeAdvance(self):
        state = self.getStateValue("state")

        if state == "INIT":
            return float('inf')  # waiting for the first job
        elif state == "WAIT":
            return float('inf')  # waiting for the next job
        elif state == "GPP":
            return 0  # plan at once
        elif state == "SEND_PATH":
            return 0  # emit at once
        elif state == "PLAN":
            return float('inf')  # the robot is driving
        else:
            return 1

    def extract_vehicle_number(self, id_string):
        """Extract the vehicle number from a model ID."""
        match = re.search(r'Vehicle(\d+)', id_string)
        if match:
            return match.group(1)
        return None

    def update_data(self, objEvent):
        """Update the robot's pose and state."""
        # update the pose
        if hasattr(objEvent, 'x') and hasattr(objEvent, 'y'):
            self.amr_position = (objEvent.x, objEvent.y)
            print(
                f"[{self.getTime()}][GlobalPlanner] AMR position updated: {self.amr_position}")

            if hasattr(objEvent, 'strID'):
                self.amr_id = objEvent.strID

            # decide whether a replan is needed
            if self.getStateValue("state") == "GPP" and self.task_goal and not self.planned_path:
                print(
                    f"[{self.getTime()}][GlobalPlanner] Position received, ready for path planning")

        # additional fields
        if hasattr(objEvent, 'yaw'):
            self.amr_yaw = objEvent.yaw
        if hasattr(objEvent, 'lin_vel'):
            self.amr_velocity = objEvent.lin_vel

    def _preprocess_obstacles(self):
        """Preprocess every obstacle once, at start-up.

        Builds and caches the shapely polygons, together with the inflated
        versions that carry the safety margin.
        """
        obstacles = self.globalVar.getObstacleInfo()

        # 1. the plain polygons
        obstacle_polygons = []
        for obs in obstacles:
            pos = obs['position']
            bbox = obs['boundingBox']
            node_id = obs.get('nodeID', None)

            # waiting areas are not obstacles
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

            # keep the node ID too, for the exclusion test
            obstacle_polygons.append({
                'polygon': polygon,
                'nodeID': node_id
            })

        self.cached_obstacle_polygons = obstacle_polygons

        # 2. the polygons inflated by the safety margin
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
        """Read the obstacles from GlobalVar.

        Node IDs listed in exclude_nodes are dropped: leaving the current goal
        in the obstacle set would make it unreachable.
        """
        obstacles = self.globalVar.getObstacleInfo()
        obstacle_polygons = []

        if exclude_nodes is None:
            exclude_nodes = []

        for obs in obstacles:
            # the current goal is not treated as an obstacle
            if 'nodeID' in obs and obs['nodeID'] in exclude_nodes:
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][GlobalPlanner] Excluding target node {obs['nodeID']} from obstacles"
                )
                continue

            # waiting areas are not obstacles
            node_id = obs.get('nodeID', None)
            if node_id and isinstance(node_id, str) and node_id.startswith('WAITING_AREA'):
                continue

            pos = obs['position']
            bbox = obs['boundingBox']

            x_min = pos['x'] - bbox['width'] / 2
            x_max = pos['x'] + bbox['width'] / 2
            y_min = pos['y'] - bbox['height'] / 2
            y_max = pos['y'] + bbox['height'] / 2

            # convert to a coordinate list, ready for shapely
            polygon_coords = [
                (x_min, y_min),
                (x_max, y_min),
                (x_max, y_max),
                (x_min, y_max)
            ]
            obstacle_polygons.append(polygon_coords)

        return obstacle_polygons

    def generate_path_with_cached_obstacles(self, start, goal, exclude_nodes=None):
        """Build the global path using the cached obstacles."""
        if exclude_nodes is None:
            exclude_nodes = []

        # use the cached inflated polygons, minus the excluded nodes
        expanded_polygons = []
        for obs_data in self.cached_expanded_polygons:
            # skip the excluded nodes
            if obs_data['nodeID'] and obs_data['nodeID'] in exclude_nodes:
                continue
            expanded_polygons.append(obs_data['polygon'])

        # an inflated obstacle containing the start point must be dropped, or planning fails
        start_point = Point(start)
        cleaned_expanded = []
        for polygon in expanded_polygons:
            if not polygon.contains(start_point):
                cleaned_expanded.append(polygon)
        expanded_polygons = cleaned_expanded

        # if a straight line is clear, skip the search entirely
        if line_of_sight(start, goal, expanded_polygons):
            self.globalVar.printTerminal(
                f"[{self.getTime()}][GlobalPlanner] Direct path available, skipping pathfinding"
            )
            self.path = [goal]  # return just the goal
            return self.path

        # select the planning algorithm
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
            # the default is a straight line
            raw_path = [start, goal]

        if raw_path:
            # shorten the path with line-of-sight checks
            simplified_path = simplify_path_with_los(
                raw_path, expanded_polygons)

            # drop the start point; emit from the first waypoint on
            if len(simplified_path) == 1:
                self.path = simplified_path
            else:
                self.path = simplified_path[1:]

            # # path visualisation
            # self.plot_path_with_terrain(
            #     start, goal, simplified_path, expanded_polygons)
            return self.path
        else:
            print("[GlobalPlanner] No path found between start and goal")
            return []

    def generate_path(self, start, goal, terrain_polygons):
        """Build the global path. Compatibility path, not using the cache."""
        # this is reached via get_obstacle_polygons, so exclude_nodes has to be
        # inferred from terrain_polygons; passing it in directly would be better

        # convert terrain_polygons to shapely polygons
        polygons = [Polygon(p) for p in terrain_polygons]

        # apply the safety margin
        safety_margin = 3.0
        expanded_polygons = []
        for polygon in polygons:
            expanded_polygons.append(polygon.buffer(safety_margin))

        # is the start point inside an obstacle?
        start_point = Point(start)
        goal_point = Point(goal)

        # an inflated obstacle containing the start point must be dropped, or planning fails
        cleaned_expanded = []
        for polygon in expanded_polygons:
            if not polygon.contains(start_point):
                cleaned_expanded.append(polygon)
        expanded_polygons = cleaned_expanded
        # the goal needs no check: exclude_nodes has already removed it

        # select the planning algorithm
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
            # the default is a straight line
            raw_path = [start, goal]

        if raw_path:
            # shorten the path with line-of-sight checks
            simplified_path = simplify_path_with_los(
                raw_path, expanded_polygons)

            # drop the start point; emit from the first waypoint on
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
        """Plot the path, deriving the map extent from the coordinates."""
        fig, ax = plt.subplots(figsize=(14, 10))

        # gather the coordinates, to size the view
        all_x = []
        all_y = []

        # draw the obstacles
        for polygon in terrain_polygons:
            # a shapely polygon: take its coordinates
            if isinstance(polygon, Polygon):
                coords = list(polygon.exterior.coords)
            else:
                # already a coordinate list: use it as is
                coords = polygon

            # collect the coordinates
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

        # start and goal
        ax.plot(start[0], start[1], 'go', markersize=10, label='Start')
        ax.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')
        all_x.extend([start[0], goal[0]])
        all_y.extend([start[1], goal[1]])

        # the path
        if path:
            path_array = np.array(path)
            ax.plot(path_array[:, 0], path_array[:, 1],
                    'b-', linewidth=2, label='Path')
            all_x.extend(path_array[:, 0].tolist())
            all_y.extend(path_array[:, 1].tolist())

        # view extent, with the same 5.0 margin visualize.py uses
        if all_x and all_y:
            min_x, max_x = min(all_x), max(all_x)
            min_y, max_y = min(all_y), max(all_y)
            margin = 5.0
            ax.set_xlim(min_x - margin, max_x + margin)
            ax.set_ylim(min_y - margin, max_y + margin)
        else:
            # with no coordinates, fall back to a default extent
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
    """Shorten a path by joining waypoints that have line of sight."""
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
    """Test whether two points have an unobstructed line between them."""
    line = LineString([p1, p2])

    for polygon in terrain_polygons:
        # already a shapely polygon: use it as is
        if isinstance(polygon, Polygon):
            if line.intersects(polygon):
                return False
        else:
            # a coordinate list: convert it to a polygon
            if line.intersects(Polygon(polygon)):
                return False

    return True
