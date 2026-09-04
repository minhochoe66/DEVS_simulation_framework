from SimulationEngine.ClassicDEVS.DEVSAtomicModel import DEVSAtomicModel
from modeling.Message.MsgCurPose import MsgCurrentPose
from SimulationEngine.Utility.Configurator import Configurator
from modeling.Message.MsgArrive import MsgArrive
import math
import numpy as np
import re
import csv
import os
import datetime


def extract_numbers(input_string):
    """Extract just the digits from a string."""
    numbers = re.findall(r'\d+', input_string)
    return ''.join(numbers)


class Maneuver(DEVSAtomicModel):
    """Atomic model that integrates the robot pose with differential-drive kinematics."""

    def __init__(self, ID, objConfiguration, globalVar=None):
        super().__init__(ID)

        self.objConfiguration = objConfiguration
        self.globalVar = globalVar  # optional; may be None

        # Ports
        self.addInputPort("RequestManeuver_I")
        self.addInputPort("Docking_I")
        self.addInputPort("StopSim")
        self.addInputPort("Undocking")  # undocking signal from Local_Planner
        self.addOutputPort("MyManeuverState_O")
        self.addInputPort("amrCommand")
        # DEVS state variable
        self.addStateVariable("state", "INIT")

        # state held directly as attributes
        self.dt = 0.1  # integration step

        # pose

        self.current_position_x = self.globalVar.getVehicleInfoByID(
            self.ID.split('_', 1)[0]).getCoordinates()[0]
        self.current_position_y = self.globalVar.getVehicleInfoByID(
            self.ID.split('_', 1)[0]).getCoordinates()[1]
        self.current_position_yaw = 0.0
        self.current_position_lin_vel = 0.0
        self.current_position_ang_vel = 0.0

        # goal position
        self.target_x = self.current_position_x
        self.target_y = self.current_position_y

        # vehicle dynamics parameters
        # self.wheelbase = 2.0
        # self.max_steer_angle = 0.6
        # self.look_ahead_distance = 3.0
        self.max_speed = self.objConfiguration.getConfiguration(
            'max_speed')
        self.min_speed = self.objConfiguration.getConfiguration(
            'min_speed')
        self.max_accel = self.objConfiguration.getConfiguration(
            'max_accel')
        self.max_yaw_rate = self.objConfiguration.getConfiguration(
            'max_yaw_rate')
        self.target_tolerance = self.objConfiguration.getConfiguration(
            'target_tolerance')

        # control variables
        self.last_acceleration = 0.0

        # path
        self.path = []
        self.current_waypoint_index = 0

        # debug flag
        self.debug_mode = True

    def funcExternalTransition(self, strPort, objEvent):
        state = self.getStateValue("state")

        if state == "WAIT":
            if strPort == "RequestManeuver_I":
                event_id = objEvent.strID.split('_', 1)[0]
                my_id = self.ID.split('_', 1)[0]

                print(
                    f"🎯 [MANEUVER_TARGET] {my_id}: RequestManeuver_I received")
                print(f"   Event from: {objEvent.strID} (base: {event_id})")
                print(f"   My ID: {self.ID} (base: {my_id})")
                print(f"   Match: {event_id == my_id}")

                if event_id == my_id:
                    new_target_x = objEvent.dblPositionX
                    new_target_y = objEvent.dblPositionY

                    print(
                        f"   ✅ New target: ({new_target_x:.2f}, {new_target_y:.2f})")
                    print(
                        f"   Current position: ({self.current_position_x:.2f}, {self.current_position_y:.2f})")

                    self.target_x = new_target_x
                    self.target_y = new_target_y
                    self.setStateValue("state", "Move")
                    print(f"   State changed to Move")
                else:
                    print(f"   ❌ IGNORED: Not my message")
            elif strPort == "amrCommand":
                if state == "WAIT":
                    if self.ID.split('_', 1)[0] == objEvent['amrID'].split('_', 1)[0]:
                        self.setStateValue("state", "Move")
        if state == "Move":
            if strPort == "RequestManeuver_I":
                if objEvent.strID.split('_', 1)[0] == self.ID.split('_', 1)[0]:
                    new_target_x = objEvent.dblPositionX
                    new_target_y = objEvent.dblPositionY

                    # update the goal
                    self.target_x = new_target_x
                    self.target_y = new_target_y

                    # move to the Move state
                    self.setStateValue("state", "Move")

                    self.continueTimeAdvance()

            elif strPort == "Docking_I":
                # a docking order mid-drive pins the robot at the dock and stops it
                if objEvent.strID.split('_', 1)[0] == self.ID.split('_', 1)[0]:
                    self.target_x = objEvent.dblPositionX
                    self.target_y = objEvent.dblPositionY
                    self.current_position_x = self.target_x
                    self.current_position_y = self.target_y
                    self.current_position_lin_vel = 0.0
                    self.current_position_ang_vel = 0.0

                    # yaw is pinned, with 0 rad pointing right
                    self.current_position_yaw = 0.0

                    vehicle_id = self.ID.split('_', 1)[0]
                    print(
                        f"🚗 [MANEUVER_DOCKING] {vehicle_id}: Moving to DOCKING position ({objEvent.dblPositionX}, {objEvent.dblPositionY})")
                    self.globalVar.getVehicleInfoByID(vehicle_id).setCoordinates(
                        [objEvent.dblPositionX, objEvent.dblPositionY])
                    self.setStateValue("state", "Docking")

                    print(
                        f"[{self.getTime()}][Maneuver] DOCKING: Set heading to IN direction (0°)")

        if strPort == "Complete_I":
            # stop on arrival
            self.current_position_lin_vel = 0.0
            self.current_position_ang_vel = 0.0
            self.setStateValue("state", "WAIT")

        # an undocking signal is accepted in any state
        if strPort == "Undocking_I":
            # undocking target
            event_id = objEvent.strID.split('_', 1)[0]
            my_id = self.ID.split('_', 1)[0]

            print(f"🚗 [MANEUVER_UNDOCKING] {my_id}: Undocking_I received")
            print(f"   Event from: {objEvent.strID} (base: {event_id})")
            print(f"   My ID: {self.ID} (base: {my_id})")
            print(f"   Match: {event_id == my_id}")

            if event_id == my_id:
                vehicle_id = self.ID.split('_', 1)[0]
                old_coords = self.globalVar.getVehicleInfoByID(
                    vehicle_id).getCoordinates()

                print(f"   ✅ Setting UNDOCKING position")
                print(f"   OLD coordinates: {old_coords}")
                print(
                    f"   NEW coordinates (outputPort): ({objEvent.dblPositionX}, {objEvent.dblPositionY})")

                self.current_position_x = objEvent.dblPositionX
                self.current_position_y = objEvent.dblPositionY

                # yaw is pinned, with 0 rad pointing right
                self.current_position_yaw = 0.0

                self.globalVar.getVehicleInfoByID(vehicle_id).setCoordinates(
                    [objEvent.dblPositionX, objEvent.dblPositionY])
                self.setStateValue("state", "Undocking")
            else:
                print(f"   ❌ IGNORED: Not my message")

        if state == "Docking_I":
            if objEvent.strID.split('_', 1)[0] == self.ID.split('_', 1)[0]:
                self.globalVar.getVehicleInfoByID(self.ID).setCoordinates(
                    [objEvent.dblPositionX, objEvent.dblPositionY])
                self.setStateValue("state", "Docking")

    def funcInternalTransition(self):
        """Internal transition."""
        state = self.getStateValue("state")
        if state == "Move":
            # integrate the pose
            self.update_position()
            # stay in Move; Global_Planner decides when the goal is reached
            self.setStateValue("state", "Move")

        elif state == "Backup":
            self.update_position(backward=True)
            # return to WAIT shortly, so Local_Planner can re-decide
            self.setStateValue("state", "WAIT")
        elif state == "INIT":
            self.setStateValue("state", "WAIT")
            # nothing happens in WAIT
        elif state == "Undocking":
            self.setStateValue("state", "WAIT")

    def funcOutput(self):
        """Output function."""
        state = self.getStateValue("state")
        if state == "Move" or state == "WAIT" or state == "INIT":

            # message carrying the current pose
            # derive the vehicle ID from the model ID: 'VEHICLE...001_maneuver' -> 'VEHICLE...001'
            vehicle_id = self.ID.rsplit(
                '_', 1)[0] if '_' in self.ID else self.ID

            msg = MsgCurrentPose(
                vehicle_id,
                self.current_position_x,
                self.current_position_y,
                self.current_position_yaw,
                self.current_position_lin_vel,
                self.current_position_ang_vel,
            )

            self.addOutputEvent("MyManeuverState_O", msg)
        elif state == "Docking":
            vehicle_id = self.ID.split('_', 1)[0]

            # read the latest coordinates from GlobalVar
            x = self.globalVar.getVehicleInfoByID(
                vehicle_id).getCoordinates()[0]
            y = self.globalVar.getVehicleInfoByID(
                vehicle_id).getCoordinates()[1]

            # pin the internal pose and publish it
            self.current_position_x = x
            self.current_position_y = y
            self.current_position_lin_vel = 0.0
            self.current_position_ang_vel = 0.0
            # yaw is pinned, with 0 rad pointing right
            self.current_position_yaw = 0.0

            msg = MsgCurrentPose(
                vehicle_id,
                x,
                y,
                self.current_position_yaw,
                self.current_position_lin_vel,
                self.current_position_ang_vel,
            )
            self.addOutputEvent("MyManeuverState_O", msg)
            return True
        elif state == "Undocking":
            vehicle_id = self.ID.split('_', 1)[0]

            # read the output-port coordinates GlobalVar has set
            x = self.globalVar.getVehicleInfoByID(
                vehicle_id).getCoordinates()[0]
            y = self.globalVar.getVehicleInfoByID(
                vehicle_id).getCoordinates()[1]

            # keep the internal pose in step
            self.current_position_x = x
            self.current_position_y = y
            self.current_position_lin_vel = 0.0
            self.current_position_ang_vel = 0.0
            # yaw is pinned, with 0 rad pointing right
            self.current_position_yaw = 0.0

            msg = MsgCurrentPose(
                vehicle_id,
                x,
                y,
                self.current_position_yaw,
                self.current_position_lin_vel,
                self.current_position_ang_vel,
            )
            self.addOutputEvent("MyManeuverState_O", msg)
            return True

    def funcTimeAdvance(self):
        """Time advance."""
        state = self.getStateValue("state")
        if state == "Move":
            return 0.1
        elif state == "WAIT":
            return float('inf')
        elif state == "Docking":
            return 1
        elif state == "Undocking":
            return 0
        elif state == "INIT":
            return 0

    def update_position(self, backward: bool = False):
        """Integrate the pose one step from the linear and angular velocity."""

        # the tracking_only profile is handled separately
        current_profile = self.objConfiguration.getConfiguration(
            "currentProfile")
        if current_profile == "tracking_only":
            # stop once close enough to the goal
            dx = self.target_x - self.current_position_x
            dy = self.target_y - self.current_position_y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < 0.1:  # within 0.1 m
                self.current_position_lin_vel = 0.0
                self.current_position_ang_vel = 0.0
                return

        # vector to the goal
        dx = self.target_x - self.current_position_x
        dy = self.target_y - self.current_position_y
        distance = math.sqrt(dx**2 + dy**2)

        # debug trace of the goal bearing, at most once per second
        if not hasattr(self, '_last_debug_time'):
            self._last_debug_time = 0

        if self.getTime() - self._last_debug_time >= 1.0:
            vehicle_id = self.ID.split('_', 1)[0]
            print(f"🚗 [MANEUVER_MOVE] {vehicle_id}: Moving")
            print(
                f"   Current: ({self.current_position_x:.2f}, {self.current_position_y:.2f})")
            print(
                f"   Target (from DWA): ({self.target_x:.2f}, {self.target_y:.2f})")
            print(f"   Distance to DWA target: {distance:.2f}m")
            print(
                f"   Yaw: {self.current_position_yaw:.2f}, Vel: {self.current_position_lin_vel:.2f}")
            self._last_debug_time = self.getTime()

        # bearing to the goal
        target_angle = math.atan2(dy, dx)

        # heading error
        angle_diff = target_angle - self.current_position_yaw
        # normalise to -pi .. pi
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # the closer the goal, the lower the angular gain

        angular_gain = min(1.5, max(0.7, distance / 3.0))  # distance-dependent gain

        # angular velocity command
        target_angular_velocity = np.clip(
            angular_gain * angle_diff,  # distance-based factor
            -self.max_yaw_rate,
            self.max_yaw_rate
        )

        # limit angular acceleration, to keep turns smooth
        angular_accel = min(1.0, max(0.5, distance / 5.0))  # distance-dependent acceleration limit
        angular_vel_diff = target_angular_velocity - self.current_position_ang_vel
        angular_vel_diff = np.clip(
            angular_vel_diff, -angular_accel * self.dt, angular_accel * self.dt)
        angular_velocity = self.current_position_ang_vel + angular_vel_diff

        # linear velocity command
        base_velocity = self.max_speed

        # speed is held constant rather than scaled by distance
        distance_factor = 1.0  # always the maximum speed

        # only the heading-error slowdown is kept, for stability
        angle_factor = 1.0 - (abs(angle_diff) / math.pi) * \
            0.3  # slow down with heading error
        angle_factor = max(0.7, angle_factor)  # never below 70%

        # while turning, slow down only slightly
        angular_factor = 1.0 - (abs(angular_velocity) /
                                self.max_yaw_rate) * 0.2  # a small slowdown
        angular_factor = max(0.8, angular_factor)  # never below 80%

        # target linear velocity
        target_velocity = base_velocity * angular_factor * distance_factor * angle_factor

        # acceleration limit
        max_accel = min(1.0, self.objConfiguration.getConfiguration(
            'max_accel') or 1.0)
        velocity_diff = target_velocity - self.current_position_lin_vel
        acceleration = np.clip(velocity_diff / self.dt, -max_accel, max_accel)

        # smooth the acceleration with a first-order filter
        alpha = 0.8
        filtered_accel = acceleration * alpha + \
            (1 - alpha) * self.last_acceleration
        self.last_acceleration = filtered_accel

        # update the linear velocity, holding a floor
        min_speed = self.min_speed  # the floor comes from the configuration

        # in reverse, the linear velocity may go negative
        if backward:
            # Local_Planner supplies a short goal behind the robot for reversing
            base_velocity = -min(self.max_speed * 0.5, 0.8)  # reverse speed is capped
            min_back = -min_speed
            linear_velocity = np.clip(
                self.current_position_lin_vel + (-abs(filtered_accel)) * self.dt, -self.max_speed, -min_speed)
        else:
            linear_velocity = np.clip(
                self.current_position_lin_vel + filtered_accel * self.dt, min_speed, self.max_speed)

        # integrate the heading
        new_yaw = self.current_position_yaw + angular_velocity * self.dt

        # integrate the position
        new_x = self.current_position_x + \
            linear_velocity * math.cos(new_yaw) * self.dt
        new_y = self.current_position_y + \
            linear_velocity * math.sin(new_yaw) * self.dt

        # store the new state
        self.current_position_yaw = new_yaw
        self.current_position_x = new_x
        self.current_position_y = new_y
        self.current_position_lin_vel = linear_velocity
        self.current_position_ang_vel = angular_velocity

        # if self.debug_mode:
        #     print(f"[{self.getTime()}][Maneuver-Debug] Updated position: ({new_x:.2f}, {new_y:.2f})")
        #     print(f"[{self.getTime()}][Maneuver-Debug] Velocity: {linear_velocity:.2f}, Angular velocity: {angular_velocity:.2f}")

    def funcSelect(self):
        """Select function."""
        pass

    def external_transition(self, strPort, objEvent):
        """Helper that lets the class invoke its own external transition."""
        self.funcExternalTransition(strPort, objEvent)
        return
