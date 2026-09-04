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
    """문자열에서 숫자만 뽑아낸다."""
    numbers = re.findall(r'\d+', input_string)
    return ''.join(numbers)


class Maneuver(DEVSAtomicModel):
    """차동 구동 운동학으로 로봇의 위치와 자세를 갱신하는 원자 모델."""

    def __init__(self, ID, objConfiguration, globalVar=None):
        super().__init__(ID)

        self.objConfiguration = objConfiguration
        self.globalVar = globalVar  # optional; may be None

        # 포트
        self.addInputPort("RequestManeuver_I")
        self.addInputPort("Docking_I")
        self.addInputPort("StopSim")
        self.addInputPort("Undocking")  # Local_Planner로부터 언도킹 신호
        self.addOutputPort("MyManeuverState_O")
        self.addInputPort("amrCommand")
        # DEVS 상태 변수
        self.addStateVariable("state", "INIT")

        # 속성으로 직접 관리하는 상태
        self.dt = 0.1  # 적분 간격

        # 위치와 자세

        self.current_position_x = self.globalVar.getVehicleInfoByID(
            self.ID.split('_', 1)[0]).getCoordinates()[0]
        self.current_position_y = self.globalVar.getVehicleInfoByID(
            self.ID.split('_', 1)[0]).getCoordinates()[1]
        self.current_position_yaw = 0.0
        self.current_position_lin_vel = 0.0
        self.current_position_ang_vel = 0.0

        # 목표 위치
        self.target_x = self.current_position_x
        self.target_y = self.current_position_y

        # 차량 동역학 파라미터
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

        # 제어 변수
        self.last_acceleration = 0.0

        # 경로
        self.path = []
        self.current_waypoint_index = 0

        # 디버그 플래그
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

                    # 목표 위치 갱신
                    self.target_x = new_target_x
                    self.target_y = new_target_y

                    # Move로 전이
                    self.setStateValue("state", "Move")

                    self.continueTimeAdvance()

            elif strPort == "Docking_I":
                # 주행 중 도킹 지시를 받으면 도킹 좌표에 고정하고 정지한다
                if objEvent.strID.split('_', 1)[0] == self.ID.split('_', 1)[0]:
                    self.target_x = objEvent.dblPositionX
                    self.target_y = objEvent.dblPositionY
                    self.current_position_x = self.target_x
                    self.current_position_y = self.target_y
                    self.current_position_lin_vel = 0.0
                    self.current_position_ang_vel = 0.0

                    # 오른쪽을 0 rad으로 두고 yaw를 고정한다
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
            # 도착하면 정지
            self.current_position_lin_vel = 0.0
            self.current_position_ang_vel = 0.0
            self.setStateValue("state", "WAIT")

        # 언도킹 신호는 어느 상태에서든 받는다
        if strPort == "Undocking_I":
            # 언도킹 목표 좌표
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

                # 오른쪽을 0 rad으로 두고 yaw를 고정한다
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
        """내부 상태 전이."""
        state = self.getStateValue("state")
        if state == "Move":
            # 위치만 적분한다
            self.update_position()
            # Move를 유지한다. 목표 도달 판단은 Global_Planner가 한다
            self.setStateValue("state", "Move")

        elif state == "Backup":
            self.update_position(backward=True)
            # 잠시 뒤 WAIT으로 돌아가 Local_Planner가 다시 판단하게 한다
            self.setStateValue("state", "WAIT")
        elif state == "INIT":
            self.setStateValue("state", "WAIT")
            # WAIT에서는 아무것도 하지 않는다
        elif state == "Undocking":
            self.setStateValue("state", "WAIT")

    def funcOutput(self):
        """출력 함수."""
        state = self.getStateValue("state")
        if state == "Move" or state == "WAIT" or state == "INIT":

            # 현재 자세를 담은 메시지
            # 모델 ID에서 차량 ID를 뽑는다 ('VEHICLE...001_maneuver' -> 'VEHICLE...001')
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

            # GlobalVar에서 최신 좌표를 읽는다
            x = self.globalVar.getVehicleInfoByID(
                vehicle_id).getCoordinates()[0]
            y = self.globalVar.getVehicleInfoByID(
                vehicle_id).getCoordinates()[1]

            # 내부 좌표를 고정하고 발행한다
            self.current_position_x = x
            self.current_position_y = y
            self.current_position_lin_vel = 0.0
            self.current_position_ang_vel = 0.0
            # 오른쪽을 0 rad으로 두고 yaw를 고정한다
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

            # GlobalVar가 갱신한 출력 포트 좌표를 읽는다
            x = self.globalVar.getVehicleInfoByID(
                vehicle_id).getCoordinates()[0]
            y = self.globalVar.getVehicleInfoByID(
                vehicle_id).getCoordinates()[1]

            # 내부 좌표도 맞춘다
            self.current_position_x = x
            self.current_position_y = y
            self.current_position_lin_vel = 0.0
            self.current_position_ang_vel = 0.0
            # 오른쪽을 0 rad으로 두고 yaw를 고정한다
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
        """시간 전이 함수."""
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
        """선속도와 각속도로 위치와 자세를 한 스텝 적분한다."""

        # tracking_only 프로파일은 따로 처리한다
        current_profile = self.objConfiguration.getConfiguration(
            "currentProfile")
        if current_profile == "tracking_only":
            # 목표에 충분히 가까우면 정지한다
            dx = self.target_x - self.current_position_x
            dy = self.target_y - self.current_position_y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < 0.1:  # 0.1 m 이하
                self.current_position_lin_vel = 0.0
                self.current_position_ang_vel = 0.0
                return

        # 목표까지의 벡터
        dx = self.target_x - self.current_position_x
        dy = self.target_y - self.current_position_y
        distance = math.sqrt(dx**2 + dy**2)

        # 목표 방향 디버그 출력. 1초에 한 번만
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

        # 목표 방위각
        target_angle = math.atan2(dy, dx)

        # 현재 방향과의 차이
        angle_diff = target_angle - self.current_position_yaw
        # -pi ~ pi로 정규화
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # 목표에 가까울수록 각속도 이득을 낮춘다

        angular_gain = min(1.5, max(0.7, distance / 3.0))  # 거리에 따른 가변 이득

        # 각속도 제어
        target_angular_velocity = np.clip(
            angular_gain * angle_diff,  # 거리 기반 계수
            -self.max_yaw_rate,
            self.max_yaw_rate
        )

        # 각가속도 제한. 회전을 부드럽게 한다
        angular_accel = min(1.0, max(0.5, distance / 5.0))  # 거리에 따른 가변 가속도
        angular_vel_diff = target_angular_velocity - self.current_position_ang_vel
        angular_vel_diff = np.clip(
            angular_vel_diff, -angular_accel * self.dt, angular_accel * self.dt)
        angular_velocity = self.current_position_ang_vel + angular_vel_diff

        # 선속도 제어
        base_velocity = self.max_speed

        # 거리에 따른 감속은 두지 않고 속도를 일정하게 유지한다
        distance_factor = 1.0  # 항상 최대 속도

        # 안정성을 위해 각도 오차에 따른 감속만 남긴다
        angle_factor = 1.0 - (abs(angle_diff) / math.pi) * \
            0.3  # 각도 오차에 따른 감속
        angle_factor = max(0.7, angle_factor)  # 최소 70 %

        # 회전 중에는 최소한만 감속한다
        angular_factor = 1.0 - (abs(angular_velocity) /
                                self.max_yaw_rate) * 0.2  # 최소한의 감속
        angular_factor = max(0.8, angular_factor)  # 최소 80 %

        # 목표 선속도
        target_velocity = base_velocity * angular_factor * distance_factor * angle_factor

        # 가속도 제한
        max_accel = min(1.0, self.objConfiguration.getConfiguration(
            'max_accel') or 1.0)
        velocity_diff = target_velocity - self.current_position_lin_vel
        acceleration = np.clip(velocity_diff / self.dt, -max_accel, max_accel)

        # 가속도를 1차 필터로 완만하게 만든다
        alpha = 0.8
        filtered_accel = acceleration * alpha + \
            (1 - alpha) * self.last_acceleration
        self.last_acceleration = filtered_accel

        # 선속도 갱신. 최소 속도는 유지한다
        min_speed = self.min_speed  # 최소 속도는 설정값을 쓴다

        # 후진 모드에서는 선속도가 음수가 될 수 있다
        if backward:
            # 후진 목표는 Local_Planner가 현재 yaw 반대편의 짧은 목표로 보내 준다
            base_velocity = -min(self.max_speed * 0.5, 0.8)  # 후진 속도는 제한한다
            min_back = -min_speed
            linear_velocity = np.clip(
                self.current_position_lin_vel + (-abs(filtered_accel)) * self.dt, -self.max_speed, -min_speed)
        else:
            linear_velocity = np.clip(
                self.current_position_lin_vel + filtered_accel * self.dt, min_speed, self.max_speed)

        # 방향 적분
        new_yaw = self.current_position_yaw + angular_velocity * self.dt

        # 위치 적분
        new_x = self.current_position_x + \
            linear_velocity * math.cos(new_yaw) * self.dt
        new_y = self.current_position_y + \
            linear_velocity * math.sin(new_yaw) * self.dt

        # 상태 반영
        self.current_position_yaw = new_yaw
        self.current_position_x = new_x
        self.current_position_y = new_y
        self.current_position_lin_vel = linear_velocity
        self.current_position_ang_vel = angular_velocity

        # if self.debug_mode:
        #     print(f"[{self.getTime()}][Maneuver-Debug] Updated position: ({new_x:.2f}, {new_y:.2f})")
        #     print(f"[{self.getTime()}][Maneuver-Debug] Velocity: {linear_velocity:.2f}, Angular velocity: {angular_velocity:.2f}")

    def funcSelect(self):
        """선택 함수."""
        pass

    def external_transition(self, strPort, objEvent):
        """클래스 내부에서 외부 전이를 직접 부르기 위한 헬퍼."""
        self.funcExternalTransition(strPort, objEvent)
        return
