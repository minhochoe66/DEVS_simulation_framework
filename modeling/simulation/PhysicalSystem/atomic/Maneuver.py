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
    """ 정규 표현식을 사용하여 문자열에서 숫자만 추출 """
    numbers = re.findall(r'\d+', input_string)
    return ''.join(numbers)


class Maneuver(DEVSAtomicModel):
    """ 개선된 Maneuver 클래스 구현 """

    def __init__(self, ID, objConfiguration, globalVar=None):
        super().__init__(ID)

        # DEVS 모델 필수 설정
        self.objConfiguration = objConfiguration
        self.globalVar = globalVar  # optional; may be None

        # ID 설정

        # 포트 설정
        self.addInputPort("RequestManeuver_I")
        self.addInputPort("Docking_I")
        self.addInputPort("StopSim")
        self.addInputPort("Undocking")  # Local_Planner로부터 언도킹 신호
        self.addOutputPort("MyManeuverState_O")
        self.addInputPort("amrCommand")
        # 기본 DEVS 상태 변수
        self.addStateVariable("state", "INIT")

        # 직접 속성으로 관리할 상태 변수들
        self.dt = 0.1  # 시간 간격

        # 위치 정보 저장

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

        # 제어 관련 변수
        self.last_acceleration = 0.0

        # 경로 계획
        self.path = []
        self.current_waypoint_index = 0

        # 디버깅 플래그
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

                    # 목표 위치 업데이트
                    self.target_x = new_target_x
                    self.target_y = new_target_y

                    # 상태를 Move로 변경
                    self.setStateValue("state", "Move")

                    self.continueTimeAdvance()

            elif strPort == "Docking_I":
                # 이동 중 도킹 지시: 즉시 도킹 좌표로 고정하고 정지
                if objEvent.strID.split('_', 1)[0] == self.ID.split('_', 1)[0]:
                    self.target_x = objEvent.dblPositionX
                    self.target_y = objEvent.dblPositionY
                    self.current_position_x = self.target_x
                    self.current_position_y = self.target_y
                    self.current_position_lin_vel = 0.0
                    self.current_position_ang_vel = 0.0

                    # 오른쪽(우) 방향을 0rad로 정의: yaw 고정
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
            # 도착 시 움직임 멈춤
            self.current_position_lin_vel = 0.0
            self.current_position_ang_vel = 0.0
            self.setStateValue("state", "WAIT")

        # 언도킹 신호 처리 (모든 상태에서)
        if strPort == "Undocking_I":
            # 언도킹 목표 좌표 설정
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

                # 오른쪽(우) 방향을 0rad로 정의: yaw 고정
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
        """ 내부 상태 전이 로직 """
        state = self.getStateValue("state")
        if state == "Move":
            # 위치 업데이트만 수행 (기동 제어)
            self.update_position()
            # Move 상태 유지 - 목표 도달 판단은 Global Planner가 담당
            self.setStateValue("state", "Move")

        elif state == "Backup":
            self.update_position(backward=True)
            # 짧은 시간 후 WAIT으로 복귀 (LocalPlanner가 재판단)
            self.setStateValue("state", "WAIT")
        elif state == "INIT":
            self.setStateValue("state", "WAIT")
            # WAIT 상태에서는 아무것도 하지 않음
        elif state == "Undocking":
            self.setStateValue("state", "WAIT")

    def funcOutput(self):
        """ 출력 생성 """
        state = self.getStateValue("state")
        if state == "Move" or state == "WAIT" or state == "INIT":

            # 현재 위치 정보를 담은 메시지 생성
            # vehicle ID 추출 (예: 'VEHICLE0000000000001_maneuver' -> 'VEHICLE0000000000001')
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

            # 출력 이벤트 추가
            self.addOutputEvent("MyManeuverState_O", msg)
        elif state == "Docking":
            vehicle_id = self.ID.split('_', 1)[0]

            # GlobalVar에서 최신 좌표 가져오기
            x = self.globalVar.getVehicleInfoByID(
                vehicle_id).getCoordinates()[0]
            y = self.globalVar.getVehicleInfoByID(
                vehicle_id).getCoordinates()[1]

            # 내부 좌표 고정 및 발행
            self.current_position_x = x
            self.current_position_y = y
            self.current_position_lin_vel = 0.0
            self.current_position_ang_vel = 0.0
            # 오른쪽(우) 방향을 0rad로 정의: yaw 고정
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

            # GlobalVar에서 업데이트된 좌표 읽어오기 (outputPort 좌표)
            x = self.globalVar.getVehicleInfoByID(
                vehicle_id).getCoordinates()[0]
            y = self.globalVar.getVehicleInfoByID(
                vehicle_id).getCoordinates()[1]

            # 내부 좌표도 업데이트
            self.current_position_x = x
            self.current_position_y = y
            self.current_position_lin_vel = 0.0
            self.current_position_ang_vel = 0.0
            # 오른쪽(우) 방향을 0rad로 정의: yaw 고정
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
        """ 시간 전이 함수 """
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
        """개선된 위치 정보 업데이트 로직"""

        # tracking_only 모드에서는 특별한 처리
        current_profile = self.objConfiguration.getConfiguration(
            "currentProfile")
        if current_profile == "tracking_only":
            # 목표까지의 거리가 매우 가까우면 정지
            dx = self.target_x - self.current_position_x
            dy = self.target_y - self.current_position_y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < 0.1:  # 0.1m 이하면 정지
                self.current_position_lin_vel = 0.0
                self.current_position_ang_vel = 0.0
                return

        # 목표까지의 벡터 계산
        dx = self.target_x - self.current_position_x
        dy = self.target_y - self.current_position_y
        distance = math.sqrt(dx**2 + dy**2)

        # 디버깅: 목표 방향 확인 (1초마다만 출력)
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

        # 목표 각도 계산
        target_angle = math.atan2(dy, dx)

        # 현재 방향과 목표 방향 간의 각도 차이 계산
        angle_diff = target_angle - self.current_position_yaw
        # 각도 정규화 (-π ~ π)
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # 목표에 가까워질수록 더 낮은 각속도 계수 사용

        # 거리가 가까워질수록 더 세밀한 제어
        angular_gain = min(1.5, max(0.7, distance / 3.0))  # 거리에 따른 가변 게인

        # 각속도 제어 - 거리에 따른 보정
        target_angular_velocity = np.clip(
            angular_gain * angle_diff,  # 거리 기반 가변 계수
            -self.max_yaw_rate,
            self.max_yaw_rate
        )

        # 각속도 변화율 제한 - 부드러운 회전을 위해 조정
        angular_accel = min(1.0, max(0.5, distance / 5.0))  # 거리에 따른 가변 가속도
        angular_vel_diff = target_angular_velocity - self.current_position_ang_vel
        angular_vel_diff = np.clip(
            angular_vel_diff, -angular_accel * self.dt, angular_accel * self.dt)
        angular_velocity = self.current_position_ang_vel + angular_vel_diff

        # 선속도 제어 - 목표지점 근처에서 감속
        base_velocity = self.max_speed

        # 거리 기반 속도 변화 제거 - 일정한 속도 유지
        distance_factor = 1.0  # 항상 최대 속도 유지

        # 각도 차이에 따른 약간의 속도 조절만 유지 (안정성을 위해)
        angle_factor = 1.0 - (abs(angle_diff) / math.pi) * \
            0.3  # 각도 차이에 따른 약간의 감속
        angle_factor = max(0.7, angle_factor)  # 최소 70%

        # 회전 시 약간의 속도 감소 - 안정성을 위해 최소한만 유지
        angular_factor = 1.0 - (abs(angular_velocity) /
                                self.max_yaw_rate) * 0.2  # 최소한의 감속
        angular_factor = max(0.8, angular_factor)  # 최소 80%

        # 최종 목표 속도 계산 - 일정한 속도 유지
        target_velocity = base_velocity * angular_factor * distance_factor * angle_factor

        # 가속도 제한 - 더 빠른 가속 허용
        max_accel = min(1.0, self.objConfiguration.getConfiguration(
            'max_accel') or 1.0)  # 가속도 감소 (1.2 -> 1.0)
        velocity_diff = target_velocity - self.current_position_lin_vel
        acceleration = np.clip(velocity_diff / self.dt, -max_accel, max_accel)

        # 필터링된 가속도 계산 - 반응성 증가
        alpha = 0.8  # 필터링 계수 증가 (0.7 -> 0.8)
        filtered_accel = acceleration * alpha + \
            (1 - alpha) * self.last_acceleration
        self.last_acceleration = filtered_accel

        # 선속도 업데이트 - 일정한 최소 속도 유지
        min_speed = self.min_speed  # 설정값 사용하여 일정한 최소 속도 유지

        # 후진 모드: 선속도 음수 허용, 목표를 뒤쪽으로 향하게 함
        if backward:
            # 목표 각도 반전: 현재 yaw 반대 방향으로 고정된 짧은 목표를 LocalPlanner가 보내줌
            base_velocity = -min(self.max_speed * 0.5, 0.8)  # 후진은 제한
            min_back = -min_speed
            linear_velocity = np.clip(
                self.current_position_lin_vel + (-abs(filtered_accel)) * self.dt, -self.max_speed, -min_speed)
        else:
            linear_velocity = np.clip(
                self.current_position_lin_vel + filtered_accel * self.dt, min_speed, self.max_speed)

        # 방향 업데이트
        new_yaw = self.current_position_yaw + angular_velocity * self.dt

        # 위치 업데이트
        new_x = self.current_position_x + \
            linear_velocity * math.cos(new_yaw) * self.dt
        new_y = self.current_position_y + \
            linear_velocity * math.sin(new_yaw) * self.dt

        # 상태 업데이트
        self.current_position_yaw = new_yaw
        self.current_position_x = new_x
        self.current_position_y = new_y
        self.current_position_lin_vel = linear_velocity
        self.current_position_ang_vel = angular_velocity

        # if self.debug_mode:
        #     print(f"[{self.getTime()}][Maneuver-Debug] Updated position: ({new_x:.2f}, {new_y:.2f})")
        #     print(f"[{self.getTime()}][Maneuver-Debug] Velocity: {linear_velocity:.2f}, Angular velocity: {angular_velocity:.2f}")

    def funcSelect(self):
        """ 선택 함수 """
        pass

    def external_transition(self, strPort, objEvent):
        """Maneuver 클래스 내부에서 사용하기 위한 외부 전이 처리 메서드"""
        # 실제 DEVS 함수인 funcExternalTransition을 호출
        self.funcExternalTransition(strPort, objEvent)
        return
