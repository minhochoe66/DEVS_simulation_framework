from SimulationEngine.ClassicDEVS.DEVSAtomicModel import DEVSAtomicModel


class FleetManagement(DEVSAtomicModel):
    def __init__(self, strID, globalVar):
        super().__init__(strID)
        self.globalVar = globalVar

        # States: WAIT (대기), UPDATE (Scheduler 정보 전달), SEND_GO_COMMAND (대기 AMR 이동), SEND_COMMAND (AMR 명령 전송)
        self.stateList = ["WAIT", "UPDATE", "SEND_GO_COMMAND", "SEND_COMMAND"]
        self.state = self.stateList[0]

        # State Variables
        self.addStateVariable('strID', strID)

        # Input Ports
        self.addInputPort("amrPosition")  # AMR 위치 정보 수신
        self.addInputPort("taskAssign")   # Scheduler로부터 작업 할당 정보
        self.addInputPort("undockingComplete")  # Local_Planner로부터 언도킹 완료 신호

        # Output Ports
        self.addOutputPort("fleetInfo")   # Scheduler에게 Fleet 정보 전달
        self.addOutputPort("amrCommand")  # AMR에게 작업 지시

        # Variables
        self.amrPositions = {}  # AMR 위치 정보 저장
        self.lastUpdateTime = 0
        # AMR별 TransportCommand 저장 {amrID: transportCommand}
        self.jobSequences = {}
        # 현재 전송할 명령 (즉시 전송)
        self.pendingCommand = None
        self.watingARMCommand = None

    def funcExternalTransition(self, strPort, objEvent):
        if strPort == "amrPosition":
            # AMR 위치 정보 업데이트
            amrID = objEvent.strID
            self.amrPositions[amrID] = {
                'x': objEvent.x,
                'y': objEvent.y,
                'yaw': objEvent.yaw,
                'lin_vel': objEvent.lin_vel,
                'ang_vel': objEvent.ang_vel,
                'timestamp': self.getTime()
            }

            # GlobalVar의 Vehicle 정보도 업데이트
            vehicleInfo = self.globalVar.getVehicleInfoByID(amrID)
            if vehicleInfo:
                # 도킹 중이면 Equipment에서 설정한 좌표를 유지
                keep_equipment_coords = False
                if hasattr(vehicleInfo, 'getEquipmentID') and vehicleInfo.getEquipmentID():
                    keep_equipment_coords = True

                if not keep_equipment_coords:
                    vehicleInfo.setCoordinates([objEvent.x, objEvent.y])

                # 속도가 0이고 작업이 없으면 IDLE 상태로
                if objEvent.lin_vel < 0.1 and vehicleInfo.intJobID is None:
                    if vehicleInfo.strState != "IDLE":
                        vehicleInfo.setState("IDLE")
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][FleetManagement] AMR {amrID} is now IDLE"
                        )

            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] AMR {amrID} position updated: ({objEvent.x:.2f}, {objEvent.y:.2f}), State: {vehicleInfo.strState if vehicleInfo else 'Unknown'}"
            )

            # UPDATE 상태로 전환 (Scheduler에게 전달하기 위해)
            # WAIT 상태일 때만 전환 (COMMAND 처리 중이면 방해하지 않음)
            if self.state == "WAIT":
                self.state = self.stateList[1]  # UPDATE

            return True

        elif strPort == "taskAssign":
            # Scheduler로부터 TransportCommand 수신
            transportCommand = objEvent
            amrID = transportCommand.assignedAMR
            jobID = transportCommand.jobID

            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] Received TransportCommand: {transportCommand}"
            )

            # AMR이 새 작업을 할당받으면 모든 Equipment의 undockedAMRs에서 제거
            for equipmentInfo in self.globalVar.getEquipmentInfo().values():
                if amrID in equipmentInfo.undockedAMRs:
                    equipmentInfo.undockedAMRs.remove(amrID)
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] 🗑️ Removed AMR {amrID} from Equipment {equipmentInfo.strEquipmentID} undocked list"
                    )

            # WaitingArea는 점유 개념 미사용 → 해제 로직 제거

            # TransportCommand 저장 (AMR별로)
            self.jobSequences[amrID] = transportCommand
            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] 📋 Stored TransportCommand for AMR {amrID}: {transportCommand.commandID}"
            )

            # FROM Equipment로 이동 명령 생성 (TransportCommand에서 정보 조회)
            self.pendingCommand = {
                'amrID': amrID,
                'jobID': jobID,
                'commandID': transportCommand.commandID,
                'fromPosition': transportCommand.getFromPosition(),
                'toPosition': transportCommand.getToPosition(),
                'fromNodeID': transportCommand.getFromNodeID(),
                'toNodeID': transportCommand.getToNodeID(),
                'action': 'TRANSPORT',  # FROM → TO 운반
                'phase': 'TO_FROM'  # FROM Equipment로 이동
            }

            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] 📥 Command prepared for AMR {amrID}: {transportCommand.getFromNodeID()} → {transportCommand.getToNodeID()}"
            )

            # SEND_COMMAND 상태로 전환하여 즉시 전송
            self.state = self.stateList[3]  # SEND_COMMAND

            return True

        elif strPort == "undockingComplete_I":
            # Local_Planner로부터 언도킹 완료 신호 수신
            amrID = objEvent[0]
            equipmentID = objEvent[1]
            jobID = objEvent[2]
            transportPhase = objEvent[3]

            # 현재 AMR 위치 확인
            vehicleInfo = self.globalVar.getVehicleInfoByID(amrID)
            current_coords = vehicleInfo.getCoordinates() if vehicleInfo else "Unknown"

            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] 🚪 UNDOCKING complete: AMR {amrID} from {equipmentID}, Phase: {transportPhase}"
            )

            # === Phase별 분기 처리 (elif로 명확히 구분) ===

            # 1️⃣ WAITING phase 처리 (WaitingArea 도착 완료)
            if transportPhase == "WAITING":
                # WaitingArea 도착 처리 (점유 관리 없음)
                waitingArea = self.globalVar.getWaitingAreaInfoByID(
                    equipmentID)
                if waitingArea:
                    pass

                vehicleInfo = self.globalVar.getVehicleInfoByID(amrID)
                if vehicleInfo:
                    vehicleInfo.setState("IDLE")

                self.globalVar.printTerminal(
                    f"[{self.getTime()}][FleetManagement] 🅿️ AMR {amrID} arrived at WaitingArea {equipmentID} - Now IDLE"
                )
                # 대기 도착 시 해당 AMR 관련 보류 명령/이동 명령 정리
                if self.pendingCommand and self.pendingCommand.get('amrID') == amrID:
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] 🧹 Clearing pendingCommand for AMR {amrID} (arrived WAITING)"
                    )
                    self.pendingCommand = None
                if self.watingARMCommand and self.watingARMCommand.get('amrID') == amrID:
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] 🧹 Clearing watingARMCommand for AMR {amrID} (arrived WAITING)"
                    )
                    self.watingARMCommand = None
                if amrID in self.jobSequences:
                    del self.jobSequences[amrID]
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] 📝 Removed TransportCommand for AMR {amrID}"
                    )
                # Scheduler에게 AMR 상태 업데이트 알림
                self.state = self.stateList[1]  # UPDATE

            # 2️⃣ TO_DESTINATION phase 처리 (Job 완료 또는 WaitingArea 도착)
            elif transportPhase == "TO_DESTINATION":
                # WaitingArea와 Equipment 구분
                is_waiting_area = equipmentID and equipmentID.startswith(
                    'WAITING_AREA')

                if is_waiting_area:
                    # WaitingArea 도착 처리 (점유 관리 없음)
                    waitingArea = self.globalVar.getWaitingAreaInfoByID(
                        equipmentID)
                    if waitingArea:
                        pass

                    vehicleInfo = self.globalVar.getVehicleInfoByID(amrID)
                    if vehicleInfo:
                        vehicleInfo.setState("IDLE")

                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] 🅿️ AMR {amrID} arrived at WaitingArea {equipmentID} - Now IDLE"
                    )
                    # 대기 도착 시 해당 AMR 관련 보류 명령/이동 명령 정리
                    if self.pendingCommand and self.pendingCommand.get('amrID') == amrID:
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][FleetManagement] 🧹 Clearing pendingCommand for AMR {amrID} (arrived WAITING)"
                        )
                        self.pendingCommand = None
                    if self.watingARMCommand and self.watingARMCommand.get('amrID') == amrID:
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][FleetManagement] 🧹 Clearing watingARMCommand for AMR {amrID} (arrived WAITING)"
                        )
                        self.watingARMCommand = None
                    if amrID in self.jobSequences:
                        del self.jobSequences[amrID]
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][FleetManagement] 📝 Removed TransportCommand for AMR {amrID}"
                        )
                else:
                    # Equipment 도착 처리 (Job 완료)
                    equipmentInfo = self.globalVar.getEquipmentInfoByID(
                        equipmentID)
                    if equipmentInfo:
                        equipmentInfo.undockedAMRs.add(amrID)
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][FleetManagement] 📝 Stored undocked AMR {amrID} in Equipment {equipmentID}"
                        )
                    vehicleInfo = self.globalVar.getVehicleInfoByID(amrID)
                    if vehicleInfo:
                        vehicleInfo.setJobID(None)
                        vehicleInfo.setState("IDLE")

                    # TransportCommand 정리
                    if amrID in self.jobSequences:
                        del self.jobSequences[amrID]

                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] ✅ AMR {amrID} is now FREE - Job #{jobID} completed"
                    )

                # Scheduler에게 AMR 상태 업데이트 알림 (항상 전송)
                # UPDATE - IDLE AMR이 생겼으므로 다음 작업 할당 가능
                self.state = self.stateList[1]  # UPDATE

            # 3️⃣ TO_FROM phase 처리 (FROM Equipment 언도킹 → TO Equipment로 이동)
            elif transportPhase == "TO_FROM" and amrID in self.jobSequences:
                transportCommand = self.jobSequences[amrID]

                # TransportCommand에서 TO Equipment 정보 조회
                nextNodeID = transportCommand.getToNodeID()
                nextEquipmentID = nextNodeID.split('_')[0]  # "B-1_IN" -> "B-1"

                # AMR 현재 위치 (outputPort에서 출발)
                currentEquipmentInfo = self.globalVar.getEquipmentInfoByID(
                    equipmentID)
                nextEquipmentInfo = self.globalVar.getEquipmentInfoByID(
                    nextEquipmentID)

                if currentEquipmentInfo and nextEquipmentInfo:
                    currentPos = currentEquipmentInfo.outputPort.get(
                        'position')
                    nextPos = transportCommand.getToPosition()

                    # TO Equipment로 이동 명령 생성 (먼저 저장)
                    self.pendingCommand = {
                        'amrID': amrID,
                        'jobID': jobID,
                        'commandID': f"NEXT_{jobID}_{equipmentID}_{nextEquipmentID}",
                        'fromPosition': currentPos,
                        'toPosition': nextPos,
                        'fromNodeID': equipmentID + '_OUT',
                        'toNodeID': nextNodeID,
                        'action': 'TRANSPORT_NEXT',
                        'phase': 'TO_DESTINATION'  # TO Equipment로 이동
                    }

                    # TO Equipment에 언도킹된 AMR 확인
                    if nextEquipmentInfo.undockedAMRs:
                        # 대기 중인 AMR에게 "비켜라" 명령 생성
                        waitingAMR = list(nextEquipmentInfo.undockedAMRs)[
                            0]  # 첫 번째 AMR

                        # 대기 중인 AMR의 현재 위치 가져오기
                        waitingVehicleInfo = self.globalVar.getVehicleInfoByID(
                            waitingAMR)
                        if waitingVehicleInfo and waitingAMR in self.amrPositions:
                            waiting_pose = self.amrPositions[waitingAMR]
                            current_pos = [
                                waiting_pose['x'], waiting_pose['y']]
                        else:
                            # 위치 정보 없으면 Equipment outputPort 위치 사용
                            current_pos = [nextPos['x'], nextPos['y']]

                        # 가장 가까운 WaitingArea 찾기 (점유 상태 무시)
                        closestArea = self.globalVar.getClosestWaitingArea(
                            current_pos)

                        if closestArea:
                            target_x = closestArea.position['x']
                            target_y = closestArea.position['y']
                            areaID = closestArea.strAreaID

                            # WaitingArea 예약 로직 제거 - 여러 AMR이 같은 곳으로 갈 수 있음
                            self.globalVar.printTerminal(
                                f"[{self.getTime()}][FleetManagement] 🅿️ Directing AMR {waitingAMR} to closest WaitingArea {areaID}"
                            )
                        else:
                            # WaitingArea가 없음 - 현재 위치에서 10m 전진
                            self.globalVar.printTerminal(
                                f"[{self.getTime()}][FleetManagement] ⚠️ No WaitingArea found! Fallback to 10m forward."
                            )
                            import math
                            current_yaw = waiting_pose['yaw'] if waitingVehicleInfo and waitingAMR in self.amrPositions else 0
                            distance = 10.0
                            target_x = current_pos[0] + \
                                distance * math.cos(current_yaw)
                            target_y = current_pos[1] + \
                                distance * math.sin(current_yaw)
                            areaID = None

                        # Equipment undockedAMRs에서 제거 (WaitingArea든 10m 전진이든 항상 실행)
                        if waitingAMR in nextEquipmentInfo.undockedAMRs:
                            nextEquipmentInfo.undockedAMRs.remove(waitingAMR)
                            destination = f"WaitingArea {areaID}" if areaID else f"10m forward to ({target_x:.2f}, {target_y:.2f})"
                            self.globalVar.printTerminal(
                                f"[{self.getTime()}][FleetManagement] 🗑️ Removed AMR {waitingAMR} from Equipment {nextEquipmentID} undocked list (moving to {destination})"
                            )

                        self.watingARMCommand = {
                            'amrID': waitingAMR,
                            'action': 'GO_WAITING',
                            'x': target_x,
                            'y': target_y,
                            'areaID': areaID,
                        }

                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][FleetManagement] 📌 TO Equipment {nextEquipmentID}에 대기 중인 AMR: {nextEquipmentInfo.undockedAMRs}"
                        )
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][FleetManagement] 🚶 Asking AMR {waitingAMR} to move to ({target_x:.2f}, {target_y:.2f})"
                        )

                        # SEND_GO_COMMAND 상태로 전환 (먼저 대기 AMR에게 명령)
                        self.state = self.stateList[2]  # SEND_GO_COMMAND
                    else:
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][FleetManagement] ✅ TO Equipment {nextEquipmentID}에 대기 중인 AMR 없음"
                        )

                        # SEND_COMMAND 상태로 바로 전환
                        self.state = self.stateList[3]  # SEND_COMMAND

                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] 🚚 Next destination: {equipmentID}_OUT → {nextNodeID} for Job #{jobID}"
                    )
                else:
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] ⚠️ Equipment info not found: {equipmentID} or {nextEquipmentID}"
                    )

            # 4️⃣ 기타: TransportCommand가 없는 경우
            else:
                if transportPhase == "TO_FROM":
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] ⚠️ No stored TransportCommand found for AMR {amrID} (Phase: {transportPhase})"
                    )
                else:
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] ⚠️ Unknown transport phase: {transportPhase} for AMR {amrID}"
                    )

            return True

        else:
            print(
                f"ERROR at FleetManagement ExternalTransition: #{self.getStateValue('strID')}")
            print(f"inputPort: {strPort}")
            print(f"CurrentState: {self.state}")
            return False

    def funcOutput(self):
        if self.state == "UPDATE":
            # Scheduler에게 Fleet 정보 전달
            self.addOutputEvent("fleetInfo", self.amrPositions.copy())

            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] Sending fleet info to Scheduler (AMRs: {len(self.amrPositions)})"
            )

            return True

        elif self.state == "SEND_GO_COMMAND":
            # 대기 중인 AMR에게 "비켜라" 명령 전송
            if self.watingARMCommand:
                self.addOutputEvent("amrGoCommand", self.watingARMCommand)

                self.globalVar.printTerminal(
                    f"[{self.getTime()}][FleetManagement] 🚶 GO Command sent to waiting AMR {self.watingARMCommand['amrID']}: Move to ({self.watingARMCommand['x']}, {self.watingARMCommand['y']})"
                )

            return True

        elif self.state == "SEND_COMMAND":
            # 준비된 명령 전송
            if self.pendingCommand:
                self.addOutputEvent("amrCommand", self.pendingCommand)

                self.globalVar.printTerminal(
                    f"[{self.getTime()}][FleetManagement] ✅ Command sent to AMR {self.pendingCommand['amrID']} for Job #{self.pendingCommand['jobID']}"
                )
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][FleetManagement] Transport: {self.pendingCommand['fromNodeID']} → {self.pendingCommand['toNodeID']}"
                )

            return True

        else:
            return True

    def funcInternalTransition(self):
        if self.state == "UPDATE":
            # UPDATE 완료 후 WAIT로 복귀
            self.state = self.stateList[0]  # WAIT
            self.lastUpdateTime = self.getTime()
            return True

        elif self.state == "SEND_GO_COMMAND":
            # GO 명령 전송 완료 후 SEND_COMMAND로 전환 (pendingCommand 전송 준비)
            self.watingARMCommand = None
            self.state = self.stateList[3]  # SEND_COMMAND
            return True

        elif self.state == "SEND_COMMAND":
            # 명령 전송 완료 후 명령 초기화 및 WAIT로 복귀
            self.pendingCommand = None
            self.state = self.stateList[0]  # WAIT
            return True

        else:
            return True

    def funcTimeAdvance(self):
        if self.state == "WAIT":
            return float('inf')
        elif self.state == "UPDATE":
            return 0  # 즉시 출력
        elif self.state == "SEND_GO_COMMAND":
            return 0  # 즉시 출력
        elif self.state == "SEND_COMMAND":
            return 0  # 즉시 출력
        else:
            return float('inf')
