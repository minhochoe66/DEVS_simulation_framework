from SimulationEngine.ClassicDEVS.DEVSAtomicModel import DEVSAtomicModel


class FleetManagement(DEVSAtomicModel):
    def __init__(self, strID, globalVar):
        super().__init__(strID)
        self.globalVar = globalVar

        # States: WAIT, UPDATE (플릿 정보 전달), SEND_GO_COMMAND (대기 AMR 이동), SEND_COMMAND (작업 지시)
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
        # AMR별 운반 명령 {amrID: transportCommand}
        self.jobSequences = {}
        # 다음 출력에서 보낼 명령
        self.pendingCommand = None
        self.watingARMCommand = None

    def funcExternalTransition(self, strPort, objEvent):
        if strPort == "amrPosition":
            # AMR 위치 갱신
            amrID = objEvent.strID
            self.amrPositions[amrID] = {
                'x': objEvent.x,
                'y': objEvent.y,
                'yaw': objEvent.yaw,
                'lin_vel': objEvent.lin_vel,
                'ang_vel': objEvent.ang_vel,
                'timestamp': self.getTime()
            }

            # GlobalVar의 차량 정보도 함께 갱신
            vehicleInfo = self.globalVar.getVehicleInfoByID(amrID)
            if vehicleInfo:
                # 도킹 중에는 Equipment가 설정한 좌표를 유지한다
                keep_equipment_coords = False
                if hasattr(vehicleInfo, 'getEquipmentID') and vehicleInfo.getEquipmentID():
                    keep_equipment_coords = True

                if not keep_equipment_coords:
                    vehicleInfo.setCoordinates([objEvent.x, objEvent.y])

                # 정지해 있고 작업이 없으면 IDLE로 되돌린다
                if objEvent.lin_vel < 0.1 and vehicleInfo.intJobID is None:
                    if vehicleInfo.strState != "IDLE":
                        vehicleInfo.setState("IDLE")
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][FleetManagement] AMR {amrID} is now IDLE"
                        )

            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] AMR {amrID} position updated: ({objEvent.x:.2f}, {objEvent.y:.2f}), State: {vehicleInfo.strState if vehicleInfo else 'Unknown'}"
            )

            # Scheduler에 알리기 위해 UPDATE로 전이한다
            # WAIT일 때만 전이한다. 명령 처리 중이면 방해하지 않는다
            if self.state == "WAIT":
                self.state = self.stateList[1]  # UPDATE

            return True

        elif strPort == "taskAssign":
            # Scheduler가 보낸 운반 명령
            transportCommand = objEvent
            amrID = transportCommand.assignedAMR
            jobID = transportCommand.jobID

            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] Received TransportCommand: {transportCommand}"
            )

            # 새 작업을 받은 AMR은 모든 장비의 undockedAMRs에서 뺀다
            for equipmentInfo in self.globalVar.getEquipmentInfo().values():
                if amrID in equipmentInfo.undockedAMRs:
                    equipmentInfo.undockedAMRs.remove(amrID)
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] 🗑️ Removed AMR {amrID} from Equipment {equipmentInfo.strEquipmentID} undocked list"
                    )

            # AMR별로 운반 명령을 보관한다
            self.jobSequences[amrID] = transportCommand
            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] 📋 Stored TransportCommand for AMR {amrID}: {transportCommand.commandID}"
            )

            # 픽업 장비로 보내는 이동 명령을 만든다
            self.pendingCommand = {
                'amrID': amrID,
                'jobID': jobID,
                'commandID': transportCommand.commandID,
                'fromPosition': transportCommand.getFromPosition(),
                'toPosition': transportCommand.getToPosition(),
                'fromNodeID': transportCommand.getFromNodeID(),
                'toNodeID': transportCommand.getToNodeID(),
                'action': 'TRANSPORT',  # FROM -> TO 운반
                'phase': 'TO_FROM'  # 먼저 FROM 장비로
            }

            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] 📥 Command prepared for AMR {amrID}: {transportCommand.getFromNodeID()} → {transportCommand.getToNodeID()}"
            )

            # 즉시 보내기 위해 SEND_COMMAND로 전이
            self.state = self.stateList[3]  # SEND_COMMAND

            return True

        elif strPort == "undockingComplete_I":
            # Local_Planner가 보낸 언도킹 완료
            amrID = objEvent[0]
            equipmentID = objEvent[1]
            jobID = objEvent[2]
            transportPhase = objEvent[3]

            vehicleInfo = self.globalVar.getVehicleInfoByID(amrID)
            current_coords = vehicleInfo.getCoordinates() if vehicleInfo else "Unknown"

            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] 🚪 UNDOCKING complete: AMR {amrID} from {equipmentID}, Phase: {transportPhase}"
            )

            # 여기서부터 운반 phase에 따라 분기한다

            # WAITING: 대기 구역에 도착한 경우
            if transportPhase == "WAITING":
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
                # 대기 구역에 도착했으면 그 AMR의 보류 명령을 정리한다
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
                # Scheduler에 상태 변화를 알린다
                self.state = self.stateList[1]  # UPDATE

            # TO_DESTINATION: 작업 완료이거나 대기 구역 도착
            elif transportPhase == "TO_DESTINATION":
                # 목적지가 대기 구역인지 장비인지 구분한다
                is_waiting_area = equipmentID and equipmentID.startswith(
                    'WAITING_AREA')

                if is_waiting_area:
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
                    # 대기 구역에 도착했으면 그 AMR의 보류 명령을 정리한다
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
                    # 장비 도착: 작업 완료
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

                    # 운반 명령 정리
                    if amrID in self.jobSequences:
                        del self.jobSequences[amrID]

                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] ✅ AMR {amrID} is now FREE - Job #{jobID} completed"
                    )

                # 유휴 AMR이 생겼으므로 Scheduler에 알려 다음 작업을 받게 한다
                self.state = self.stateList[1]  # UPDATE

            # TO_FROM: 픽업 장비에서 언도킹했으니 목적지 장비로 보낸다
            elif transportPhase == "TO_FROM" and amrID in self.jobSequences:
                transportCommand = self.jobSequences[amrID]

                # 운반 명령에서 목적지 장비를 읽는다
                nextNodeID = transportCommand.getToNodeID()
                nextEquipmentID = nextNodeID.split('_')[0]  # "B-1_IN" -> "B-1"

                # AMR은 출력 포트에서 출발한다
                currentEquipmentInfo = self.globalVar.getEquipmentInfoByID(
                    equipmentID)
                nextEquipmentInfo = self.globalVar.getEquipmentInfoByID(
                    nextEquipmentID)

                if currentEquipmentInfo and nextEquipmentInfo:
                    currentPos = currentEquipmentInfo.outputPort.get(
                        'position')
                    nextPos = transportCommand.getToPosition()

                    # 목적지 장비로 가는 이동 명령을 만들어 보류해 둔다
                    self.pendingCommand = {
                        'amrID': amrID,
                        'jobID': jobID,
                        'commandID': f"NEXT_{jobID}_{equipmentID}_{nextEquipmentID}",
                        'fromPosition': currentPos,
                        'toPosition': nextPos,
                        'fromNodeID': equipmentID + '_OUT',
                        'toNodeID': nextNodeID,
                        'action': 'TRANSPORT_NEXT',
                        'phase': 'TO_DESTINATION'  # 목적지 장비로
                    }

                    # 목적지 장비 앞에 언도킹한 채 서 있는 AMR이 있는지 본다
                    if nextEquipmentInfo.undockedAMRs:
                        # 서 있는 AMR에게 비켜 달라는 이동 명령을 만든다
                        waitingAMR = list(nextEquipmentInfo.undockedAMRs)[
                            0]  # 첫 번째 AMR

                        # 비켜야 할 AMR의 현재 위치
                        waitingVehicleInfo = self.globalVar.getVehicleInfoByID(
                            waitingAMR)
                        if waitingVehicleInfo and waitingAMR in self.amrPositions:
                            waiting_pose = self.amrPositions[waitingAMR]
                            current_pos = [
                                waiting_pose['x'], waiting_pose['y']]
                        else:
                            # 위치를 모르면 장비 출력 포트를 쓴다
                            current_pos = [nextPos['x'], nextPos['y']]

                        # 가장 가까운 대기 구역으로 보낸다. 대기 구역은 배타적이지 않다
                        closestArea = self.globalVar.getClosestWaitingArea(
                            current_pos)

                        if closestArea:
                            target_x = closestArea.position['x']
                            target_y = closestArea.position['y']
                            areaID = closestArea.strAreaID

                            self.globalVar.printTerminal(
                                f"[{self.getTime()}][FleetManagement] 🅿️ Directing AMR {waitingAMR} to closest WaitingArea {areaID}"
                            )
                        else:
                            # 대기 구역이 없으면 현재 위치에서 10 m 전진시킨다
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

                        # 두 경우 모두 장비의 undockedAMRs에서 제거한다
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

                        # 비켜 달라는 명령을 먼저 보낸다
                        self.state = self.stateList[2]  # SEND_GO_COMMAND
                    else:
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][FleetManagement] ✅ TO Equipment {nextEquipmentID}에 대기 중인 AMR 없음"
                        )

                        # 막는 AMR이 없으면 바로 작업 지시를 보낸다
                        self.state = self.stateList[3]  # SEND_COMMAND

                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] 🚚 Next destination: {equipmentID}_OUT → {nextNodeID} for Job #{jobID}"
                    )
                else:
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] ⚠️ Equipment info not found: {equipmentID} or {nextEquipmentID}"
                    )

            # 그 밖의 경우: 이 AMR에 걸린 운반 명령이 없다
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
            # 플릿 정보를 Scheduler로 보낸다
            self.addOutputEvent("fleetInfo", self.amrPositions.copy())

            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] Sending fleet info to Scheduler (AMRs: {len(self.amrPositions)})"
            )

            return True

        elif self.state == "SEND_GO_COMMAND":
            # 비켜 달라는 이동 명령을 보낸다
            if self.watingARMCommand:
                self.addOutputEvent("amrGoCommand", self.watingARMCommand)

                self.globalVar.printTerminal(
                    f"[{self.getTime()}][FleetManagement] 🚶 GO Command sent to waiting AMR {self.watingARMCommand['amrID']}: Move to ({self.watingARMCommand['x']}, {self.watingARMCommand['y']})"
                )

            return True

        elif self.state == "SEND_COMMAND":
            # 보류해 둔 명령을 보낸다
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
            # UPDATE를 마치면 WAIT으로 돌아간다
            self.state = self.stateList[0]  # WAIT
            self.lastUpdateTime = self.getTime()
            return True

        elif self.state == "SEND_GO_COMMAND":
            # 비켜라 명령을 보냈으면 이어서 보류 명령을 보낸다
            self.watingARMCommand = None
            self.state = self.stateList[3]  # SEND_COMMAND
            return True

        elif self.state == "SEND_COMMAND":
            # 명령을 보냈으면 비우고 WAIT으로 돌아간다
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
