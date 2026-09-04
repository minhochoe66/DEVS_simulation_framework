from SimulationEngine.ClassicDEVS.DEVSAtomicModel import DEVSAtomicModel
from modeling.simulation.SystemController.TransportCommand import TransportCommandManager


class Scheduler(DEVSAtomicModel):
    def __init__(self, strID, globalVar):
        super().__init__(strID)
        self.globalVar = globalVar

        # States
        self.stateList = ["WAIT", "COMMAND", "COMPLETE"]
        self.state = self.stateList[0]

        # State Variables
        self.addStateVariable('strID', strID)

        # Input Ports
        self.addInputPort("informDone")   # Equipment에서 작업 완료
        self.addInputPort("informFree")   # Equipment 준비 완료
        self.addInputPort("fleetInfo")    # FleetManagement로부터 AMR 정보

        # Output Ports
        self.addOutputPort("jobAssign")   # 작업 배정 (외부용)
        self.addOutputPort("taskAssign")  # FleetManagement에게 작업 할당 정보 전달

        # Variables
        self.lstCompleteJob = []  # 완료된 작업 리스트 (다음 공정 대기)
        self.amrPositions = {}    # FleetManagement로부터 받은 AMR 위치 정보

        # TransportCommand 관리자
        self.commandManager = TransportCommandManager(globalVar)
        self.count = 0

        # numStages 설정 가져오기
        self.numStages = globalVar.objConfiguration.getConfiguration(
            "numStages") if globalVar.objConfiguration else 3

    def funcExternalTransition(self, strPort, objEvent):
        if strPort == "informDone":
            # objEvent: [equipmentID, jobID]
            equipmentID = objEvent[0]
            jobID = objEvent[1]

            jobInfo = self.globalVar.getTargetJobsByID(jobID)
            equipmentInfo = self.globalVar.getEquipmentInfoByID(equipmentID)

            # 다음 공정 설정
            self.setNextProcess(jobInfo)
            self.lstCompleteJob.append(jobID)

            self.globalVar.printTerminal(
                f"[{self.getTime()}][Scheduler] Job #{jobID} done at {equipmentID}, next: {jobInfo.strNextProcess}"
            )

            if self.state == "WAIT":
                self.state = self.stateList[1]  # COMMAND
            else:
                self.continueTimeAdvance()

            return True

        elif strPort == "informFree":
            # objEvent: equipmentID
            equipmentID = objEvent
            equipmentInfo = self.globalVar.getEquipmentInfoByID(equipmentID)

            # SOURCE 장비는 무시 (Data_generator가 관리)
            if equipmentInfo.strType != "SOURCE":
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Scheduler] Equipment {equipmentID} is FREE"
                )
                if self.state == "WAIT":
                    self.state = self.stateList[1]  # COMMAND

            if equipmentInfo.strType == "SINK":
                # 최종 완료
                self.count += 1
                if self.count == self.globalVar.objConfiguration.getConfiguration("numJob"):
                    self.state = self.stateList[2]  # COMPLETE
                else:
                    if self.state == "WAIT":
                        self.state = self.stateList[1]  # COMMAND

        elif strPort == "fleetInfo":
            # FleetManagement로부터 AMR 정보 수신
            self.amrPositions = objEvent  # objEvent는 dictionary

            self.globalVar.printTerminal(
                f"[{self.getTime()}][Scheduler] Received fleet info with {len(self.amrPositions)} AMRs"
            )

            # AMR 상태 업데이트 시 항상 작업 할당 시도
            # lstCompleteJob에 대기 중인 작업이 있으면 재할당 시도
            if self.lstCompleteJob:
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Scheduler] Re-attempting assignment for {len(self.lstCompleteJob)} queued jobs"
                )

            # WAIT 상태일 때만 COMMAND로 전환
            if self.state == "WAIT":
                self.state = self.stateList[1]  # COMMAND
            # 이미 COMMAND 상태면 현재 처리 완료 후 다시 검사됨

            return True

        else:
            print(
                f"ERROR at Scheduler ExternalTransition: #{self.getStateValue('strID')}")
            print(f"inputPort: {strPort}")
            print(f"CurrentState: {self.state}")
            return False

    def funcOutput(self):
        if self.state == "COMMAND":
            # 작업 배정 조건 체크 (AMR 포함)
            transportCommand = self.checkCommandCondition()

            if transportCommand is not None:
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Scheduler] ✅ {transportCommand}"
                )

                # FleetManagement에게 TransportCommand 전달
                self.addOutputEvent("taskAssign", transportCommand)

                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Scheduler] TransportCommand sent: {transportCommand.commandID}"
                )

            return True
        elif self.state == "COMPLETE":
            self.addOutputEvent("Complete_job_O", True)
            return True
        else:
            print(f"ERROR at Scheduler Output: #{self.getStateValue('strID')}")
            print(f"CurrentState: {self.state}")
            return False

    def funcInternalTransition(self):
        if self.state == "COMMAND":
            self.state = self.stateList[0]  # WAIT
            return True
        elif self.state == "COMPLETE":
            self.state = self.stateList[0]  # WAIT
            return True
        else:
            print(
                f"ERROR at Scheduler InternalTransition: #{self.getStateValue('strID')}")
            print(f"CurrentState: {self.state}")
            return False

    def funcTimeAdvance(self):
        if self.state == "WAIT":
            return float('inf')
        else:
            return 0

    def setNextProcess(self, jobInfo):
        """다음 공정 스테이지 설정 (numStages에 따라 동적 흐름)"""
        currentProcess = jobInfo.strCurrentProcess

        # numStages에 따른 동적 공정 흐름
        # numStages=1: SOURCE → STAGE_B → SINK
        # numStages=2: SOURCE → STAGE_B → STAGE_C → SINK
        # numStages=3: SOURCE → STAGE_B → STAGE_C → STAGE_D → SINK

        if currentProcess == "SOURCE":
            # A 완료 → 항상 B로
            jobInfo.setNextProcess("STAGE_B", None)

        elif currentProcess and currentProcess.startswith("PROCESS_B"):
            # B 완료 → numStages에 따라 분기
            if self.numStages == 1:
                jobInfo.setNextProcess("SINK", None)
            else:
                jobInfo.setNextProcess("STAGE_C", None)

        elif currentProcess and currentProcess.startswith("PROCESS_C"):
            # C 완료 → numStages에 따라 분기
            if self.numStages == 2:
                jobInfo.setNextProcess("SINK", None)
            else:
                jobInfo.setNextProcess("STAGE_D", None)

        elif currentProcess and currentProcess.startswith("PROCESS_D"):
            # D 완료 → SINK
            jobInfo.setNextProcess("SINK", None)

        else:
            # SINK 또는 알 수 없는 상태
            jobInfo.setNextProcess(None, None)

        self.globalVar.printTerminal(
            f"[{self.getTime()}][Scheduler] Next process for Job #{jobInfo.intJobID}: {currentProcess} → {jobInfo.strNextProcess} (numStages={self.numStages})"
        )

    def checkCommandCondition(self):
        """작업과 장비 매칭 + AMR 할당 + TransportCommand 생성"""
        equipmentInfo = self.globalVar.getEquipmentInfo()
        vehicleInfo = self.globalVar.getVehicleInfo()

        # 1. 완료된 작업 중 배정 가능한 것 찾기 (높은 우선순위)
        for jobID in self.lstCompleteJob[:]:  # 복사본으로 순회
            jobInfo = self.globalVar.getTargetJobsByID(jobID)

            # 현재 작업이 있는 장비 찾기 (fromEquipment)
            fromEquipmentID = jobInfo.strCurrentProcessEqpID
            if fromEquipmentID is None:
                continue
            fromEquipment = self.globalVar.getEquipmentInfoByID(
                fromEquipmentID)

            # 다음 공정 장비 중 EMPTY인 것 찾기 (toEquipment)
            for equipmentID, equipmentValue in equipmentInfo.items():
                # 플렉서블 플로우샵: nextProcess가 STAGE_X 형식이면 스테이지로 매칭
                if jobInfo.strNextProcess and jobInfo.strNextProcess.startswith("STAGE_"):
                    type_match = (equipmentValue.strStageID ==
                                  jobInfo.strNextProcess)
                else:
                    # SOURCE, SINK 등은 정확히 매칭
                    type_match = (equipmentValue.strType ==
                                  jobInfo.strNextProcess)

                if (type_match and equipmentValue.strState == "EMPTY"):

                    # IDLE 상태인 AMR 찾기 (fromEquipment와 가장 가까운 AMR)
                    selectedAMR = self.findAvailableAMR(
                        fromEquipment, vehicleInfo)

                    if selectedAMR is not None:
                        # TransportCommand 생성
                        transportCommand = self.commandManager.createCommand(
                            jobID, fromEquipment, equipmentValue
                        )

                        # AMR 할당
                        transportCommand.assignAMR(selectedAMR, self.getTime())
                        transportCommand.createTime = self.getTime()

                        # 작업 배정
                        jobInfo.setNextProcess(
                            equipmentValue.strType, equipmentID)
                        self.lstCompleteJob.remove(jobID)
                        equipmentValue.setEquipmentState("RESERVED")

                        # AMR 상태 업데이트
                        vehicleInfo[selectedAMR].setState("RESERVED")
                        vehicleInfo[selectedAMR].setJobID(jobID)

                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][Scheduler] AMR {selectedAMR} assigned to Job #{jobID}"
                        )
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][Scheduler] From: {transportCommand.getFromNodeID()} → To: {transportCommand.getToNodeID()}"
                        )

                        return transportCommand
                    else:
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][Scheduler] No available AMR for Job #{jobID} → Equipment {equipmentID}"
                        )

        # 2. DONE 상태 장비의 작업 처리 (낮은 우선순위)
        for equipmentID, equipmentValue in equipmentInfo.items():
            if equipmentValue.strState == "DONE" and equipmentValue.intProcessingJobID:
                jobID = equipmentValue.intProcessingJobID
                jobInfo = self.globalVar.getTargetJobsByID(jobID)

                # 다음 공정 장비 중 EMPTY인 것 찾기 (toEquipment)
                for nextEquipmentID, nextEquipmentValue in equipmentInfo.items():
                    # 플렉서블 플로우샵: nextProcess가 STAGE_X 형식이면 스테이지로 매칭
                    if jobInfo.strNextProcess and jobInfo.strNextProcess.startswith("STAGE_"):
                        type_match = (
                            nextEquipmentValue.strStageID == jobInfo.strNextProcess)
                    else:
                        # SOURCE, SINK 등은 정확히 매칭
                        type_match = (nextEquipmentValue.strType ==
                                      jobInfo.strNextProcess)

                    if (type_match and nextEquipmentValue.strState == "EMPTY"):

                        # IDLE 상태인 AMR 찾기 (equipmentValue=fromEquipment와 가장 가까운 AMR)
                        selectedAMR = self.findAvailableAMR(
                            equipmentValue, vehicleInfo)

                        if selectedAMR is not None:
                            # TransportCommand 생성
                            transportCommand = self.commandManager.createCommand(
                                jobID, equipmentValue, nextEquipmentValue
                            )

                            # AMR 할당
                            transportCommand.assignAMR(
                                selectedAMR, self.getTime())
                            transportCommand.createTime = self.getTime()

                            # 작업 배정
                            jobInfo.setNextProcess(
                                nextEquipmentValue.strType, nextEquipmentID)
                            nextEquipmentValue.setEquipmentState("RESERVED")

                            # AMR 상태 업데이트
                            vehicleInfo[selectedAMR].setState("RESERVED")
                            vehicleInfo[selectedAMR].setJobID(jobID)

                            self.globalVar.printTerminal(
                                f"[{self.getTime()}][Scheduler] AMR {selectedAMR} assigned to Job #{jobID} (from DONE equipment - LOW PRIORITY)"
                            )
                            self.globalVar.printTerminal(
                                f"[{self.getTime()}][Scheduler] From: {transportCommand.getFromNodeID()} → To: {transportCommand.getToNodeID()}"
                            )

                            return transportCommand
                        else:
                            self.globalVar.printTerminal(
                                f"[{self.getTime()}][Scheduler] No available AMR for Job #{jobID} → Equipment {nextEquipmentID}"
                            )

        return None

    def findAvailableAMR(self, targetEquipment, vehicleInfo):
        """IDLE 상태이고 출발지(fromEquipment)와 가장 가까운 AMR 선택

        Args:
            targetEquipment: fromEquipment (출발지 장비) - AMR이 먼저 가야 할 곳
            vehicleInfo: AMR 정보 딕셔너리
        """
        import math

        # 출발지 장비의 입력 포트 위치 (AMR이 픽업하러 가야 할 곳)
        if targetEquipment.inputPort:
            targetPos = targetEquipment.inputPort['position']
        else:
            targetPos = targetEquipment.workPosition['position']

        targetX = targetPos['x']
        targetY = targetPos['y']

        minDistance = float('inf')
        selectedAMR = None

        # IDLE 상태인 AMR 찾기
        for amrID, amrValue in vehicleInfo.items():
            if amrValue.strState == "IDLE" and hasattr(amrValue, 'lstCoordinates'):
                # AMR과 장비 사이의 거리 계산
                amrX, amrY = amrValue.lstCoordinates
                distance = math.sqrt((targetX - amrX)**2 + (targetY - amrY)**2)

                if distance < minDistance:
                    minDistance = distance
                    selectedAMR = amrID

        if selectedAMR:
            self.globalVar.printTerminal(
                f"[{self.getTime()}][Scheduler] Selected AMR {selectedAMR} (distance to fromEquipment: {minDistance:.2f}m)"
            )

        return selectedAMR
