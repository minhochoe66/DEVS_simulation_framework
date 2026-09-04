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
        self.addOutputPort("jobAssign")   # 작업 배정 (외부 출력)
        self.addOutputPort("taskAssign")  # FleetManagement로 보내는 작업 할당

        # Variables
        self.lstCompleteJob = []  # 다음 공정을 기다리는 완료 작업
        self.amrPositions = {}    # FleetManagement가 보낸 AMR 위치

        # 운반 명령 관리자
        self.commandManager = TransportCommandManager(globalVar)
        self.count = 0

        self.numStages = globalVar.objConfiguration.getConfiguration(
            "numStages") if globalVar.objConfiguration else 3

    def funcExternalTransition(self, strPort, objEvent):
        if strPort == "informDone":
            # objEvent: [equipmentID, jobID]
            equipmentID = objEvent[0]
            jobID = objEvent[1]

            jobInfo = self.globalVar.getTargetJobsByID(jobID)
            equipmentInfo = self.globalVar.getEquipmentInfoByID(equipmentID)

            # 다음 공정 결정
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

            # SOURCE는 Data_generator가 관리하므로 무시한다
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
            self.amrPositions = objEvent  # objEvent는 dictionary

            self.globalVar.printTerminal(
                f"[{self.getTime()}][Scheduler] Received fleet info with {len(self.amrPositions)} AMRs"
            )

            # AMR 상태가 바뀔 때마다 할당을 시도한다
            # 대기 중인 완료 작업이 있으면 다시 배정한다
            if self.lstCompleteJob:
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Scheduler] Re-attempting assignment for {len(self.lstCompleteJob)} queued jobs"
                )

            # WAIT일 때만 COMMAND로 전이한다
            if self.state == "WAIT":
                self.state = self.stateList[1]  # COMMAND
            # 이미 COMMAND면 현재 처리가 끝난 뒤 다시 검사된다

            return True

        else:
            print(
                f"ERROR at Scheduler ExternalTransition: #{self.getStateValue('strID')}")
            print(f"inputPort: {strPort}")
            print(f"CurrentState: {self.state}")
            return False

    def funcOutput(self):
        if self.state == "COMMAND":
            # 작업, 장비, AMR을 매칭한다
            transportCommand = self.checkCommandCondition()

            if transportCommand is not None:
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Scheduler] ✅ {transportCommand}"
                )

                # 운반 명령을 FleetManagement로 보낸다
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
        """numStages에 맞추어 다음 공정 스테이지를 정한다."""
        currentProcess = jobInfo.strCurrentProcess

        # numStages=1: SOURCE -> STAGE_B -> SINK
        # numStages=2: SOURCE -> STAGE_B -> STAGE_C -> SINK
        # numStages=3: SOURCE -> STAGE_B -> STAGE_C -> STAGE_D -> SINK

        if currentProcess == "SOURCE":
            # A 다음은 항상 B
            jobInfo.setNextProcess("STAGE_B", None)

        elif currentProcess and currentProcess.startswith("PROCESS_B"):
            # B 다음은 numStages에 따라 갈린다
            if self.numStages == 1:
                jobInfo.setNextProcess("SINK", None)
            else:
                jobInfo.setNextProcess("STAGE_C", None)

        elif currentProcess and currentProcess.startswith("PROCESS_C"):
            # C 다음도 numStages에 따라 갈린다
            if self.numStages == 2:
                jobInfo.setNextProcess("SINK", None)
            else:
                jobInfo.setNextProcess("STAGE_D", None)

        elif currentProcess and currentProcess.startswith("PROCESS_D"):
            # D 다음은 SINK
            jobInfo.setNextProcess("SINK", None)

        else:
            # SINK이거나 알 수 없는 공정
            jobInfo.setNextProcess(None, None)

        self.globalVar.printTerminal(
            f"[{self.getTime()}][Scheduler] Next process for Job #{jobInfo.intJobID}: {currentProcess} → {jobInfo.strNextProcess} (numStages={self.numStages})"
        )

    def checkCommandCondition(self):
        """대기 작업을 빈 장비와 유휴 AMR에 매칭하고 운반 명령을 만든다."""
        equipmentInfo = self.globalVar.getEquipmentInfo()
        vehicleInfo = self.globalVar.getVehicleInfo()

        # 1. 이미 완료되어 다음 공정을 기다리는 작업을 먼저 배정한다
        for jobID in self.lstCompleteJob[:]:  # 순회 중 원본이 바뀌므로 복사본을 쓴다
            jobInfo = self.globalVar.getTargetJobsByID(jobID)

            # 작업이 놓여 있는 장비가 픽업 위치가 된다
            fromEquipmentID = jobInfo.strCurrentProcessEqpID
            if fromEquipmentID is None:
                continue
            fromEquipment = self.globalVar.getEquipmentInfoByID(
                fromEquipmentID)

            # 다음 공정 장비 중 EMPTY인 것이 목적지가 된다
            for equipmentID, equipmentValue in equipmentInfo.items():
                # 유연 흐름 공정: nextProcess가 STAGE_X면 같은 스테이지의 아무 장비나 쓸 수 있다
                if jobInfo.strNextProcess and jobInfo.strNextProcess.startswith("STAGE_"):
                    type_match = (equipmentValue.strStageID ==
                                  jobInfo.strNextProcess)
                else:
                    # SOURCE와 SINK는 정확히 일치해야 한다
                    type_match = (equipmentValue.strType ==
                                  jobInfo.strNextProcess)

                if (type_match and equipmentValue.strState == "EMPTY"):

                    # 픽업 위치에 가장 가까운 유휴 AMR을 고른다
                    selectedAMR = self.findAvailableAMR(
                        fromEquipment, vehicleInfo)

                    if selectedAMR is not None:
                        # 운반 명령 생성
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

                        # AMR 상태 갱신
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

        # 2. 그 다음으로 DONE 상태 장비의 작업을 처리한다
        for equipmentID, equipmentValue in equipmentInfo.items():
            if equipmentValue.strState == "DONE" and equipmentValue.intProcessingJobID:
                jobID = equipmentValue.intProcessingJobID
                jobInfo = self.globalVar.getTargetJobsByID(jobID)

                # 다음 공정 장비 중 EMPTY인 것이 목적지가 된다
                for nextEquipmentID, nextEquipmentValue in equipmentInfo.items():
                    # 유연 흐름 공정: nextProcess가 STAGE_X면 같은 스테이지의 아무 장비나 쓸 수 있다
                    if jobInfo.strNextProcess and jobInfo.strNextProcess.startswith("STAGE_"):
                        type_match = (
                            nextEquipmentValue.strStageID == jobInfo.strNextProcess)
                    else:
                        # SOURCE와 SINK는 정확히 일치해야 한다
                        type_match = (nextEquipmentValue.strType ==
                                      jobInfo.strNextProcess)

                    if (type_match and nextEquipmentValue.strState == "EMPTY"):

                        # 픽업 위치에 가장 가까운 유휴 AMR을 고른다
                        selectedAMR = self.findAvailableAMR(
                            equipmentValue, vehicleInfo)

                        if selectedAMR is not None:
                            # 운반 명령 생성
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

                            # AMR 상태 갱신
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
        """픽업 위치에 가장 가까운 유휴 AMR을 고른다.

        targetEquipment는 AMR이 먼저 가야 할 출발지 장비다.
        """
        import math

        # 출발지 장비의 입력 포트가 픽업 지점이다
        if targetEquipment.inputPort:
            targetPos = targetEquipment.inputPort['position']
        else:
            targetPos = targetEquipment.workPosition['position']

        targetX = targetPos['x']
        targetY = targetPos['y']

        minDistance = float('inf')
        selectedAMR = None

        for amrID, amrValue in vehicleInfo.items():
            if amrValue.strState == "IDLE" and hasattr(amrValue, 'lstCoordinates'):
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
