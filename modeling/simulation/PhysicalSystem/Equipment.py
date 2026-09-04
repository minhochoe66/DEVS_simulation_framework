from SimulationEngine.ClassicDEVS.DEVSAtomicModel import DEVSAtomicModel
import numpy as np


class Equipment(DEVSAtomicModel):

    def __init__(self, strID, globalVar, equipmentInfo):
        super().__init__(strID)

        # Global Variables
        self.globalVar = globalVar
        self.equipmentInfo = equipmentInfo

        # States
        self.stateList = ["EMPTY", "LOAD", "BUSY",
                          "DONE", "UNLOAD", "INFORM", "DOCKING", "UNDOCKING"]
        self.state = self.stateList[0]

        # Input Ports
        self.addInputPort("job")
        self.addInputPort("amrDocking")  # Maneuver용 도킹 신호 (무시)
        self.addInputPort("EquipmentDocking")  # Equipment용 도킹 신호

        # Output Ports
        self.addOutputPort("informDone")
        self.addOutputPort("informFree")
        self.addOutputPort("jobExchange")  # AMR과 작업 교환

        # State Variables
        self.addStateVariable("strID", strID)
        self.addStateVariable("processType", equipmentInfo.strType)
        self.addStateVariable("stageID", equipmentInfo.strStageID)
        self.addStateVariable("processTime", equipmentInfo.dblProcessTime)
        self.addStateVariable("exchangeTime", equipmentInfo.dblExchangeTime)

        # Variables
        self.currentJob = None
        self.reservedJob = None  # 예약된 작업 (job 이벤트로 받음)
        self.currentAMR = None   # 현재 도킹한 AMR ID
        self.dockingPhase = None  # "TO_FROM" or "TO_DESTINATION"

    def funcExternalTransition(self, strPort, objEvent):
        if strPort == "job":
            # objEvent: [equipmentID, jobID]
            if self.getStateValue("strID") == objEvent[0]:
                jobID = objEvent[1]
                equipmentInfo = self.globalVar.getEquipmentInfoByID(
                    objEvent[0])

                if equipmentInfo.strType == "SOURCE":
                    # SOURCE: 바로 처리 시작 (AMR 없이 작업 생성)
                    jobInfo = self.globalVar.getTargetJobsByID(jobID)
                    jobInfo.setTime('start', objEvent[0], self.getTime())
                    jobInfo.setCurrentProcess(
                        equipmentInfo.strType, equipmentInfo.strEquipmentID)

                    equipmentInfo.setProcessingJobID(jobID)
                    equipmentInfo.setEquipmentState("BUSY")
                    self.currentJob = jobID
                    self.state = self.stateList[2]  # BUSY

                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] Job #{jobID} started (SOURCE)"
                    )

                elif self.state == "EMPTY":
                    # 일반 장비: 작업 예약 (AMR 대기)
                    self.reservedJob = jobID
                    equipmentInfo.setEquipmentState("RESERVED")

                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] Job #{jobID} reserved (waiting for AMR)"
                    )
                else:
                    self.continueTimeAdvance()
            else:
                self.continueTimeAdvance()
            return True

        elif strPort == "EquipmentDocking":
            # objEvent: [amrID, equipmentID, phase]
            if self.getStateValue("strID") == objEvent[1]:
                old_amr = self.currentAMR
                self.currentAMR = objEvent[0]
                self.equipment_id = objEvent[1]
                self.dockingPhase = objEvent[2]
                self.state = "DOCKING"

                print(
                    f"🚪 [EQUIPMENT_AMR] {self.getStateValue('strID')}: currentAMR changed from {old_amr} → {self.currentAMR}")
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] ✅ AMR {self.currentAMR} docking (phase: {self.dockingPhase})"
                )
            else:
                self.continueTimeAdvance()
            return True

    def funcOutput(self):
        if self.state == "LOAD":
            # AMR → Equipment: 작업 전달
            equipmentInfo = self.globalVar.getEquipmentInfoByID(
                self.getStateValue("strID"))

            # TO_DESTINATION: AMR이 가져온 작업 ID 사용
            # TO_FROM: 예약된 작업 ID 사용
            if self.dockingPhase == "TO_DESTINATION":
                # AMR의 작업 ID 가져오기
                amr_job_id = self.globalVar.getVehicleInfoByID(
                    self.currentAMR).intJobID
                if amr_job_id is None:
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] Warning: AMR {self.currentAMR} has no job ID"
                    )
                    return
                job_id = amr_job_id
            else:
                # TO_FROM: 예약된 작업 사용
                if self.reservedJob is None:
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] Warning: No reserved job for TO_FROM phase"
                    )
                    return
                job_id = self.reservedJob

            jobInfo = self.globalVar.getTargetJobsByID(job_id)

            jobInfo.setTime('start', self.getStateValue(
                "strID"), self.getTime())
            jobInfo.setCurrentProcess(
                equipmentInfo.strType, equipmentInfo.strEquipmentID)

            self.currentJob = job_id
            if self.dockingPhase == "TO_FROM":
                self.reservedJob = None

            self.globalVar.printTerminal(
                f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] Job #{self.currentJob} loaded from AMR {self.currentAMR}"
            )
            self.addOutputEvent(
                "jobExchange", [self.getStateValue("strID"), self.currentAMR])
            return True

        elif self.state == "BUSY":
            equipmentInfo = self.globalVar.getEquipmentInfoByID(
                self.getStateValue("strID"))
            jobInfo = self.globalVar.getTargetJobsByID(
                equipmentInfo.intProcessingJobID)

            # Process Time 기록
            equipmentInfo.totalProcessedTime += self.getStateValue(
                "processTime")

            # Job 완료 처리
            equipmentInfo.setEquipmentState("DONE")
            jobInfo.setTime('done', self.getStateValue(
                "strID"), self.getTime())

            if equipmentInfo.strType == "SINK":
                # 최종 출구: 완료 처리
                jobInfo.setTime('out', self.getStateValue(
                    "strID"), self.getTime())
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] Job #{jobInfo.intJobID} COMPLETED (waiting for AMR)"
                )
            else:
                # 일반 공정: 처리 완료
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] Job #{jobInfo.intJobID} process done (waiting for AMR)"
                )

            self.addOutputEvent(
                "informDone", [self.getStateValue("strID"), jobInfo.intJobID])
            return True

        elif self.state == "UNLOAD":
            # Equipment → AMR: 작업 전달
            equipmentInfo = self.globalVar.getEquipmentInfoByID(
                self.getStateValue("strID"))
            jobInfo = self.globalVar.getTargetJobsByID(self.currentJob)

            jobInfo.setTime('out', self.getStateValue("strID"), self.getTime())

            self.globalVar.printTerminal(
                f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] Job #{self.currentJob} unloaded to AMR {self.currentAMR}"
            )
            # self.addOutputEvent(
            #     "jobExchange", [self.getStateValue("strID"), self.currentAMR])

            return True

        elif self.state == "INFORM":
            equipmentInfo = self.globalVar.getEquipmentInfoByID(
                self.getStateValue("strID"))
            self.globalVar.printTerminal(
                f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] Ready for next job"
            )
            self.addOutputEvent("informFree", self.getStateValue("strID"))
            return True

        elif self.state == "UNDOCKING":
            print(
                f"🚪 [EQUIPMENT_UNDOCKING] {self.getStateValue('strID')}: Using currentAMR={self.currentAMR}, phase={self.dockingPhase}")

            # ⚠️ 좌표 설정은 Local_Planner에서만 처리 (충돌 방지)
            # Out_pos = self.globalVar.getEquipmentInfoByID(
            #     self.getStateValue("strID")).outputPort.get('position')
            # self.globalVar.getVehicleInfoByID(
            #     self.currentAMR).setCoordinates([Out_pos['x'], Out_pos['y']])

            # Equipment 연결만 해제
            self.globalVar.getVehicleInfoByID(
                self.currentAMR).setEquipmentID(None)

            # Phase에 따라 다른 신호 전송
            if self.dockingPhase == "TO_FROM":
                # FROM Equipment: 픽업 후 → 작업 ID와 함께 신호
                self.addOutputEvent("UndockingComplete", [
                    self.currentAMR, self.getStateValue("strID"), self.currentJob, "TO_FROM"])
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] FROM UNDOCKING: Job #{self.currentJob} picked up by AMR {self.currentAMR}"
                )
            else:  # TO_DESTINATION
                # TO Equipment: 하역 후 → FREE 신호
                self.addOutputEvent("UndockingComplete", [
                    self.currentAMR, self.getStateValue("strID"), self.currentJob, "TO_DESTINATION"])
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] TO UNDOCKING: AMR {self.currentAMR} job completed - sending FREE signal"
                )
        else:
            print(f"ERROR at Equipment Output: #{self.getStateValue('strID')}")
            print(f"CurrentState: {self.state}")
            return False

    def funcInternalTransition(self):
        if self.state == "DOCKING":
            equipmentInfo = self.globalVar.getEquipmentInfoByID(
                self.getStateValue("strID"))

            # GlobalVar에 AMR-장비 연결 저장
            self.globalVar.getVehicleInfoByID(
                self.currentAMR).setEquipmentID(self.getStateValue("strID"))

            # AMR 위치를 WORK 포트로 설정 (도킹 후 1초)
            work_pos = equipmentInfo.workPosition.get('position')
            self.globalVar.getVehicleInfoByID(
                self.currentAMR).setCoordinates([work_pos['x'], work_pos['y']])
            # Phase에 따라 LOAD/UNLOAD 분기
            if self.dockingPhase == "TO_FROM":
                # From Equipment: 픽업 → UNLOAD
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] Phase TO_FROM → UNLOAD (pickup)"
                )
                self.globalVar.getVehicleInfoByID(
                    self.currentAMR).setCoordinates([work_pos['x'], work_pos['y']])
                self.state = self.stateList[4]  # UNLOAD
            else:  # TO_DESTINATION
                # To Equipment: 하역 → LOAD
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] Phase TO_DESTINATION → LOAD (delivery)"
                )
                self.state = self.stateList[1]  # LOAD
            return True

        elif self.state == "LOAD":
            # LOAD → UNDOCKING (AMR이 작업을 전달하고 나감)
            self.state = self.stateList[7]  # UNDOCKING
            return True

        elif self.state == "BUSY":
            # BUSY → DONE (AMR 대기)
            equipmentInfo = self.globalVar.getEquipmentInfoByID(
                self.getStateValue("strID"))
            if equipmentInfo.strType == "SINK":
                self.state = self.stateList[5]  # INFORM
            else:
                self.state = self.stateList[3]  # DONE

            return True

        elif self.state == "UNLOAD":
            # UNLOAD → UNDOCKING (AMR이 작업을 가지고 나감)
            self.state = self.stateList[7]  # UNDOCKING
            return True

        elif self.state == "INFORM":
            # INFORM → EMPTY (대기 상태로)
            equipmentInfo = self.globalVar.getEquipmentInfoByID(
                self.getStateValue("strID"))
            equipmentInfo.setEquipmentState("EMPTY")
            equipmentInfo.intProcessingJobID = None

            self.state = self.stateList[0]  # EMPTY
            self.currentJob = None
            return True

        elif self.state == "UNDOCKING":
            # Phase에 따라 다른 상태로 전환
            if self.dockingPhase == "TO_FROM":
                # FROM Equipment: UNDOCKING → EMPTY (작업을 AMR이 가져감)
                equipmentInfo = self.globalVar.getEquipmentInfoByID(
                    self.getStateValue("strID"))
                equipmentInfo.setEquipmentState("EMPTY")
                equipmentInfo.intProcessingJobID = None

                print(
                    f"🚪 [EQUIPMENT_AMR] {self.getStateValue('strID')}: Clearing currentAMR {self.currentAMR} → None (TO_FROM complete)")
                self.state = self.stateList[0]  # EMPTY
                self.currentJob = None
                self.currentAMR = None
                return True
            else:  # TO_DESTINATION
                # TO Equipment: UNDOCKING → BUSY (작업 처리 시작)
                equipmentInfo = self.globalVar.getEquipmentInfoByID(
                    self.getStateValue("strID"))
                equipmentInfo.setProcessingJobID(self.currentJob)
                equipmentInfo.setEquipmentState("BUSY")

                print(
                    f"🚪 [EQUIPMENT_AMR] {self.getStateValue('strID')}: Clearing currentAMR {self.currentAMR} → None (TO_DESTINATION complete)")
                self.state = self.stateList[2]  # BUSY
                self.currentAMR = None
                return True
        else:
            print(
                f"ERROR at Equipment InternalTransition: #{self.getStateValue('strID')}")
            print(f"CurrentState: {self.state}")
            return False

    def funcTimeAdvance(self):
        if self.state == "EMPTY":
            return float('inf')
        elif self.state == "LOAD" or self.state == "UNLOAD":
            return self.getStateValue("exchangeTime")
        elif self.state == "BUSY":
            return self.getStateValue("processTime")
        elif self.state == "DONE":
            return float('inf')  # AMR 도착 대기
        elif self.state == "INFORM":
            return 0
        elif self.state == "DOCKING":
            return 1
        elif self.state == "UNDOCKING":
            return 1
        else:
            return float('inf')
