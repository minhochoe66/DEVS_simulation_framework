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
        self.addInputPort("amrDocking")  # docking signal for Maneuver, unused here
        self.addInputPort("EquipmentDocking")  # docking signal for Equipment

        # Output Ports
        self.addOutputPort("informDone")
        self.addOutputPort("informFree")
        self.addOutputPort("jobExchange")  # job handover with the AMR

        # State Variables
        self.addStateVariable("strID", strID)
        self.addStateVariable("processType", equipmentInfo.strType)
        self.addStateVariable("stageID", equipmentInfo.strStageID)
        self.addStateVariable("processTime", equipmentInfo.dblProcessTime)
        self.addStateVariable("exchangeTime", equipmentInfo.dblExchangeTime)

        # Variables
        self.currentJob = None
        self.reservedJob = None  # job reserved by a job event
        self.currentAMR = None   # ID of the currently docked AMR
        self.dockingPhase = None  # "TO_FROM" or "TO_DESTINATION"

    def funcExternalTransition(self, strPort, objEvent):
        if strPort == "job":
            # objEvent: [equipmentID, jobID]
            if self.getStateValue("strID") == objEvent[0]:
                jobID = objEvent[1]
                equipmentInfo = self.globalVar.getEquipmentInfoByID(
                    objEvent[0])

                if equipmentInfo.strType == "SOURCE":
                    # SOURCE starts processing immediately, without an AMR
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
                    # other machines reserve the job and wait for an AMR
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
            # AMR -> Equipment: hand the job over
            equipmentInfo = self.globalVar.getEquipmentInfoByID(
                self.getStateValue("strID"))

            # TO_DESTINATION uses the job ID the AMR carries
            # TO_FROM uses the reserved job ID
            if self.dockingPhase == "TO_DESTINATION":
                amr_job_id = self.globalVar.getVehicleInfoByID(
                    self.currentAMR).intJobID
                if amr_job_id is None:
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] Warning: AMR {self.currentAMR} has no job ID"
                    )
                    return
                job_id = amr_job_id
            else:
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

            # Record the processing time
            equipmentInfo.totalProcessedTime += self.getStateValue(
                "processTime")

            # Job completion
            equipmentInfo.setEquipmentState("DONE")
            jobInfo.setTime('done', self.getStateValue(
                "strID"), self.getTime())

            if equipmentInfo.strType == "SINK":
                # SINK: the job leaves the system
                jobInfo.setTime('out', self.getStateValue(
                    "strID"), self.getTime())
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] Job #{jobInfo.intJobID} COMPLETED (waiting for AMR)"
                )
            else:
                # ordinary stage: report completion
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] Job #{jobInfo.intJobID} process done (waiting for AMR)"
                )

            self.addOutputEvent(
                "informDone", [self.getStateValue("strID"), jobInfo.intJobID])
            return True

        elif self.state == "UNLOAD":
            # Equipment -> AMR: hand the job over
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

            # Coordinates are set only by Local_Planner, to avoid conflicting writes
            # Out_pos = self.globalVar.getEquipmentInfoByID(
            #     self.getStateValue("strID")).outputPort.get('position')
            # self.globalVar.getVehicleInfoByID(
            #     self.currentAMR).setCoordinates([Out_pos['x'], Out_pos['y']])

            # release only the equipment linkage
            self.globalVar.getVehicleInfoByID(
                self.currentAMR).setEquipmentID(None)

            # the signal sent depends on the transport phase
            if self.dockingPhase == "TO_FROM":
                # FROM equipment: after pickup, report with the job ID
                self.addOutputEvent("UndockingComplete", [
                    self.currentAMR, self.getStateValue("strID"), self.currentJob, "TO_FROM"])
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] FROM UNDOCKING: Job #{self.currentJob} picked up by AMR {self.currentAMR}"
                )
            else:  # TO_DESTINATION
                # TO equipment: after delivery, report free
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

            # record the AMR-equipment linkage in GlobalVar
            self.globalVar.getVehicleInfoByID(
                self.currentAMR).setEquipmentID(self.getStateValue("strID"))

            # one second after docking, move the AMR to the WORK port
            work_pos = equipmentInfo.workPosition.get('position')
            self.globalVar.getVehicleInfoByID(
                self.currentAMR).setCoordinates([work_pos['x'], work_pos['y']])
            # the transport phase selects LOAD or UNLOAD
            if self.dockingPhase == "TO_FROM":
                # FROM equipment: a pickup, so UNLOAD
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] Phase TO_FROM → UNLOAD (pickup)"
                )
                self.globalVar.getVehicleInfoByID(
                    self.currentAMR).setCoordinates([work_pos['x'], work_pos['y']])
                self.state = self.stateList[4]  # UNLOAD
            else:  # TO_DESTINATION
                # TO equipment: a delivery, so LOAD
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Equipment({self.getStateValue('strID')})] Phase TO_DESTINATION → LOAD (delivery)"
                )
                self.state = self.stateList[1]  # LOAD
            return True

        elif self.state == "LOAD":
            # LOAD -> UNDOCKING: the AMR has delivered and leaves
            self.state = self.stateList[7]  # UNDOCKING
            return True

        elif self.state == "BUSY":
            # BUSY -> DONE: wait for an AMR
            equipmentInfo = self.globalVar.getEquipmentInfoByID(
                self.getStateValue("strID"))
            if equipmentInfo.strType == "SINK":
                self.state = self.stateList[5]  # INFORM
            else:
                self.state = self.stateList[3]  # DONE

            return True

        elif self.state == "UNLOAD":
            # UNLOAD -> UNDOCKING: the AMR takes the job and leaves
            self.state = self.stateList[7]  # UNDOCKING
            return True

        elif self.state == "INFORM":
            # INFORM -> EMPTY
            equipmentInfo = self.globalVar.getEquipmentInfoByID(
                self.getStateValue("strID"))
            equipmentInfo.setEquipmentState("EMPTY")
            equipmentInfo.intProcessingJobID = None

            self.state = self.stateList[0]  # EMPTY
            self.currentJob = None
            return True

        elif self.state == "UNDOCKING":
            # the next state depends on the transport phase
            if self.dockingPhase == "TO_FROM":
                # FROM equipment: UNDOCKING -> EMPTY, the AMR took the job
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
                # TO equipment: UNDOCKING -> BUSY, processing starts
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
            return float('inf')  # wait for an AMR to arrive
        elif self.state == "INFORM":
            return 0
        elif self.state == "DOCKING":
            return 1
        elif self.state == "UNDOCKING":
            return 1
        else:
            return float('inf')
