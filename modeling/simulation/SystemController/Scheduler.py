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
        self.addInputPort("informDone")   # Equipment finished a job
        self.addInputPort("informFree")   # Equipment is free
        self.addInputPort("fleetInfo")    # fleet state from FleetManagement

        # Output Ports
        self.addOutputPort("jobAssign")   # job assignment, published outward
        self.addOutputPort("taskAssign")  # task assignment sent to FleetManagement

        # Variables
        self.lstCompleteJob = []  # finished jobs waiting for their next stage
        self.amrPositions = {}    # AMR poses received from FleetManagement

        # transport command manager
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

            # decide the next stage
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

            # SOURCE is driven by Data_generator, so ignore it here
            if equipmentInfo.strType != "SOURCE":
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Scheduler] Equipment {equipmentID} is FREE"
                )
                if self.state == "WAIT":
                    self.state = self.stateList[1]  # COMMAND

            if equipmentInfo.strType == "SINK":
                # job finished
                self.count += 1
                if self.count == self.globalVar.objConfiguration.getConfiguration("numJob"):
                    self.state = self.stateList[2]  # COMPLETE
                else:
                    if self.state == "WAIT":
                        self.state = self.stateList[1]  # COMMAND

        elif strPort == "fleetInfo":
            self.amrPositions = objEvent  # objEvent is a dict

            self.globalVar.printTerminal(
                f"[{self.getTime()}][Scheduler] Received fleet info with {len(self.amrPositions)} AMRs"
            )

            # retry assignment whenever the fleet state changes
            # a job waiting in lstCompleteJob gets another chance
            if self.lstCompleteJob:
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Scheduler] Re-attempting assignment for {len(self.lstCompleteJob)} queued jobs"
                )

            # only WAIT may move to COMMAND
            if self.state == "WAIT":
                self.state = self.stateList[1]  # COMMAND
            # already in COMMAND: it will be re-examined once the current pass ends

            return True

        else:
            print(
                f"ERROR at Scheduler ExternalTransition: #{self.getStateValue('strID')}")
            print(f"inputPort: {strPort}")
            print(f"CurrentState: {self.state}")
            return False

    def funcOutput(self):
        if self.state == "COMMAND":
            # match jobs, machines and robots
            transportCommand = self.checkCommandCondition()

            if transportCommand is not None:
                self.globalVar.printTerminal(
                    f"[{self.getTime()}][Scheduler] ✅ {transportCommand}"
                )

                # hand the transport command to FleetManagement
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
        """Choose the job's next stage, following the routing implied by numStages."""
        currentProcess = jobInfo.strCurrentProcess

        # numStages=1: SOURCE -> STAGE_B -> SINK
        # numStages=2: SOURCE -> STAGE_B -> STAGE_C -> SINK
        # numStages=3: SOURCE -> STAGE_B -> STAGE_C -> STAGE_D -> SINK

        if currentProcess == "SOURCE":
            # after A, always B
            jobInfo.setNextProcess("STAGE_B", None)

        elif currentProcess and currentProcess.startswith("PROCESS_B"):
            # after B, numStages decides
            if self.numStages == 1:
                jobInfo.setNextProcess("SINK", None)
            else:
                jobInfo.setNextProcess("STAGE_C", None)

        elif currentProcess and currentProcess.startswith("PROCESS_C"):
            # after C, numStages decides
            if self.numStages == 2:
                jobInfo.setNextProcess("SINK", None)
            else:
                jobInfo.setNextProcess("STAGE_D", None)

        elif currentProcess and currentProcess.startswith("PROCESS_D"):
            # after D, the SINK
            jobInfo.setNextProcess("SINK", None)

        else:
            # the SINK, or an unknown process type
            jobInfo.setNextProcess(None, None)

        self.globalVar.printTerminal(
            f"[{self.getTime()}][Scheduler] Next process for Job #{jobInfo.intJobID}: {currentProcess} → {jobInfo.strNextProcess} (numStages={self.numStages})"
        )

    def checkCommandCondition(self):
        """Match waiting jobs to free machines and idle robots, and issue transport commands."""
        equipmentInfo = self.globalVar.getEquipmentInfo()
        vehicleInfo = self.globalVar.getVehicleInfo()

        # 1. jobs already waiting for their next stage take priority
        for jobID in self.lstCompleteJob[:]:  # iterate over a copy: the list is modified inside the loop
            jobInfo = self.globalVar.getTargetJobsByID(jobID)

            # the machine holding the job becomes the pickup point
            fromEquipmentID = jobInfo.strCurrentProcessEqpID
            if fromEquipmentID is None:
                continue
            fromEquipment = self.globalVar.getEquipmentInfoByID(
                fromEquipmentID)

            # an EMPTY machine at the next stage becomes the destination
            for equipmentID, equipmentValue in equipmentInfo.items():
                # flexible flow shop: a STAGE_X target matches any machine in that stage
                if jobInfo.strNextProcess and jobInfo.strNextProcess.startswith("STAGE_"):
                    type_match = (equipmentValue.strStageID ==
                                  jobInfo.strNextProcess)
                else:
                    # SOURCE and SINK must match exactly
                    type_match = (equipmentValue.strType ==
                                  jobInfo.strNextProcess)

                if (type_match and equipmentValue.strState == "EMPTY"):

                    # pick the idle AMR closest to the pickup point
                    selectedAMR = self.findAvailableAMR(
                        fromEquipment, vehicleInfo)

                    if selectedAMR is not None:
                        # issue the transport command
                        transportCommand = self.commandManager.createCommand(
                            jobID, fromEquipment, equipmentValue
                        )

                        # assign the AMR
                        transportCommand.assignAMR(selectedAMR, self.getTime())
                        transportCommand.createTime = self.getTime()

                        # assign the job
                        jobInfo.setNextProcess(
                            equipmentValue.strType, equipmentID)
                        self.lstCompleteJob.remove(jobID)
                        equipmentValue.setEquipmentState("RESERVED")

                        # update the AMR state
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

        # 2. then the jobs sitting on machines in the DONE state
        for equipmentID, equipmentValue in equipmentInfo.items():
            if equipmentValue.strState == "DONE" and equipmentValue.intProcessingJobID:
                jobID = equipmentValue.intProcessingJobID
                jobInfo = self.globalVar.getTargetJobsByID(jobID)

                # an EMPTY machine at the next stage becomes the destination
                for nextEquipmentID, nextEquipmentValue in equipmentInfo.items():
                    # flexible flow shop: a STAGE_X target matches any machine in that stage
                    if jobInfo.strNextProcess and jobInfo.strNextProcess.startswith("STAGE_"):
                        type_match = (
                            nextEquipmentValue.strStageID == jobInfo.strNextProcess)
                    else:
                        # SOURCE and SINK must match exactly
                        type_match = (nextEquipmentValue.strType ==
                                      jobInfo.strNextProcess)

                    if (type_match and nextEquipmentValue.strState == "EMPTY"):

                        # pick the idle AMR closest to the pickup point
                        selectedAMR = self.findAvailableAMR(
                            equipmentValue, vehicleInfo)

                        if selectedAMR is not None:
                            # issue the transport command
                            transportCommand = self.commandManager.createCommand(
                                jobID, equipmentValue, nextEquipmentValue
                            )

                            # assign the AMR
                            transportCommand.assignAMR(
                                selectedAMR, self.getTime())
                            transportCommand.createTime = self.getTime()

                            # assign the job
                            jobInfo.setNextProcess(
                                nextEquipmentValue.strType, nextEquipmentID)
                            nextEquipmentValue.setEquipmentState("RESERVED")

                            # update the AMR state
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
        """Pick the idle AMR closest to the pickup point.

        targetEquipment is the source machine, the first place the robot must reach.
        """
        import math

        # the source machine's input port is the pickup point
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
