from SimulationEngine.ClassicDEVS.DEVSAtomicModel import DEVSAtomicModel


class FleetManagement(DEVSAtomicModel):
    def __init__(self, strID, globalVar):
        super().__init__(strID)
        self.globalVar = globalVar

        # States: WAIT, UPDATE (publish fleet info), SEND_GO_COMMAND (move a parked AMR), SEND_COMMAND (issue an order)
        self.stateList = ["WAIT", "UPDATE", "SEND_GO_COMMAND", "SEND_COMMAND"]
        self.state = self.stateList[0]

        # State Variables
        self.addStateVariable('strID', strID)

        # Input Ports
        self.addInputPort("amrPosition")  # AMR pose
        self.addInputPort("taskAssign")   # task assignment from Scheduler
        self.addInputPort("undockingComplete")  # undocking complete, from Local_Planner

        # Output Ports
        self.addOutputPort("fleetInfo")   # fleet state sent to Scheduler
        self.addOutputPort("amrCommand")  # order sent to an AMR

        # Variables
        self.amrPositions = {}  # AMR poses
        self.lastUpdateTime = 0
        # transport command per AMR, {amrID: transportCommand}
        self.jobSequences = {}
        # the command to emit on the next output
        self.pendingCommand = None
        self.watingARMCommand = None

    def funcExternalTransition(self, strPort, objEvent):
        if strPort == "amrPosition":
            # Update the AMR pose
            amrID = objEvent.strID
            self.amrPositions[amrID] = {
                'x': objEvent.x,
                'y': objEvent.y,
                'yaw': objEvent.yaw,
                'lin_vel': objEvent.lin_vel,
                'ang_vel': objEvent.ang_vel,
                'timestamp': self.getTime()
            }

            # keep the vehicle record in GlobalVar in step
            vehicleInfo = self.globalVar.getVehicleInfoByID(amrID)
            if vehicleInfo:
                # while docked, keep the coordinates Equipment set
                keep_equipment_coords = False
                if hasattr(vehicleInfo, 'getEquipmentID') and vehicleInfo.getEquipmentID():
                    keep_equipment_coords = True

                if not keep_equipment_coords:
                    vehicleInfo.setCoordinates([objEvent.x, objEvent.y])

                # a stopped robot with no job goes back to IDLE
                if objEvent.lin_vel < 0.1 and vehicleInfo.intJobID is None:
                    if vehicleInfo.strState != "IDLE":
                        vehicleInfo.setState("IDLE")
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][FleetManagement] AMR {amrID} is now IDLE"
                        )

            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] AMR {amrID} position updated: ({objEvent.x:.2f}, {objEvent.y:.2f}), State: {vehicleInfo.strState if vehicleInfo else 'Unknown'}"
            )

            # move to UPDATE so the change reaches Scheduler
            # only from WAIT: never interrupt a command in flight
            if self.state == "WAIT":
                self.state = self.stateList[1]  # UPDATE

            return True

        elif strPort == "taskAssign":
            # a transport command has arrived from Scheduler
            transportCommand = objEvent
            amrID = transportCommand.assignedAMR
            jobID = transportCommand.jobID

            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] Received TransportCommand: {transportCommand}"
            )

            # an AMR with a new job is cleared from every machine's undockedAMRs
            for equipmentInfo in self.globalVar.getEquipmentInfo().values():
                if amrID in equipmentInfo.undockedAMRs:
                    equipmentInfo.undockedAMRs.remove(amrID)
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] 🗑️ Removed AMR {amrID} from Equipment {equipmentInfo.strEquipmentID} undocked list"
                    )

            # store the command against its AMR
            self.jobSequences[amrID] = transportCommand
            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] 📋 Stored TransportCommand for AMR {amrID}: {transportCommand.commandID}"
            )

            # build the move order to the pickup machine
            self.pendingCommand = {
                'amrID': amrID,
                'jobID': jobID,
                'commandID': transportCommand.commandID,
                'fromPosition': transportCommand.getFromPosition(),
                'toPosition': transportCommand.getToPosition(),
                'fromNodeID': transportCommand.getFromNodeID(),
                'toNodeID': transportCommand.getToNodeID(),
                'action': 'TRANSPORT',  # FROM -> TO transport
                'phase': 'TO_FROM'  # first leg: to the FROM machine
            }

            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] 📥 Command prepared for AMR {amrID}: {transportCommand.getFromNodeID()} → {transportCommand.getToNodeID()}"
            )

            # move to SEND_COMMAND to emit it at once
            self.state = self.stateList[3]  # SEND_COMMAND

            return True

        elif strPort == "undockingComplete_I":
            # undocking complete, reported by Local_Planner
            amrID = objEvent[0]
            equipmentID = objEvent[1]
            jobID = objEvent[2]
            transportPhase = objEvent[3]

            vehicleInfo = self.globalVar.getVehicleInfoByID(amrID)
            current_coords = vehicleInfo.getCoordinates() if vehicleInfo else "Unknown"

            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] 🚪 UNDOCKING complete: AMR {amrID} from {equipmentID}, Phase: {transportPhase}"
            )

            # from here the transport phase decides what happens

            # WAITING: the robot has reached a waiting area
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
                # on arrival, drop any pending order held for this AMR
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
                # tell Scheduler the fleet state changed
                self.state = self.stateList[1]  # UPDATE

            # TO_DESTINATION: either the job is finished or a waiting area was reached
            elif transportPhase == "TO_DESTINATION":
                # distinguish a waiting area from a machine
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
                    # on arrival, drop any pending order held for this AMR
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
                    # arrived at a machine: the job is done
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

                    # retire the transport command
                    if amrID in self.jobSequences:
                        del self.jobSequences[amrID]

                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] ✅ AMR {amrID} is now FREE - Job #{jobID} completed"
                    )

                # an AMR just became idle, so let Scheduler assign it something
                self.state = self.stateList[1]  # UPDATE

            # TO_FROM: undocked from the pickup machine, now head for the destination
            elif transportPhase == "TO_FROM" and amrID in self.jobSequences:
                transportCommand = self.jobSequences[amrID]

                # read the destination machine from the transport command
                nextNodeID = transportCommand.getToNodeID()
                nextEquipmentID = nextNodeID.split('_')[0]  # "B-1_IN" -> "B-1"

                # the AMR departs from the output port
                currentEquipmentInfo = self.globalVar.getEquipmentInfoByID(
                    equipmentID)
                nextEquipmentInfo = self.globalVar.getEquipmentInfoByID(
                    nextEquipmentID)

                if currentEquipmentInfo and nextEquipmentInfo:
                    currentPos = currentEquipmentInfo.outputPort.get(
                        'position')
                    nextPos = transportCommand.getToPosition()

                    # build the move order to the destination and hold it
                    self.pendingCommand = {
                        'amrID': amrID,
                        'jobID': jobID,
                        'commandID': f"NEXT_{jobID}_{equipmentID}_{nextEquipmentID}",
                        'fromPosition': currentPos,
                        'toPosition': nextPos,
                        'fromNodeID': equipmentID + '_OUT',
                        'toNodeID': nextNodeID,
                        'action': 'TRANSPORT_NEXT',
                        'phase': 'TO_DESTINATION'  # second leg: to the TO machine
                    }

                    # is another AMR parked, undocked, in front of the destination?
                    if nextEquipmentInfo.undockedAMRs:
                        # tell that AMR to give way
                        waitingAMR = list(nextEquipmentInfo.undockedAMRs)[
                            0]  # the first AMR

                        # where the blocking AMR currently is
                        waitingVehicleInfo = self.globalVar.getVehicleInfoByID(
                            waitingAMR)
                        if waitingVehicleInfo and waitingAMR in self.amrPositions:
                            waiting_pose = self.amrPositions[waitingAMR]
                            current_pos = [
                                waiting_pose['x'], waiting_pose['y']]
                        else:
                            # if the pose is unknown, fall back to the machine's output port
                            current_pos = [nextPos['x'], nextPos['y']]

                        # send it to the nearest waiting area; areas are not exclusive
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
                            # with no waiting area available, just push it 10 m forward
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

                        # either way, clear it from the machine's undockedAMRs
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

                        # emit the give-way order first
                        self.state = self.stateList[2]  # SEND_GO_COMMAND
                    else:
                        self.globalVar.printTerminal(
                            f"[{self.getTime()}][FleetManagement] ✅ TO Equipment {nextEquipmentID}에 대기 중인 AMR 없음"
                        )

                        # nothing in the way: emit the transport order directly
                        self.state = self.stateList[3]  # SEND_COMMAND

                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] 🚚 Next destination: {equipmentID}_OUT → {nextNodeID} for Job #{jobID}"
                    )
                else:
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][FleetManagement] ⚠️ Equipment info not found: {equipmentID} or {nextEquipmentID}"
                    )

            # otherwise: this AMR has no transport command attached
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
            # publish the fleet state to Scheduler
            self.addOutputEvent("fleetInfo", self.amrPositions.copy())

            self.globalVar.printTerminal(
                f"[{self.getTime()}][FleetManagement] Sending fleet info to Scheduler (AMRs: {len(self.amrPositions)})"
            )

            return True

        elif self.state == "SEND_GO_COMMAND":
            # emit the give-way order
            if self.watingARMCommand:
                self.addOutputEvent("amrGoCommand", self.watingARMCommand)

                self.globalVar.printTerminal(
                    f"[{self.getTime()}][FleetManagement] 🚶 GO Command sent to waiting AMR {self.watingARMCommand['amrID']}: Move to ({self.watingARMCommand['x']}, {self.watingARMCommand['y']})"
                )

            return True

        elif self.state == "SEND_COMMAND":
            # emit the order that was held
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
            # after UPDATE, return to WAIT
            self.state = self.stateList[0]  # WAIT
            self.lastUpdateTime = self.getTime()
            return True

        elif self.state == "SEND_GO_COMMAND":
            # the give-way order is out; now send the order that was held
            self.watingARMCommand = None
            self.state = self.stateList[3]  # SEND_COMMAND
            return True

        elif self.state == "SEND_COMMAND":
            # order sent: clear it and return to WAIT
            self.pendingCommand = None
            self.state = self.stateList[0]  # WAIT
            return True

        else:
            return True

    def funcTimeAdvance(self):
        if self.state == "WAIT":
            return float('inf')
        elif self.state == "UPDATE":
            return 0  # emit at once
        elif self.state == "SEND_GO_COMMAND":
            return 0  # emit at once
        elif self.state == "SEND_COMMAND":
            return 0  # emit at once
        else:
            return float('inf')
