"""Transport commands for the AMR fleet.

Modelled on OHT scheduling practice, as a From-To command.
"""


class TransportCommand:
    """Base class for a transport command."""

    def __init__(self, commandID, jobID, fromLocation, toLocation, globalVar):
        """fromLocation and toLocation are dicts of {'nodeID', 'position', 'equipmentID'}."""
        self.commandID = commandID
        self.jobID = jobID
        self.fromLocation = fromLocation
        self.toLocation = toLocation
        self.globalVar = globalVar

        # Command state
        # PENDING:          waiting
        # ASSIGNED:         an AMR has been assigned
        # MOVING_TO_PICKUP: driving to the pickup point
        # LOADING:          loading
        # TRANSPORTING:     carrying the job to its destination
        # UNLOADING:        unloading
        # DONE:             finished
        self.status = "PENDING"

        # Assigned AMR
        self.assignedAMR = None
        self.assignedTime = None

        # Timestamps
        self.createTime = None
        self.startTime = None
        self.pickupTime = None
        self.deliveryTime = None
        self.completeTime = None

    def assignAMR(self, amrID, currentTime):
        """Assign an AMR to this command."""
        self.assignedAMR = amrID
        self.assignedTime = currentTime
        self.status = "ASSIGNED"

    def updateStatus(self, newStatus, currentTime):
        """Update the command state and stamp the corresponding time."""
        oldStatus = self.status
        self.status = newStatus

        if newStatus == "MOVING_TO_PICKUP" and self.startTime is None:
            self.startTime = currentTime
        elif newStatus == "LOADING" and self.pickupTime is None:
            self.pickupTime = currentTime
        elif newStatus == "UNLOADING" and self.deliveryTime is None:
            self.deliveryTime = currentTime
        elif newStatus == "DONE" and self.completeTime is None:
            self.completeTime = currentTime

        return oldStatus

    def getFromPosition(self):
        """Pickup position."""
        return self.fromLocation['position']

    def getToPosition(self):
        """Destination position."""
        return self.toLocation['position']

    def getFromNodeID(self):
        """Pickup node ID."""
        return self.fromLocation['nodeID']

    def getToNodeID(self):
        """Destination node ID."""
        return self.toLocation['nodeID']

    def getSourceEquipmentID(self):
        """Source equipment ID."""
        return self.fromLocation.get('equipmentID', None)

    def getDestEquipmentID(self):
        """Destination equipment ID."""
        return self.toLocation.get('equipmentID', None)

    def isCompleted(self):
        """Whether the command has finished."""
        return self.status == "DONE"

    def isPending(self):
        """Whether the command is still pending."""
        return self.status == "PENDING"

    def isAssigned(self):
        """Whether an AMR has been assigned."""
        return self.status != "PENDING" and not self.isCompleted()

    def __str__(self):
        return f"TransportCommand(ID={self.commandID}, Job={self.jobID}, " \
               f"From={self.fromLocation['nodeID']}, To={self.toLocation['nodeID']}, " \
               f"AMR={self.assignedAMR}, Status={self.status})"

    def __repr__(self):
        return self.__str__()


class SourceToEquipmentCommand(TransportCommand):
    """Transport from the SOURCE to a process machine."""

    def __init__(self, commandID, jobID, sourceEquipment, targetEquipment, globalVar):
        # pickup: enter through the SOURCE input port
        # the AMR enters at SOURCE_IN and leaves loaded from SOURCE_OUT
        fromLocation = {
            'nodeID': sourceEquipment.inputPort['nodeID'] if sourceEquipment.inputPort else sourceEquipment.workPosition['nodeID'],
            'position': sourceEquipment.inputPort['position'] if sourceEquipment.inputPort else sourceEquipment.workPosition['position'],
            'equipmentID': sourceEquipment.strEquipmentID
        }

        # delivery: enter through the target machine's input port
        # the AMR enters at TARGET_IN, unloads, and leaves by TARGET_OUT
        toLocation = {
            'nodeID': targetEquipment.inputPort['nodeID'] if targetEquipment.inputPort else targetEquipment.workPosition['nodeID'],
            'position': targetEquipment.inputPort['position'] if targetEquipment.inputPort else targetEquipment.workPosition['position'],
            'equipmentID': targetEquipment.strEquipmentID
        }

        super().__init__(commandID, jobID, fromLocation, toLocation, globalVar)
        self.commandType = "SOURCE_TO_EQUIPMENT"


class EquipmentToEquipmentCommand(TransportCommand):
    """Transport between two process machines."""

    def __init__(self, commandID, jobID, fromEquipment, toEquipment, globalVar):
        # pickup: enter through the source machine's input port
        # the AMR enters at FROM_IN and leaves loaded from FROM_OUT
        fromLocation = {
            'nodeID': fromEquipment.inputPort['nodeID'] if fromEquipment.inputPort else fromEquipment.workPosition['nodeID'],
            'position': fromEquipment.inputPort['position'] if fromEquipment.inputPort else fromEquipment.workPosition['position'],
            'equipmentID': fromEquipment.strEquipmentID
        }

        # delivery: enter through the target machine's input port
        # the AMR enters at TO_IN, unloads, and leaves by TO_OUT
        toLocation = {
            'nodeID': toEquipment.inputPort['nodeID'] if toEquipment.inputPort else toEquipment.workPosition['nodeID'],
            'position': toEquipment.inputPort['position'] if toEquipment.inputPort else toEquipment.workPosition['position'],
            'equipmentID': toEquipment.strEquipmentID
        }

        super().__init__(commandID, jobID, fromLocation, toLocation, globalVar)
        self.commandType = "EQUIPMENT_TO_EQUIPMENT"


class EquipmentToSinkCommand(TransportCommand):
    """The final transport, from a process machine to the SINK."""

    def __init__(self, commandID, jobID, fromEquipment, sinkEquipment, globalVar):
        # pickup: enter through the source machine's input port
        # the AMR enters at FROM_IN and leaves loaded from FROM_OUT
        fromLocation = {
            'nodeID': fromEquipment.inputPort['nodeID'] if fromEquipment.inputPort else fromEquipment.workPosition['nodeID'],
            'position': fromEquipment.inputPort['position'] if fromEquipment.inputPort else fromEquipment.workPosition['position'],
            'equipmentID': fromEquipment.strEquipmentID
        }

        # delivery: enter through the SINK input port
        # the AMR enters at SINK_IN, unloads, and leaves by SINK_OUT
        toLocation = {
            'nodeID': sinkEquipment.inputPort['nodeID'] if sinkEquipment.inputPort else sinkEquipment.workPosition['nodeID'],
            'position': sinkEquipment.inputPort['position'] if sinkEquipment.inputPort else sinkEquipment.workPosition['position'],
            'equipmentID': sinkEquipment.strEquipmentID
        }

        super().__init__(commandID, jobID, fromLocation, toLocation, globalVar)
        self.commandType = "EQUIPMENT_TO_SINK"


class TransportCommandManager:
    """Creates and holds the transport commands."""

    def __init__(self, globalVar):
        self.globalVar = globalVar
        self.commands = {}  # commandID -> TransportCommand
        self.nextCommandID = 1

    def createCommand(self, jobID, fromEquipment, toEquipment):
        """Build the transport command that matches the source and target machine types."""
        commandID = f"CMD_{self.nextCommandID:06d}"
        self.nextCommandID += 1

        # the machine types select the command class
        if fromEquipment.strType == "SOURCE":
            command = SourceToEquipmentCommand(
                commandID, jobID, fromEquipment, toEquipment, self.globalVar)
        elif toEquipment.strType == "SINK":
            command = EquipmentToSinkCommand(
                commandID, jobID, fromEquipment, toEquipment, self.globalVar)
        else:
            command = EquipmentToEquipmentCommand(
                commandID, jobID, fromEquipment, toEquipment, self.globalVar)

        self.commands[commandID] = command
        return command

    def getCommand(self, commandID):
        """Look a command up by its ID."""
        return self.commands.get(commandID, None)

    def getPendingCommands(self):
        """Commands still pending."""
        return [cmd for cmd in self.commands.values() if cmd.isPending()]

    def getActiveCommands(self):
        """Commands in progress."""
        return [cmd for cmd in self.commands.values() if cmd.isAssigned()]

    def getCommandsByAMR(self, amrID):
        """Commands assigned to one AMR."""
        return [cmd for cmd in self.commands.values() if cmd.assignedAMR == amrID]

    def getCommandsByJob(self, jobID):
        """Commands belonging to one job."""
        return [cmd for cmd in self.commands.values() if cmd.jobID == jobID]

    def removeCommand(self, commandID):
        """Remove a command."""
        if commandID in self.commands:
            del self.commands[commandID]
