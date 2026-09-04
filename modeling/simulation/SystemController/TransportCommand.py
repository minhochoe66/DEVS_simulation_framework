"""AMR 운반 명령.

OHT 스케줄링 방식을 참고하여 From-To 명령 체계로 구현하였다.
"""


class TransportCommand:
    """운반 명령의 기본 클래스."""

    def __init__(self, commandID, jobID, fromLocation, toLocation, globalVar):
        """fromLocation과 toLocation은 {'nodeID', 'position', 'equipmentID'} 형식이다."""
        self.commandID = commandID
        self.jobID = jobID
        self.fromLocation = fromLocation
        self.toLocation = toLocation
        self.globalVar = globalVar

        # 명령 상태
        # PENDING: 대기중
        # ASSIGNED: AMR 할당됨
        # MOVING_TO_PICKUP: 픽업 위치로 이동중
        # LOADING: 적재중
        # TRANSPORTING: 목적지로 운반중
        # UNLOADING: 하역중
        # DONE: 완료
        self.status = "PENDING"

        # 할당된 AMR
        self.assignedAMR = None
        self.assignedTime = None

        # 시각 기록
        self.createTime = None
        self.startTime = None
        self.pickupTime = None
        self.deliveryTime = None
        self.completeTime = None

    def assignAMR(self, amrID, currentTime):
        """명령에 AMR을 할당한다."""
        self.assignedAMR = amrID
        self.assignedTime = currentTime
        self.status = "ASSIGNED"

    def updateStatus(self, newStatus, currentTime):
        """명령 상태를 갱신하고 해당 시각을 기록한다."""
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
        """픽업 좌표."""
        return self.fromLocation['position']

    def getToPosition(self):
        """목적지 좌표."""
        return self.toLocation['position']

    def getFromNodeID(self):
        """픽업 노드 ID."""
        return self.fromLocation['nodeID']

    def getToNodeID(self):
        """목적지 노드 ID."""
        return self.toLocation['nodeID']

    def getSourceEquipmentID(self):
        """출발 장비 ID."""
        return self.fromLocation.get('equipmentID', None)

    def getDestEquipmentID(self):
        """목적지 장비 ID."""
        return self.toLocation.get('equipmentID', None)

    def isCompleted(self):
        """명령이 완료되었는가."""
        return self.status == "DONE"

    def isPending(self):
        """명령이 대기 중인가."""
        return self.status == "PENDING"

    def isAssigned(self):
        """명령에 AMR이 할당되었는가."""
        return self.status != "PENDING" and not self.isCompleted()

    def __str__(self):
        return f"TransportCommand(ID={self.commandID}, Job={self.jobID}, " \
               f"From={self.fromLocation['nodeID']}, To={self.toLocation['nodeID']}, " \
               f"AMR={self.assignedAMR}, Status={self.status})"

    def __repr__(self):
        return self.__str__()


class SourceToEquipmentCommand(TransportCommand):
    """SOURCE에서 공정 장비로 운반하는 명령."""

    def __init__(self, commandID, jobID, sourceEquipment, targetEquipment, globalVar):
        # 픽업: SOURCE의 InputPort로 진입한다
        # AMR은 SOURCE_IN으로 들어가 SOURCE_OUT에서 싣고 나온다
        fromLocation = {
            'nodeID': sourceEquipment.inputPort['nodeID'] if sourceEquipment.inputPort else sourceEquipment.workPosition['nodeID'],
            'position': sourceEquipment.inputPort['position'] if sourceEquipment.inputPort else sourceEquipment.workPosition['position'],
            'equipmentID': sourceEquipment.strEquipmentID
        }

        # 하역: 목표 장비의 InputPort로 진입한다
        # AMR은 TARGET_IN으로 들어가 하역하고 TARGET_OUT으로 나온다
        toLocation = {
            'nodeID': targetEquipment.inputPort['nodeID'] if targetEquipment.inputPort else targetEquipment.workPosition['nodeID'],
            'position': targetEquipment.inputPort['position'] if targetEquipment.inputPort else targetEquipment.workPosition['position'],
            'equipmentID': targetEquipment.strEquipmentID
        }

        super().__init__(commandID, jobID, fromLocation, toLocation, globalVar)
        self.commandType = "SOURCE_TO_EQUIPMENT"


class EquipmentToEquipmentCommand(TransportCommand):
    """공정 장비 사이를 운반하는 명령."""

    def __init__(self, commandID, jobID, fromEquipment, toEquipment, globalVar):
        # 픽업: 출발 장비의 InputPort로 진입한다
        # AMR은 FROM_IN으로 들어가 FROM_OUT에서 싣고 나온다
        fromLocation = {
            'nodeID': fromEquipment.inputPort['nodeID'] if fromEquipment.inputPort else fromEquipment.workPosition['nodeID'],
            'position': fromEquipment.inputPort['position'] if fromEquipment.inputPort else fromEquipment.workPosition['position'],
            'equipmentID': fromEquipment.strEquipmentID
        }

        # 하역: 목표 장비의 InputPort로 진입한다
        # AMR은 TO_IN으로 들어가 하역하고 TO_OUT으로 나온다
        toLocation = {
            'nodeID': toEquipment.inputPort['nodeID'] if toEquipment.inputPort else toEquipment.workPosition['nodeID'],
            'position': toEquipment.inputPort['position'] if toEquipment.inputPort else toEquipment.workPosition['position'],
            'equipmentID': toEquipment.strEquipmentID
        }

        super().__init__(commandID, jobID, fromLocation, toLocation, globalVar)
        self.commandType = "EQUIPMENT_TO_EQUIPMENT"


class EquipmentToSinkCommand(TransportCommand):
    """공정 장비에서 SINK로 운반하는 마지막 명령."""

    def __init__(self, commandID, jobID, fromEquipment, sinkEquipment, globalVar):
        # 픽업: 출발 장비의 InputPort로 진입한다
        # AMR은 FROM_IN으로 들어가 FROM_OUT에서 싣고 나온다
        fromLocation = {
            'nodeID': fromEquipment.inputPort['nodeID'] if fromEquipment.inputPort else fromEquipment.workPosition['nodeID'],
            'position': fromEquipment.inputPort['position'] if fromEquipment.inputPort else fromEquipment.workPosition['position'],
            'equipmentID': fromEquipment.strEquipmentID
        }

        # 하역: SINK의 InputPort로 진입한다
        # AMR은 SINK_IN으로 들어가 하역하고 SINK_OUT으로 나온다
        toLocation = {
            'nodeID': sinkEquipment.inputPort['nodeID'] if sinkEquipment.inputPort else sinkEquipment.workPosition['nodeID'],
            'position': sinkEquipment.inputPort['position'] if sinkEquipment.inputPort else sinkEquipment.workPosition['position'],
            'equipmentID': sinkEquipment.strEquipmentID
        }

        super().__init__(commandID, jobID, fromLocation, toLocation, globalVar)
        self.commandType = "EQUIPMENT_TO_SINK"


class TransportCommandManager:
    """운반 명령을 생성하고 보관한다."""

    def __init__(self, globalVar):
        self.globalVar = globalVar
        self.commands = {}  # commandID -> TransportCommand
        self.nextCommandID = 1

    def createCommand(self, jobID, fromEquipment, toEquipment):
        """출발 장비와 목표 장비의 종류에 맞는 운반 명령을 만든다."""
        commandID = f"CMD_{self.nextCommandID:06d}"
        self.nextCommandID += 1

        # 장비 종류로 명령 클래스를 고른다
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
        """commandID로 명령을 찾는다."""
        return self.commands.get(commandID, None)

    def getPendingCommands(self):
        """대기 중인 명령."""
        return [cmd for cmd in self.commands.values() if cmd.isPending()]

    def getActiveCommands(self):
        """실행 중인 명령."""
        return [cmd for cmd in self.commands.values() if cmd.isAssigned()]

    def getCommandsByAMR(self, amrID):
        """특정 AMR에 할당된 명령."""
        return [cmd for cmd in self.commands.values() if cmd.assignedAMR == amrID]

    def getCommandsByJob(self, jobID):
        """특정 작업에 속한 명령."""
        return [cmd for cmd in self.commands.values() if cmd.jobID == jobID]

    def removeCommand(self, commandID):
        """명령을 제거한다."""
        if commandID in self.commands:
            del self.commands[commandID]
