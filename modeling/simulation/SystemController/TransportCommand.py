"""
TransportCommand: AMR의 물품 운반 명령 관리 클래스

기존 OHT 스케줄링 방식을 참고하여 From-To 명령 체계로 구현
"""


class TransportCommand:
    """운반 명령 기본 클래스"""

    def __init__(self, commandID, jobID, fromLocation, toLocation, globalVar):
        """
        Args:
            commandID: 명령 고유 ID
            jobID: 작업 ID
            fromLocation: 픽업 위치 정보 {'nodeID': str, 'position': dict, 'equipmentID': str}
            toLocation: 전달 위치 정보 {'nodeID': str, 'position': dict, 'equipmentID': str}
            globalVar: 전역 변수 객체
        """
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
        # TRANSPORTING: 운반중 (목적지로 이동)
        # UNLOADING: 하역중
        # DONE: 완료
        self.status = "PENDING"

        # AMR 할당 정보
        self.assignedAMR = None
        self.assignedTime = None

        # 시간 기록
        self.createTime = None
        self.startTime = None
        self.pickupTime = None
        self.deliveryTime = None
        self.completeTime = None

    def assignAMR(self, amrID, currentTime):
        """AMR 할당"""
        self.assignedAMR = amrID
        self.assignedTime = currentTime
        self.status = "ASSIGNED"

    def updateStatus(self, newStatus, currentTime):
        """상태 업데이트"""
        oldStatus = self.status
        self.status = newStatus

        # 시간 기록
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
        """픽업 위치 반환"""
        return self.fromLocation['position']

    def getToPosition(self):
        """목적지 위치 반환"""
        return self.toLocation['position']

    def getFromNodeID(self):
        """픽업 노드 ID 반환"""
        return self.fromLocation['nodeID']

    def getToNodeID(self):
        """목적지 노드 ID 반환"""
        return self.toLocation['nodeID']

    def getSourceEquipmentID(self):
        """출발 장비 ID 반환"""
        return self.fromLocation.get('equipmentID', None)

    def getDestEquipmentID(self):
        """목적지 장비 ID 반환"""
        return self.toLocation.get('equipmentID', None)

    def isCompleted(self):
        """완료 여부"""
        return self.status == "DONE"

    def isPending(self):
        """대기 중 여부"""
        return self.status == "PENDING"

    def isAssigned(self):
        """할당됨 여부"""
        return self.status != "PENDING" and not self.isCompleted()

    def __str__(self):
        return f"TransportCommand(ID={self.commandID}, Job={self.jobID}, " \
               f"From={self.fromLocation['nodeID']}, To={self.toLocation['nodeID']}, " \
               f"AMR={self.assignedAMR}, Status={self.status})"

    def __repr__(self):
        return self.__str__()


class SourceToEquipmentCommand(TransportCommand):
    """Source에서 Equipment로 운반하는 명령"""

    def __init__(self, commandID, jobID, sourceEquipment, targetEquipment, globalVar):
        """
        Args:
            sourceEquipment: Source 장비 객체
            targetEquipment: 목표 장비 객체
        """
        # Source의 InputPort로 진입 (픽업을 위해)
        # AMR: SOURCE_IN → (내부) → SOURCE_OUT에서 픽업하고 진출
        fromLocation = {
            'nodeID': sourceEquipment.inputPort['nodeID'] if sourceEquipment.inputPort else sourceEquipment.workPosition['nodeID'],
            'position': sourceEquipment.inputPort['position'] if sourceEquipment.inputPort else sourceEquipment.workPosition['position'],
            'equipmentID': sourceEquipment.strEquipmentID
        }

        # 목표 장비의 InputPort로 진입 (전달을 위해)
        # AMR: TARGET_IN → (하역) → TARGET_OUT으로 진출
        toLocation = {
            'nodeID': targetEquipment.inputPort['nodeID'] if targetEquipment.inputPort else targetEquipment.workPosition['nodeID'],
            'position': targetEquipment.inputPort['position'] if targetEquipment.inputPort else targetEquipment.workPosition['position'],
            'equipmentID': targetEquipment.strEquipmentID
        }

        super().__init__(commandID, jobID, fromLocation, toLocation, globalVar)
        self.commandType = "SOURCE_TO_EQUIPMENT"


class EquipmentToEquipmentCommand(TransportCommand):
    """Equipment에서 다른 Equipment로 운반하는 명령"""

    def __init__(self, commandID, jobID, fromEquipment, toEquipment, globalVar):
        """
        Args:
            fromEquipment: 출발 장비 객체
            toEquipment: 목표 장비 객체
        """
        # 출발 장비의 InputPort로 진입 (픽업을 위해)
        # AMR: FROM_IN → (내부) → FROM_OUT에서 픽업하고 진출
        fromLocation = {
            'nodeID': fromEquipment.inputPort['nodeID'] if fromEquipment.inputPort else fromEquipment.workPosition['nodeID'],
            'position': fromEquipment.inputPort['position'] if fromEquipment.inputPort else fromEquipment.workPosition['position'],
            'equipmentID': fromEquipment.strEquipmentID
        }

        # 목표 장비의 InputPort로 진입 (전달을 위해)
        # AMR: TO_IN → (하역) → TO_OUT으로 진출
        toLocation = {
            'nodeID': toEquipment.inputPort['nodeID'] if toEquipment.inputPort else toEquipment.workPosition['nodeID'],
            'position': toEquipment.inputPort['position'] if toEquipment.inputPort else toEquipment.workPosition['position'],
            'equipmentID': toEquipment.strEquipmentID
        }

        super().__init__(commandID, jobID, fromLocation, toLocation, globalVar)
        self.commandType = "EQUIPMENT_TO_EQUIPMENT"


class EquipmentToSinkCommand(TransportCommand):
    """Equipment에서 Sink로 운반하는 명령 (최종 완료)"""

    def __init__(self, commandID, jobID, fromEquipment, sinkEquipment, globalVar):
        """
        Args:
            fromEquipment: 출발 장비 객체
            sinkEquipment: Sink 장비 객체
        """
        # 출발 장비의 InputPort로 진입 (픽업을 위해)
        # AMR: FROM_IN → (내부) → FROM_OUT에서 픽업하고 진출
        fromLocation = {
            'nodeID': fromEquipment.inputPort['nodeID'] if fromEquipment.inputPort else fromEquipment.workPosition['nodeID'],
            'position': fromEquipment.inputPort['position'] if fromEquipment.inputPort else fromEquipment.workPosition['position'],
            'equipmentID': fromEquipment.strEquipmentID
        }

        # Sink의 InputPort로 진입 (전달을 위해)
        # AMR: SINK_IN → (하역) → SINK_OUT으로 진출
        toLocation = {
            'nodeID': sinkEquipment.inputPort['nodeID'] if sinkEquipment.inputPort else sinkEquipment.workPosition['nodeID'],
            'position': sinkEquipment.inputPort['position'] if sinkEquipment.inputPort else sinkEquipment.workPosition['position'],
            'equipmentID': sinkEquipment.strEquipmentID
        }

        super().__init__(commandID, jobID, fromLocation, toLocation, globalVar)
        self.commandType = "EQUIPMENT_TO_SINK"


class TransportCommandManager:
    """운반 명령 관리자"""

    def __init__(self, globalVar):
        self.globalVar = globalVar
        self.commands = {}  # commandID -> TransportCommand
        self.nextCommandID = 1

    def createCommand(self, jobID, fromEquipment, toEquipment):
        """
        운반 명령 생성

        Args:
            jobID: 작업 ID
            fromEquipment: 출발 장비 객체
            toEquipment: 목표 장비 객체

        Returns:
            TransportCommand 객체
        """
        commandID = f"CMD_{self.nextCommandID:06d}"
        self.nextCommandID += 1

        # 명령 타입에 따라 적절한 클래스 선택
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
        """명령 조회"""
        return self.commands.get(commandID, None)

    def getPendingCommands(self):
        """대기 중인 명령 리스트"""
        return [cmd for cmd in self.commands.values() if cmd.isPending()]

    def getActiveCommands(self):
        """실행 중인 명령 리스트"""
        return [cmd for cmd in self.commands.values() if cmd.isAssigned()]

    def getCommandsByAMR(self, amrID):
        """특정 AMR에 할당된 명령 리스트"""
        return [cmd for cmd in self.commands.values() if cmd.assignedAMR == amrID]

    def getCommandsByJob(self, jobID):
        """특정 Job 관련 명령 리스트"""
        return [cmd for cmd in self.commands.values() if cmd.jobID == jobID]

    def removeCommand(self, commandID):
        """명령 제거"""
        if commandID in self.commands:
            del self.commands[commandID]
