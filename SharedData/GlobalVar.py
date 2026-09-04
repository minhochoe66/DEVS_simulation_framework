import copy
import json


class GlobalVar:
    def __init__(self, isTerminalOn, equipmentInfo, seqInfo, performanceInfo, vehicleInfo, numJob, numVehicle, objConfiguration=None):
        self.equipmentInfo = {}
        self.sequenceInfo = {}
        self.vehicleInfo = {}
        self.targetJobs = {}
        self.performanceInfo = {}
        self.obstacleInfo = []  # BoundingBox 기반 정적 장애물
        self.watingAreaInfo = {}  # WaitingArea 정보
        self.isTerminalOn = isTerminalOn
        self.initVehicleInfo = {}
        self.objConfiguration = objConfiguration  # Configuration 객체 저장
        self.GlobalPlanner_algorithm_time = 0
        self.LocalPlanner_algorithm_time = 0
        self.GlobalPlanner_call_count = 0
        self.LocalPlanner_call_count = 0
        self.setEquipmentInfo(equipmentInfo)
        self.setSequenceInfo(seqInfo)
        self.setPerformanceInfo(performanceInfo)
        self.setVehicleInfo(vehicleInfo, numVehicle)
        self.setObstacleInfo()
        self.setTargetJobs(numJob, 1)
        self.setWaitingAreaInfo()

    def getGlobalPlanner_algorithm_time(self):
        return self.GlobalPlanner_algorithm_time

    def getLocalPlanner_algorithm_time(self):
        return self.LocalPlanner_algorithm_time

    def print_algorithm_statistics(self):
        """알고리즘 실행 시간 통계 출력"""
        print("\n" + "="*70)
        print("📊 ALGORITHM PERFORMANCE STATISTICS")
        print("="*70)

        # Global Planner 통계
        global_total = self.GlobalPlanner_algorithm_time
        global_count = self.GlobalPlanner_call_count
        global_avg = global_total / global_count if global_count > 0 else 0

        print(f"\n🌐 Global Planner:")
        print(f"   Total Time:     {global_total:.3f} seconds")
        print(f"   Call Count:     {global_count}")
        print(f"   Average Time:   {global_avg:.3f} seconds/call")

        # Local Planner 통계
        local_total = self.LocalPlanner_algorithm_time
        local_count = self.LocalPlanner_call_count
        local_avg = local_total / local_count if local_count > 0 else 0

        print(f"\n📍 Local Planner (DWA):")
        print(f"   Total Time:     {local_total:.3f} seconds")
        print(f"   Call Count:     {local_count}")
        print(f"   Average Time:   {local_avg:.4f} seconds/call")

        # 전체 통계
        total_time = global_total + local_total
        total_count = global_count + local_count

        print(f"\n🔧 Total Algorithm Time:")
        print(f"   Combined:       {total_time:.3f} seconds")
        print(f"   Total Calls:    {total_count}")

        # 비율
        if total_time > 0:
            global_percent = (global_total / total_time) * 100
            local_percent = (local_total / total_time) * 100
            print(f"\n📈 Time Distribution:")
            print(f"   Global Planner: {global_percent:.1f}%")
            print(f"   Local Planner:  {local_percent:.1f}%")

        print("="*70 + "\n")

    ## generate jobs ##
    def setTargetJobs(self, targetNum, seqNum):
        jobID = 1
        for i in range(targetNum):
            self.targetJobs[jobID] = Job(
                jobID, seqNum, self.getSequenceInfoBySeqNum(seqNum).lstSequence)
            jobID += 1

    def getTargetJobs(self):
        return self.targetJobs

    def getTargetJobsByID(self, ID):
        return self.targetJobs[int(ID)]

    ## function for Vehicle information ##
    def setVehicleInfo(self, Info, numVehicle):
        for i in range(numVehicle):
            self.vehicleInfo[Info[i]["vehicleID"]] = Vehicle(
                Info[i]["vehicleID"], Info[i]["coordinates"])
            self.initVehicleInfo[Info[i]["vehicleID"]] = copy.deepcopy(
                self.vehicleInfo[Info[i]["vehicleID"]])

    def getVehicleInfo(self):
        return self.vehicleInfo

    def getInitVehicleInfo(self):
        return self.initVehicleInfo

    def getVehicleInfoByID(self, ID):
        return self.vehicleInfo[str(ID)]

    ## function for sequence information ##
    def setSequenceInfo(self, Info):
        for i in range(len(Info)):
            self.sequenceInfo[Info[i]["seqNum"]] = Sequence(
                Info[i]["seqNum"], Info[i]["sequenceList"])

    def getSequenceInfo(self):
        return self.sequenceInfo

    def getSequenceInfoBySeqNum(self, seqNum):
        return self.sequenceInfo[int(seqNum)]

    ## function for equipment information ##
    def setEquipmentInfo(self, Info):
        for i in range(len(Info)):
            equipmentID = Info[i]["equipmentID"]
            inputPort = Info[i].get("inputPort", None)
            outputPort = Info[i].get("outputPort", None)
            workPosition = Info[i]["workPosition"]
            processTime = Info[i]["processTime"]
            processType = Info[i]["processType"]
            performance = Info[i]["performance"]
            stageID = Info[i]["stageID"]
            exchangeTime = Info[i]["exchangeTime"]

            self.equipmentInfo[equipmentID] = Equipment(
                equipmentID, inputPort, outputPort, workPosition,
                processTime, processType, performance, stageID, exchangeTime
            )

    def getEquipmentInfo(self):
        return self.equipmentInfo

    def getEquipmentInfoByID(self, ID):
        return self.equipmentInfo[str(ID)]

    def getEquipmentInfoByStageID(self, stageID):
        result = []
        for key, value in self.equipmentInfo.items():
            if value.strStageID == stageID:
                result.append(value)
        return result

    def getEquipmentInfoByProcessType(self, processType):
        for key, value in self.equipmentInfo.items():
            if value.strType == processType:
                return value
        return None

    ## function for obstacle information (BoundingBox) ##
    def setObstacleInfo(self):
        """모든 Equipment의 BoundingBox를 장애물로 등록"""
        for key, value in self.equipmentInfo.items():
            # Input Port BoundingBox
            if value.inputPort is not None:
                self.obstacleInfo.append({
                    'type': 'inputPort',
                    'equipmentID': key,
                    'nodeID': value.inputPort['nodeID'],
                    'position': value.inputPort['position'],
                    'boundingBox': value.inputPort['boundingBox']
                })

            # Output Port BoundingBox
            if value.outputPort is not None:
                self.obstacleInfo.append({
                    'type': 'outputPort',
                    'equipmentID': key,
                    'nodeID': value.outputPort['nodeID'],
                    'position': value.outputPort['position'],
                    'boundingBox': value.outputPort['boundingBox']
                })

            # Work Position BoundingBox
            self.obstacleInfo.append({
                'type': 'workPosition',
                'equipmentID': key,
                'nodeID': value.workPosition['nodeID'],
                'position': value.workPosition['position'],
                'boundingBox': value.workPosition['boundingBox']
            })

    def getObstacleInfo(self):
        return self.obstacleInfo

    ## function for waiting area information ##
    def setWaitingAreaInfo(self):
        """Configuration에서 WaitingArea 정보 로드"""
        if self.objConfiguration:
            watingAreaList = self.objConfiguration.getConfiguration(
                "watingAreaInfo")
            if watingAreaList:
                for area in watingAreaList:
                    areaID = area["areaID"]
                    self.watingAreaInfo[areaID] = WatingArea(
                        areaID,
                        area["position"],
                        area["boundingBox"]
                    )
                self.printTerminal(
                    f"[GlobalVar] Loaded {len(self.watingAreaInfo)} waiting areas")

    def getWaitingAreaInfo(self):
        return self.watingAreaInfo

    def getWaitingAreaInfoByID(self, ID):
        return self.watingAreaInfo.get(str(ID), None)

    def getClosestWaitingArea(self, position):
        """가장 가까운 WaitingArea 찾기 (점유 상태 무시 - 여러 AMR 동시 사용 가능)"""
        import math
        min_distance = float('inf')
        closest_area = None

        for areaID, area in self.watingAreaInfo.items():
            # 점유 상태 체크 제거 - 여러 AMR이 같은 WaitingArea 사용 가능

            area_pos = area.position
            distance = math.sqrt(
                (position[0] - area_pos['x'])**2 +
                (position[1] - area_pos['y'])**2
            )

            if distance < min_distance:
                min_distance = distance
                closest_area = area

        return closest_area

    ## function for performance information ##
    def setPerformanceInfo(self, Info):
        performanceValue = Info[0]
        for key, value in self.equipmentInfo.items():
            value.intPerformanceValue = performanceValue[value.strPerformance]

    ## function for print ##
    def printTerminal(self, log):
        if self.isTerminalOn == True:
            print(log)


class Sequence:
    def __init__(self, seqNum, sequenceList):
        self.intSeqNum = int(seqNum)
        self.lstSequence = sequenceList


class WatingArea:
    def __init__(self, areaID, position, boundingBox):
        self.strAreaID = str(areaID)
        self.position = position
        self.boundingBox = boundingBox
        # Equipment의 undockedAMRs와 동일하게 set으로 관리
        self.occupiedAMRs = set()  # WaitingArea에 있는 AMR ID들


class Equipment:
    def __init__(self, equipmentID, inputPort, outputPort, workPosition, processTime, processType, performance, stageID, exchangeTime):
        self.strEquipmentID = str(equipmentID)
        self.strStageID = str(stageID)

        # 입력 포트 (AMR이 작업 투입)
        # {'nodeID': 'B-1_IN', 'position': {...}, 'boundingBox': {...}}
        self.inputPort = inputPort

        # 출력 포트 (AMR이 작업 회수)
        # {'nodeID': 'B-1_OUT', 'position': {...}, 'boundingBox': {...}}
        self.outputPort = outputPort

        # 작업 위치 (장비 본체)
        # {'nodeID': 'B-1_WORK', 'position': {...}, 'boundingBox': {...}}
        self.workPosition = workPosition

        self.dblProcessTime = float(processTime)
        self.dblExchangeTime = float(exchangeTime)
        self.strType = str(processType)
        self.strState = "EMPTY"  # EMPTY, RESERVED, BUSY, DONE
        self.intProcessingJobID = None
        self.strPerformance = performance
        self.intPerformanceValue = None
        self.lstProcessingJobID = []
        self.totalProcessedTime = 0

        # UNDOCKING된 AMR ID 저장 (set)
        self.undockedAMRs = set()

    def setEquipmentState(self, state):
        self.strState = state

    def setProcessingJobID(self, ID):
        self.intProcessingJobID = int(ID)
        self.lstProcessingJobID.append(int(ID))


class Job:
    def __init__(self, jobID, seqNum, lstInitProcess):
        self.intJobID = int(jobID)
        self.intSeqNum = seqNum
        self.lstInitProcess = lstInitProcess
        self.lstFinalProcess = copy.deepcopy(lstInitProcess)  # deepcopy ?
        self.dictStartTime = {}
        self.dictStartCarryObject = {}
        self.dictDoneTime = {}
        self.dictOutTime = {}
        self.dictOutCarryObject = {}
        self.dictYieldScore = {}
        self.dblYieldScore = 0
        self.lstCommandID = []

        self.strCurrentProcess = None
        self.strCurrentProcessEqpID = None
        self.strNextProcess = None
        self.strNextProcessEqpID = None

    def setTime(self, state, equipmentID, time):
        if state == "start":
            self.dictStartTime[equipmentID] = time
        elif state == "done":
            self.dictDoneTime[equipmentID] = time
        elif state == "out":
            self.dictOutTime[equipmentID] = time
        else:
            print("Wrong input 'state', must be ['start' or 'done' or 'out]")

    def setCarryObject(self, port, equipmentID, objID):
        if port == 'in':
            self.dictStartCarryObject[equipmentID] = objID
        elif port == 'out':
            self.dictOutCarryObject[equipmentID] = objID
        else:
            print("Wrong input 'port', must be ['in' or 'out']")

    def setScore(self, score, eqpID):
        self.dblYieldScore = score
        self.dictYieldScore[eqpID] = score

    def setCurrentProcess(self, processType, equipmentID):
        self.strCurrentProcess = processType
        self.strCurrentProcessEqpID = equipmentID

    def setNextProcess(self, processType, equipmentID):
        self.strNextProcess = processType
        self.strNextProcessEqpID = equipmentID

    def setCommandID(self, commandID):
        if commandID not in self.lstCommandID:
            self.lstCommandID.append(commandID)


class Vehicle:
    def __init__(self, vehicleID, coordinates):
        self.strVehicleID = str(vehicleID)
        self.lstCoordinates = coordinates  # [x, y] 현재 위치
        # IDLE, RESERVED, MOVE, ARRIVAL, LIFTDOWN, LOAD, LIFTUP, FROMDONE, UNLOAD
        self.strState = "IDLE"
        self.intJobID = None
        self.lstJobID = []
        self.strCommandID = None
        self.lstCommandID = []
        self.dictActivationTime = {}
        self.doneWaitStartTime = None
        # Docking linkage
        self.currentEquipmentID = None  # 도킹된 장비 ID
        self.dblYaw = None

    def setState(self, state):
        old_state = self.strState
        self.strState = state
        if old_state != state:
            print(f"🔄 [AMR_STATE] {self.strVehicleID}: {old_state} → {state}")

    def setCoordinates(self, coorinates):
        # 디버깅: 좌표 변경 추적
        import traceback
        import inspect

        # 호출자 정보 가져오기
        stack = traceback.extract_stack()
        caller_info = stack[-2] if len(stack) >= 2 else None
        caller_file = caller_info.filename.split(
            '\\')[-1] if caller_info else "Unknown"
        caller_line = caller_info.lineno if caller_info else "?"
        caller_func = caller_info.name if caller_info else "Unknown"

        self.lstCoordinates = coorinates

    def getstate(self):
        return self.strState

    def setYaw(self, yaw):
        self.dblYaw = yaw

    def getYaw(self):
        return self.dblYaw

    def getCoordinates(self):
        return self.lstCoordinates

    def setJobID(self, jobID):
        self.intJobID = jobID
        self.lstJobID.append(jobID)

    def setCommandID(self, commandID):
        self.strCommandID = commandID
        if commandID not in self.lstCommandID:
            self.lstCommandID.append(commandID)

    def setActivationTime(self, commandID, time):
        if commandID not in self.dictActivationTime:
            self.dictActivationTime[commandID] = time
        else:
            self.dictActivationTime[commandID] = self.dictActivationTime[commandID] + time

    # --- Docking linkage helpers ---
    def setEquipmentID(self, equipmentID):
        self.currentEquipmentID = str(
            equipmentID) if equipmentID is not None else None

    def getEquipmentID(self):
        return self.currentEquipmentID
