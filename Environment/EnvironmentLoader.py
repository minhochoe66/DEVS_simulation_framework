from SimulationEngine.Utility.Configurator import Configurator
import json


class EnvironmentLoader:
    def __init__(self, strPath, lstFileNames):
        objConfiguration = Configurator()

        for fName in lstFileNames:
            strFileName = strPath + fName + ".json"
            print(f"Loading {strFileName}")

            with open(strFileName, 'r', encoding='utf-8') as json_file:
                objData = json.load(json_file)

                if objData["fileName"] == "map":
                    # 장비 정보 - numStages에 따라 필터링
                    numStages = objConfiguration.getConfiguration("numStages")
                    if numStages is None:
                        numStages = 3  # 기본값: 모든 스테이지 포함

                    # 스테이지 필터링
                    filteredEquipmentInfo = self.filterEquipmentByStages(
                        objData["equipmentInfo"], numStages)

                    objConfiguration.addConfiguration(
                        "equipmentInfo", filteredEquipmentInfo)

                    print(
                        f"Filtered equipment count: {len(filteredEquipmentInfo)} (numStages={numStages})")

                    # 대기 구역 정보 - numStages에 따라 필터링 및 위치 조정
                    if "WatingareaInfo" in objData:
                        filteredWaitingAreas = self.filterWaitingAreasByStages(
                            objData["WatingareaInfo"], numStages)
                        objConfiguration.addConfiguration(
                            "watingAreaInfo", filteredWaitingAreas)
                        print(
                            f"Filtered waiting areas: {len(objData['WatingareaInfo'])} -> {len(filteredWaitingAreas)}")

                    # 필터링된 맵 정보를 새 JSON 파일로 저장
                    self.saveFilteredMap(strPath, filteredEquipmentInfo,
                                         filteredWaitingAreas if "WatingareaInfo" in objData else [],
                                         numStages)

                elif objData["fileName"] == "processInfo":
                    # 공정 시퀀스 및 성능 정보 - numStages에 따라 필터링
                    numStages = objConfiguration.getConfiguration("numStages")
                    if numStages is None:
                        numStages = 3  # 기본값

                    # 시퀀스 필터링
                    filteredSeqInfo = self.filterSequenceByStages(
                        objData["seqInfo"], numStages)

                    objConfiguration.addConfiguration(
                        "seqInfo", filteredSeqInfo)
                    objConfiguration.addConfiguration(
                        "performanceInfo", objData["performanceInfo"])

                elif objData["fileName"] == "vehicleInfo":
                    # AMR 정보 및 파라미터
                    objConfiguration.addConfiguration(
                        "vehicleInfo", objData["vehicleInfo"])
                    objConfiguration.addConfiguration(
                        "vehicleParam", objData["vehicleParam"])

                    # vehicle 파라미터를 개별 키로도 추가 (각 모델에서 쉽게 접근)
                    for key, value in objData["vehicleParam"].items():
                        objConfiguration.addConfiguration(key, value)

                elif objData["fileName"] == "setup":
                    # 시뮬레이션 설정
                    objConfiguration.addConfiguration(
                        "numVehicles", objData["numVehicles"])
                    objConfiguration.addConfiguration(
                        "numJob", objData["numJob"])
                    objConfiguration.addConfiguration(
                        "numStages", objData.get("numStages", 3))  # 중간 스테이지 수
                    objConfiguration.addConfiguration(
                        "isTerminalOn", objData["isTerminalOn"])
                    objConfiguration.addConfiguration(
                        "isVisualizerOn", objData["isVisualizerOn"])
                    objConfiguration.addConfiguration(
                        "renderTime", objData["renderTime"])
                    objConfiguration.addConfiguration(
                        "monteCarlo", objData.get("monteCarlo", 1))  # 몬테카를로 반복 횟수
                    objConfiguration.addConfiguration(
                        "Vehiclechange", objData.get("Vehiclechange", False))  # 차량 수 변경 모드

                else:
                    print(
                        f"Warning: Unknown file type '{objData['fileName']}'")

        self.objConfiguration = objConfiguration

    def filterEquipmentByStages(self, equipmentInfo, numStages):
        """
        numStages에 따라 중간 공정 스테이지를 필터링하고 위치 조정

        Args:
            equipmentInfo: 전체 장비 정보 리스트
            numStages: 중간 스테이지 수 (1, 2, 3)
                - 1: A → B → OUT
                - 2: A → B → C → OUT
                - 3: A → B → C → D → OUT

        Returns:
            필터링 및 위치 조정된 장비 정보 리스트
        """
        import copy

        # 스테이지별 매핑 (B=1, C=2, D=3)
        stageMapping = {
            "STAGE_B": 1,
            "STAGE_C": 2,
            "STAGE_D": 3
        }

        filtered = []
        for equipment in equipmentInfo:
            eq_copy = copy.deepcopy(equipment)
            stageID = eq_copy.get("stageID", "")
            equipmentID = eq_copy.get("equipmentID", "")

            # A는 항상 포함 (위치 고정)
            if stageID == "STAGE_A":
                filtered.append(eq_copy)

            # OUT은 항상 포함하되 위치 조정
            elif stageID == "STAGE_OUT":
                # OUT 위치 = 70 + (numStages * 60)
                # numStages=1: x=130, numStages=2: x=190, numStages=3: x=250
                out_x_base = 70 + (numStages * 60)

                if eq_copy.get("inputPort"):
                    eq_copy["inputPort"]["position"]["x"] = out_x_base
                if eq_copy.get("outputPort"):
                    eq_copy["outputPort"]["position"]["x"] = out_x_base + 10
                if eq_copy.get("workPosition"):
                    eq_copy["workPosition"]["position"]["x"] = out_x_base + 5

                filtered.append(eq_copy)

            # WAITING_AREA는 해당 스테이지가 포함된 경우만
            elif eq_copy.get("processType") == "WAITING_AREA":
                should_include = False

                if equipmentID.startswith("WAIT_B") and numStages >= 1:
                    should_include = True
                elif equipmentID.startswith("WAIT_C") and numStages >= 2:
                    should_include = True
                elif equipmentID.startswith("WAIT_D") and numStages >= 3:
                    should_include = True
                elif equipmentID.startswith("WAIT_OUT"):
                    should_include = True
                    # WAIT_OUT 위치 = OUT_OUT + 20
                    out_x_base = 70 + (numStages * 60)
                    if eq_copy.get("workPosition"):
                        eq_copy["workPosition"]["position"]["x"] = out_x_base + 10 + 20

                if should_include:
                    filtered.append(eq_copy)

            # 중간 공정 스테이지 필터링
            elif stageID in stageMapping:
                if stageMapping[stageID] <= numStages:
                    filtered.append(eq_copy)

        print(f"Equipment filtered: {len(equipmentInfo)} -> {len(filtered)}")
        print(f"  OUT position adjusted to x={70 + (numStages * 60)}")
        return filtered

    def filterWaitingAreasByStages(self, waitingAreaInfo, numStages):
        """
        numStages에 따라 WaitingArea를 필터링하고 위치 조정

        Args:
            waitingAreaInfo: 전체 대기 구역 정보 리스트
            numStages: 중간 스테이지 수 (1, 2, 3)

        Returns:
            필터링 및 위치 조정된 대기 구역 정보 리스트
        """
        import copy

        # 스테이지별 매핑
        areaStageMapping = {
            "WAITING_AREA_A": 0,      # 항상 포함
            "WAITING_AREA_B": 1,
            "WAITING_AREA_C": 2,
            "WAITING_AREA_D": 3,
            "WAITING_AREA_OUT": 0     # 항상 포함 (위치만 조정)
        }

        filtered = []
        for area in waitingAreaInfo:
            area_copy = copy.deepcopy(area)
            areaID = area_copy.get("areaID", "")

            # 스테이지 확인 (areaID의 prefix로 판단)
            area_stage = None
            for prefix, stage in areaStageMapping.items():
                if areaID.startswith(prefix):
                    area_stage = stage
                    break

            # 필터링: 해당 스테이지가 포함되는지 확인
            if area_stage is not None:
                # WAITING_AREA_A는 항상 포함
                if areaID.startswith("WAITING_AREA_A"):
                    filtered.append(area_copy)

                # WAITING_AREA_OUT은 항상 포함하되 위치 조정
                elif areaID.startswith("WAITING_AREA_OUT"):
                    # OUT_OUT 위치 + 15
                    out_x_base = 70 + (numStages * 60)
                    area_copy["position"]["x"] = out_x_base + \
                        10 + 15  # OUT_OUT(+10) + 거리(+15)
                    filtered.append(area_copy)

                # B, C, D는 numStages에 따라 필터링
                elif area_stage <= numStages:
                    filtered.append(area_copy)

        print(
            f"  WaitingArea filtered: {len(waitingAreaInfo)} -> {len(filtered)}")
        print(
            f"  WAITING_AREA_OUT position adjusted to x={70 + (numStages * 60) + 25}")
        return filtered

    def filterSequenceByStages(self, seqInfo, numStages):
        """
        numStages에 따라 공정 시퀀스를 필터링

        Args:
            seqInfo: 공정 시퀀스 정보 리스트
            numStages: 중간 스테이지 수

        Returns:
            필터링된 시퀀스 정보
        """
        # 스테이지별 프로세스 타입 매핑
        processStageMapping = {
            "PROCESS_B1": 1, "PROCESS_B2": 1, "PROCESS_B3": 1,
            "PROCESS_C1": 2, "PROCESS_C2": 2, "PROCESS_C3": 2,
            "PROCESS_D1": 3, "PROCESS_D2": 3, "PROCESS_D3": 3
        }

        filtered = []
        for seq in seqInfo:
            filteredSequence = []
            for processType in seq["sequenceList"]:
                # SOURCE와 SINK는 항상 포함
                if processType == "SOURCE" or processType == "SINK":
                    filteredSequence.append(processType)
                # 중간 공정은 numStages에 따라 필터링
                elif processType in processStageMapping:
                    if processStageMapping[processType] <= numStages:
                        filteredSequence.append(processType)

            # 필터링된 시퀀스 저장
            filtered.append({
                "seqNum": seq["seqNum"],
                "sequenceList": filteredSequence
            })

        print(
            f"Sequence filtered: {seqInfo[0]['sequenceList']} -> {filtered[0]['sequenceList']}")
        return filtered

    def saveFilteredMap(self, strPath, filteredEquipmentInfo, filteredWaitingAreas, numStages):
        """
        필터링된 맵 정보를 새로운 JSON 파일로 저장

        Args:
            strPath: JSON 파일 경로
            filteredEquipmentInfo: 필터링된 장비 정보
            filteredWaitingAreas: 필터링된 대기 구역 정보
            numStages: 중간 스테이지 수
        """
        import os
        from datetime import datetime

        # 필터링된 맵 데이터 구성
        filtered_map_data = {
            "fileName": "map",
            "equipmentInfo": filteredEquipmentInfo,
            "WatingareaInfo": filteredWaitingAreas
        }

        # Visualizations 폴더 내에 최신 시뮬레이션 폴더 찾기
        viz_path = os.path.join(os.path.dirname(strPath), "Visualizations")
        if os.path.exists(viz_path):
            # 최신 폴더 찾기 (날짜시간 형식으로 정렬)
            sim_folders = [f for f in os.listdir(
                viz_path) if os.path.isdir(os.path.join(viz_path, f))]
            if sim_folders:
                latest_folder = sorted(sim_folders)[-1]
                iteration_path = os.path.join(
                    viz_path, latest_folder, "iteration_1")

                # iteration_1 폴더가 없으면 생성
                if not os.path.exists(iteration_path):
                    os.makedirs(iteration_path)

                # 필터링된 map.json 저장
                filtered_map_path = os.path.join(iteration_path, "map.json")
                with open(filtered_map_path, 'w', encoding='utf-8') as f:
                    json.dump(filtered_map_data, f,
                              indent=4, ensure_ascii=False)

                print(f"  Filtered map saved to: {filtered_map_path}")
                print(
                    f"  (numStages={numStages}, Equipment={len(filteredEquipmentInfo)}, WaitingAreas={len(filteredWaitingAreas)})")

    def getConfiguration(self):
        return self.objConfiguration
