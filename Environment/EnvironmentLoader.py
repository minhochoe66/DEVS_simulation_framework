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
                    # 장비 정보. numStages에 따라 필터링한다
                    numStages = objConfiguration.getConfiguration("numStages")
                    if numStages is None:
                        numStages = 3  # 기본값은 모든 스테이지

                    filteredEquipmentInfo = self.filterEquipmentByStages(
                        objData["equipmentInfo"], numStages)

                    objConfiguration.addConfiguration(
                        "equipmentInfo", filteredEquipmentInfo)

                    print(
                        f"Filtered equipment count: {len(filteredEquipmentInfo)} (numStages={numStages})")

                    # 대기 구역도 같은 기준으로 걸러내고 위치를 조정한다
                    if "WatingareaInfo" in objData:
                        filteredWaitingAreas = self.filterWaitingAreasByStages(
                            objData["WatingareaInfo"], numStages)
                        objConfiguration.addConfiguration(
                            "watingAreaInfo", filteredWaitingAreas)
                        print(
                            f"Filtered waiting areas: {len(objData['WatingareaInfo'])} -> {len(filteredWaitingAreas)}")

                    # 실제로 사용한 레이아웃을 결과 폴더에 기록한다
                    self.saveFilteredMap(strPath, filteredEquipmentInfo,
                                         filteredWaitingAreas if "WatingareaInfo" in objData else [],
                                         numStages)

                elif objData["fileName"] == "processInfo":
                    # 공정 시퀀스와 성능 정보
                    numStages = objConfiguration.getConfiguration("numStages")
                    if numStages is None:
                        numStages = 3  # 기본값

                    filteredSeqInfo = self.filterSequenceByStages(
                        objData["seqInfo"], numStages)

                    objConfiguration.addConfiguration(
                        "seqInfo", filteredSeqInfo)
                    objConfiguration.addConfiguration(
                        "performanceInfo", objData["performanceInfo"])

                elif objData["fileName"] == "vehicleInfo":
                    # AMR 정보와 주행 파라미터
                    objConfiguration.addConfiguration(
                        "vehicleInfo", objData["vehicleInfo"])
                    objConfiguration.addConfiguration(
                        "vehicleParam", objData["vehicleParam"])

                    # 각 파라미터를 최상위 키로도 등록해 모델에서 바로 읽게 한다
                    for key, value in objData["vehicleParam"].items():
                        objConfiguration.addConfiguration(key, value)

                elif objData["fileName"] == "setup":
                    # 실행 설정
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
        """numStages에 맞추어 중간 공정 장비를 걸러내고 SINK 위치를 옮긴다.

        numStages는 1, 2, 3 중 하나이며 각각 A-B-OUT, A-B-C-OUT, A-B-C-D-OUT에
        해당한다. SINK의 x 좌표는 스테이지 수에 따라 70 + numStages * 60으로 정한다.
        """
        import copy

        # 스테이지 순번 (B=1, C=2, D=3)
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

            # A는 항상 포함하고 위치도 고정
            if stageID == "STAGE_A":
                filtered.append(eq_copy)

            # OUT도 항상 포함하되 위치를 옮긴다
            elif stageID == "STAGE_OUT":
                # numStages 1, 2, 3 -> x = 130, 190, 250
                out_x_base = 70 + (numStages * 60)

                if eq_copy.get("inputPort"):
                    eq_copy["inputPort"]["position"]["x"] = out_x_base
                if eq_copy.get("outputPort"):
                    eq_copy["outputPort"]["position"]["x"] = out_x_base + 10
                if eq_copy.get("workPosition"):
                    eq_copy["workPosition"]["position"]["x"] = out_x_base + 5

                filtered.append(eq_copy)

            # 대기 구역은 해당 스테이지가 살아 있을 때만 포함
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
                    # WAIT_OUT은 OUT_OUT보다 20만큼 뒤
                    out_x_base = 70 + (numStages * 60)
                    if eq_copy.get("workPosition"):
                        eq_copy["workPosition"]["position"]["x"] = out_x_base + 10 + 20

                if should_include:
                    filtered.append(eq_copy)

            # 중간 공정 스테이지
            elif stageID in stageMapping:
                if stageMapping[stageID] <= numStages:
                    filtered.append(eq_copy)

        print(f"Equipment filtered: {len(equipmentInfo)} -> {len(filtered)}")
        print(f"  OUT position adjusted to x={70 + (numStages * 60)}")
        return filtered

    def filterWaitingAreasByStages(self, waitingAreaInfo, numStages):
        """numStages에 맞추어 대기 구역을 걸러내고 위치를 조정한다."""
        import copy

        # 대기 구역이 속한 스테이지
        areaStageMapping = {
            "WAITING_AREA_A": 0,      # 항상 포함
            "WAITING_AREA_B": 1,
            "WAITING_AREA_C": 2,
            "WAITING_AREA_D": 3,
            "WAITING_AREA_OUT": 0     # 항상 포함, 위치만 조정
        }

        filtered = []
        for area in waitingAreaInfo:
            area_copy = copy.deepcopy(area)
            areaID = area_copy.get("areaID", "")

            # areaID 접두어로 스테이지를 판별한다
            area_stage = None
            for prefix, stage in areaStageMapping.items():
                if areaID.startswith(prefix):
                    area_stage = stage
                    break

            if area_stage is not None:
                # A 대기 구역은 항상 포함
                if areaID.startswith("WAITING_AREA_A"):
                    filtered.append(area_copy)

                # OUT 대기 구역도 항상 포함하되 위치를 옮긴다
                elif areaID.startswith("WAITING_AREA_OUT"):
                    out_x_base = 70 + (numStages * 60)
                    area_copy["position"]["x"] = out_x_base + \
                        10 + 15  # OUT_OUT(+10)에서 15만큼 더
                    filtered.append(area_copy)

                # B, C, D는 numStages로 거른다
                elif area_stage <= numStages:
                    filtered.append(area_copy)

        print(
            f"  WaitingArea filtered: {len(waitingAreaInfo)} -> {len(filtered)}")
        print(
            f"  WAITING_AREA_OUT position adjusted to x={70 + (numStages * 60) + 25}")
        return filtered

    def filterSequenceByStages(self, seqInfo, numStages):
        """numStages에 맞추어 공정 시퀀스를 잘라낸다."""
        # 공정 타입이 속한 스테이지
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
                # 중간 공정은 numStages로 거른다
                elif processType in processStageMapping:
                    if processStageMapping[processType] <= numStages:
                        filteredSequence.append(processType)

            filtered.append({
                "seqNum": seq["seqNum"],
                "sequenceList": filteredSequence
            })

        print(
            f"Sequence filtered: {seqInfo[0]['sequenceList']} -> {filtered[0]['sequenceList']}")
        return filtered

    def saveFilteredMap(self, strPath, filteredEquipmentInfo, filteredWaitingAreas, numStages):
        """필터링된 레이아웃을 가장 최근 결과 폴더에 map.json으로 저장한다."""
        import os
        from datetime import datetime

        filtered_map_data = {
            "fileName": "map",
            "equipmentInfo": filteredEquipmentInfo,
            "WatingareaInfo": filteredWaitingAreas
        }

        # Visualizations 아래에서 가장 최근 실행 폴더를 찾는다
        viz_path = os.path.join(os.path.dirname(strPath), "Visualizations")
        if os.path.exists(viz_path):
            # 폴더 이름이 타임스탬프이므로 정렬하면 마지막이 최신
            sim_folders = [f for f in os.listdir(
                viz_path) if os.path.isdir(os.path.join(viz_path, f))]
            if sim_folders:
                latest_folder = sorted(sim_folders)[-1]
                iteration_path = os.path.join(
                    viz_path, latest_folder, "iteration_1")

                if not os.path.exists(iteration_path):
                    os.makedirs(iteration_path)

                filtered_map_path = os.path.join(iteration_path, "map.json")
                with open(filtered_map_path, 'w', encoding='utf-8') as f:
                    json.dump(filtered_map_data, f,
                              indent=4, ensure_ascii=False)

                print(f"  Filtered map saved to: {filtered_map_path}")
                print(
                    f"  (numStages={numStages}, Equipment={len(filteredEquipmentInfo)}, WaitingAreas={len(filteredWaitingAreas)})")

    def getConfiguration(self):
        return self.objConfiguration
