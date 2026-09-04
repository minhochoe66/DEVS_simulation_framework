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
                    # Equipment, filtered by numStages
                    numStages = objConfiguration.getConfiguration("numStages")
                    if numStages is None:
                        numStages = 3  # default: every stage

                    filteredEquipmentInfo = self.filterEquipmentByStages(
                        objData["equipmentInfo"], numStages)

                    objConfiguration.addConfiguration(
                        "equipmentInfo", filteredEquipmentInfo)

                    print(
                        f"Filtered equipment count: {len(filteredEquipmentInfo)} (numStages={numStages})")

                    # Waiting areas, filtered and repositioned the same way
                    if "WatingareaInfo" in objData:
                        filteredWaitingAreas = self.filterWaitingAreasByStages(
                            objData["WatingareaInfo"], numStages)
                        objConfiguration.addConfiguration(
                            "watingAreaInfo", filteredWaitingAreas)
                        print(
                            f"Filtered waiting areas: {len(objData['WatingareaInfo'])} -> {len(filteredWaitingAreas)}")

                    # Record the layout actually used alongside the results
                    self.saveFilteredMap(strPath, filteredEquipmentInfo,
                                         filteredWaitingAreas if "WatingareaInfo" in objData else [],
                                         numStages)

                elif objData["fileName"] == "processInfo":
                    # Process sequences and machine performance factors
                    numStages = objConfiguration.getConfiguration("numStages")
                    if numStages is None:
                        numStages = 3  # default

                    filteredSeqInfo = self.filterSequenceByStages(
                        objData["seqInfo"], numStages)

                    objConfiguration.addConfiguration(
                        "seqInfo", filteredSeqInfo)
                    objConfiguration.addConfiguration(
                        "performanceInfo", objData["performanceInfo"])

                elif objData["fileName"] == "vehicleInfo":
                    # Robots and their motion parameters
                    objConfiguration.addConfiguration(
                        "vehicleInfo", objData["vehicleInfo"])
                    objConfiguration.addConfiguration(
                        "vehicleParam", objData["vehicleParam"])

                    # also promote each parameter to a top-level key, so models can read it directly
                    for key, value in objData["vehicleParam"].items():
                        objConfiguration.addConfiguration(key, value)

                elif objData["fileName"] == "setup":
                    # Run settings
                    objConfiguration.addConfiguration(
                        "numVehicles", objData["numVehicles"])
                    objConfiguration.addConfiguration(
                        "numJob", objData["numJob"])
                    objConfiguration.addConfiguration(
                        "numStages", objData.get("numStages", 3))  # number of intermediate stages
                    objConfiguration.addConfiguration(
                        "isTerminalOn", objData["isTerminalOn"])
                    objConfiguration.addConfiguration(
                        "isVisualizerOn", objData["isVisualizerOn"])
                    objConfiguration.addConfiguration(
                        "renderTime", objData["renderTime"])
                    objConfiguration.addConfiguration(
                        "monteCarlo", objData.get("monteCarlo", 1))  # Monte Carlo replications
                    objConfiguration.addConfiguration(
                        "Vehiclechange", objData.get("Vehiclechange", False))  # fleet-size sweep mode

                else:
                    print(
                        f"Warning: Unknown file type '{objData['fileName']}'")

        self.objConfiguration = objConfiguration

    def filterEquipmentByStages(self, equipmentInfo, numStages):
        """Filter the intermediate stages by numStages and shift the SINK to match.

        numStages is 1, 2 or 3, giving A-B-OUT, A-B-C-OUT or A-B-C-D-OUT.
        The SINK sits at x = 70 + numStages * 60 so the layout stays proportionate.
        """
        import copy

        # stage ordinal: B=1, C=2, D=3
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

            # STAGE_A is always kept, at a fixed position
            if stageID == "STAGE_A":
                filtered.append(eq_copy)

            # the SINK is always kept, but moved
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

            # a waiting area survives only if its stage does
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
                    # WAIT_OUT sits 20 beyond OUT_OUT
                    out_x_base = 70 + (numStages * 60)
                    if eq_copy.get("workPosition"):
                        eq_copy["workPosition"]["position"]["x"] = out_x_base + 10 + 20

                if should_include:
                    filtered.append(eq_copy)

            # intermediate stages
            elif stageID in stageMapping:
                if stageMapping[stageID] <= numStages:
                    filtered.append(eq_copy)

        print(f"Equipment filtered: {len(equipmentInfo)} -> {len(filtered)}")
        print(f"  OUT position adjusted to x={70 + (numStages * 60)}")
        return filtered

    def filterWaitingAreasByStages(self, waitingAreaInfo, numStages):
        """Filter the waiting areas by numStages and reposition them to match."""
        import copy

        # the stage each waiting area belongs to
        areaStageMapping = {
            "WAITING_AREA_A": 0,      # always kept
            "WAITING_AREA_B": 1,
            "WAITING_AREA_C": 2,
            "WAITING_AREA_D": 3,
            "WAITING_AREA_OUT": 0     # always kept, only repositioned
        }

        filtered = []
        for area in waitingAreaInfo:
            area_copy = copy.deepcopy(area)
            areaID = area_copy.get("areaID", "")

            # the areaID prefix identifies the stage
            area_stage = None
            for prefix, stage in areaStageMapping.items():
                if areaID.startswith(prefix):
                    area_stage = stage
                    break

            if area_stage is not None:
                # the STAGE_A waiting area is always kept
                if areaID.startswith("WAITING_AREA_A"):
                    filtered.append(area_copy)

                # the SINK waiting area is always kept, but moved
                elif areaID.startswith("WAITING_AREA_OUT"):
                    out_x_base = 70 + (numStages * 60)
                    area_copy["position"]["x"] = out_x_base + \
                        10 + 15  # 15 beyond OUT_OUT, which is itself +10
                    filtered.append(area_copy)

                # B, C and D are filtered by numStages
                elif area_stage <= numStages:
                    filtered.append(area_copy)

        print(
            f"  WaitingArea filtered: {len(waitingAreaInfo)} -> {len(filtered)}")
        print(
            f"  WAITING_AREA_OUT position adjusted to x={70 + (numStages * 60) + 25}")
        return filtered

    def filterSequenceByStages(self, seqInfo, numStages):
        """Trim each process sequence to numStages."""
        # the stage each process type belongs to
        processStageMapping = {
            "PROCESS_B1": 1, "PROCESS_B2": 1, "PROCESS_B3": 1,
            "PROCESS_C1": 2, "PROCESS_C2": 2, "PROCESS_C3": 2,
            "PROCESS_D1": 3, "PROCESS_D2": 3, "PROCESS_D3": 3
        }

        filtered = []
        for seq in seqInfo:
            filteredSequence = []
            for processType in seq["sequenceList"]:
                # SOURCE and SINK are always kept
                if processType == "SOURCE" or processType == "SINK":
                    filteredSequence.append(processType)
                # intermediate stages are filtered by numStages
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
        """Write the filtered layout as map.json inside the most recent result folder."""
        import os
        from datetime import datetime

        filtered_map_data = {
            "fileName": "map",
            "equipmentInfo": filteredEquipmentInfo,
            "WatingareaInfo": filteredWaitingAreas
        }

        # find the most recent run folder under Visualizations
        viz_path = os.path.join(os.path.dirname(strPath), "Visualizations")
        if os.path.exists(viz_path):
            # folder names are timestamps, so the last one sorted is the newest
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
