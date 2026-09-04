from SimulationEngine.ClassicDEVS.DEVSAtomicModel import DEVSAtomicModel


class Data_generator(DEVSAtomicModel):

    def __init__(self, ID, globalVar):
        super().__init__(ID)
        self.globalVar = globalVar

        # State Variables
        self.addStateVariable('state', 'GEN')

        # Output Port
        self.addOutputPort("job")

        # Variables
        self.jobRemains = len(self.globalVar.getTargetJobs())
        self.availableEquipments = []
        self.jobID = 0

    def funcExternalTransition(self, strPort, event):
        pass

    def funcOutput(self):
        state = self.getStateValue('state')
        if state == "GEN":
            if self.jobRemains != 0:
                self.checkEquipmentState()
                for equipmentID in self.availableEquipments:
                    self.jobID = self.jobID + 1
                    self.jobRemains = self.jobRemains - 1
                    self.addOutputEvent("job", [equipmentID, self.jobID])
                    self.availableEquipments.pop(0)
                    self.globalVar.printTerminal(
                        f"[{self.getTime()}][Data_generator] Job #{self.jobID} generated → {equipmentID}"
                    )
                    break
            else:
                self.setStateValue('state', 'WAIT')

    def funcInternalTransition(self):
        state = self.getStateValue('state')
        if state == 'GEN':
            pass

    def funcTimeAdvance(self):
        state = self.getStateValue('state')
        if state == 'GEN':
            return 1
        else:
            return float('inf')

    def checkEquipmentState(self):
        """SOURCE 장비(A-1) 상태 확인"""
        equipmentInfo = self.globalVar.getEquipmentInfo()

        for key, value in equipmentInfo.items():
            # processType이 "SOURCE"인 장비 체크 (A-1)
            if key not in self.availableEquipments and value.strType == "SOURCE" and value.strState == "EMPTY":
                self.availableEquipments.append(key)
