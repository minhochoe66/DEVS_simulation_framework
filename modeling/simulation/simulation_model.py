from SimulationEngine.ClassicDEVS.DEVSCoupledModel import DEVSCoupledModel
from modeling.simulation.PhysicalSystem.Equipment import Equipment
from modeling.simulation.SystemController.SystemController import SystemController
from modeling.simulation.PhysicalSystem.Vehicle import AMR


class SimulationModel(DEVSCoupledModel):

    def __init__(self, ID, globalVar):
        super().__init__(ID)

        self.globalVar = globalVar
        equipmentInfo = globalVar.getEquipmentInfo()
        vehicleInfo = globalVar.getVehicleInfo()

        # Input Port
        self.addInputPort("job")

        # System controller
        systemController = SystemController(
            "SystemController", self.globalVar,)
        self.addModel(systemController)

        # AMR fleet
        self.objVehicles = []
        # Step 1: create the AMRs and couple them to the system controller
        for vehicleID in vehicleInfo.keys():
            objVehicle = AMR(
                vehicleID, self.globalVar.objConfiguration, self.globalVar, algorithm="RRT")
            self.objVehicles.append(objVehicle)
            self.addModel(objVehicle)

            # AMR pose -> SystemController, which routes it on to FleetManagement
            self.addInternalCoupling(
                objVehicle, "MyManeuverState_O", systemController, "amrPosition"
            )

            # SystemController orders -> AMR
            # each AMR acts only on messages whose amrID matches its own
            self.addInternalCoupling(
                systemController, "amrCommand", objVehicle, "amrCommand"
            )
            self.addInternalCoupling(
                systemController, "amrGoCommand", objVehicle, "amrGoCommand"
            )
            self.addInternalCoupling(
                systemController, "Complete_job_O", objVehicle, "Complete_job_I"
            )
            self.addExternalOutputCoupling(
                objVehicle, "MyManeuverState_O", "MyManeuverState_O")
            self.addInternalCoupling(
                objVehicle, "UndockingComplete_O", systemController, "undockingComplete_I")

        # Step 2: share poses between AMRs so that peers act as dynamic obstacles
        for i, objVehicle_i in enumerate(self.objVehicles):
            for j, objVehicle_j in enumerate(self.objVehicles):
                if i != j:
                    self.addInternalCoupling(
                        objVehicle_j, "MyManeuverState_O",
                        objVehicle_i, "OtherManeuverState_I"
                    )
        # Equipment models
        self.objEquipment = []
        for key, eqpInfo in equipmentInfo.items():
            objEquipment = Equipment(
                eqpInfo.strEquipmentID, self.globalVar, eqpInfo)
            self.objEquipment.append(objEquipment)
            self.addModel(objEquipment)

            # only SOURCE equipment receives the external job input
            if eqpInfo.strType == "SOURCE":
                self.addExternalInputCoupling("job", objEquipment, "job")

            self.addInternalCoupling(
                objEquipment, "informDone", systemController, "informDone")

            self.addInternalCoupling(
                objEquipment, "informFree", systemController, "informFree")

        # AMR <-> Equipment: docking and job handover
        for objVehicle in self.objVehicles:
            for objEquipment in self.objEquipment:
                # docking is broadcast to every machine; each acts only if the ID matches
                self.addInternalCoupling(
                    objVehicle, "EquipmentDocking", objEquipment, "EquipmentDocking"
                )

                # job handover completion is broadcast the same way
                self.addInternalCoupling(
                    objEquipment, "jobExchange", objVehicle, "jobExchange_I"
                )
                self.addInternalCoupling(
                    objEquipment, "UndockingComplete", objVehicle, "UndockingComplete_I"
                )

        # self.addExternalOutputCoupling(
        #     systemController, "Complete_O", "Complete_O")
