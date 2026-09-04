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

        # SystemController 생성
        systemController = SystemController(
            "SystemController", self.globalVar,)
        self.addModel(systemController)

        # AMR 플릿
        self.objVehicles = []
        # 1단계: AMR 생성 및 SystemController 연결
        for vehicleID in vehicleInfo.keys():
            objVehicle = AMR(
                vehicleID, self.globalVar.objConfiguration, self.globalVar, algorithm="RRT")
            self.objVehicles.append(objVehicle)
            self.addModel(objVehicle)

            # AMR 위치 -> SystemController (내부에서 FleetManagement로 전달)
            self.addInternalCoupling(
                objVehicle, "MyManeuverState_O", systemController, "amrPosition"
            )

            # SystemController 작업 지시 -> AMR
            # 각 AMR은 amrID가 일치하는 메시지만 처리한다
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

        # 2단계: AMR 간 위치 공유. 서로를 동적 장애물로 인식한다
        for i, objVehicle_i in enumerate(self.objVehicles):
            for j, objVehicle_j in enumerate(self.objVehicles):
                if i != j:
                    self.addInternalCoupling(
                        objVehicle_j, "MyManeuverState_O",
                        objVehicle_i, "OtherManeuverState_I"
                    )
        # Equipment 모델
        self.objEquipment = []
        for key, eqpInfo in equipmentInfo.items():
            objEquipment = Equipment(
                eqpInfo.strEquipmentID, self.globalVar, eqpInfo)
            self.objEquipment.append(objEquipment)
            self.addModel(objEquipment)

            # SOURCE 장비만 외부 job 입력을 받는다
            if eqpInfo.strType == "SOURCE":
                self.addExternalInputCoupling("job", objEquipment, "job")

            self.addInternalCoupling(
                objEquipment, "informDone", systemController, "informDone")

            self.addInternalCoupling(
                objEquipment, "informFree", systemController, "informFree")

        # AMR <-> Equipment (도킹 및 작업 교환)
        for objVehicle in self.objVehicles:
            for objEquipment in self.objEquipment:
                # 도킹 신호는 모든 장비에 브로드캐스트되고, 장비는 ID가 일치할 때만 처리한다
                self.addInternalCoupling(
                    objVehicle, "EquipmentDocking", objEquipment, "EquipmentDocking"
                )

                # 작업 교환 완료 신호도 같은 방식으로 브로드캐스트된다
                self.addInternalCoupling(
                    objEquipment, "jobExchange", objVehicle, "jobExchange_I"
                )
                self.addInternalCoupling(
                    objEquipment, "UndockingComplete", objVehicle, "UndockingComplete_I"
                )

        # self.addExternalOutputCoupling(
        #     systemController, "Complete_O", "Complete_O")
