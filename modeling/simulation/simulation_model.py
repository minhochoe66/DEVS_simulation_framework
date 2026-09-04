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

        # AMR Vehicle 생성
        self.objVehicles = []
        # 1단계: 모든 AMR 생성 및 기본 연결
        for vehicleID in vehicleInfo.keys():
            objVehicle = AMR(
                vehicleID, self.globalVar.objConfiguration, self.globalVar, algorithm="RRT")
            self.objVehicles.append(objVehicle)
            self.addModel(objVehicle)

            # AMR의 위치 정보를 SystemController에 연결
            # SystemController는 내부에서 FleetManagement로 전달
            self.addInternalCoupling(
                objVehicle, "MyManeuverState_O", systemController, "amrPosition"
            )

            # SystemController의 작업 지시를 AMR에 연결
            # 각 AMR은 메시지의 amrID를 확인하여 자신의 작업만 처리
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

        # 2단계: AMR 간 위치 정보 공유 연결
        # 각 AMR이 다른 모든 AMR의 위치 정보를 받도록 연결
        for i, objVehicle_i in enumerate(self.objVehicles):
            for j, objVehicle_j in enumerate(self.objVehicles):
                if i != j:  # 자기 자신은 제외
                    # AMR_j의 위치 정보를 AMR_i가 받음
                    self.addInternalCoupling(
                        objVehicle_j, "MyManeuverState_O",
                        objVehicle_i, "OtherManeuverState_I"
                    )
        # Equipment 모델 생성 및 연결
        self.objEquipment = []
        for key, eqpInfo in equipmentInfo.items():
            objEquipment = Equipment(
                eqpInfo.strEquipmentID, self.globalVar, eqpInfo)
            self.objEquipment.append(objEquipment)
            self.addModel(objEquipment)

            # SOURCE 장비는 외부 job 입력과 연결
            if eqpInfo.strType == "SOURCE":
                self.addExternalInputCoupling("job", objEquipment, "job")

            # 모든 장비의 informDone을 SystemController에 연결
            self.addInternalCoupling(
                objEquipment, "informDone", systemController, "informDone")

            # 모든 장비의 informFree를 SystemController에 연결
            self.addInternalCoupling(
                objEquipment, "informFree", systemController, "informFree")

        # AMR ↔ Equipment 연결 (도킹 및 작업 교환)
        for objVehicle in self.objVehicles:
            for objEquipment in self.objEquipment:
                # AMR의 도킹 신호를 모든 장비에 브로드캐스트
                # 각 장비는 자신의 ID와 매칭되는 경우만 처리
                self.addInternalCoupling(
                    objVehicle, "EquipmentDocking", objEquipment, "EquipmentDocking"
                )

                # 장비의 작업 교환 완료 신호를 모든 AMR에 브로드캐스트
                # 각 AMR은 자신의 ID와 매칭되는 경우만 처리
                self.addInternalCoupling(
                    objEquipment, "jobExchange", objVehicle, "jobExchange_I"
                )
                self.addInternalCoupling(
                    objEquipment, "UndockingComplete", objVehicle, "UndockingComplete_I"
                )

        # self.addExternalOutputCoupling(
        #     systemController, "Complete_O", "Complete_O")
