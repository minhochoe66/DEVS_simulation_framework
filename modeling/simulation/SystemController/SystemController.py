from SimulationEngine.ClassicDEVS.DEVSCoupledModel import DEVSCoupledModel
from modeling.simulation.SystemController.FleetManagement import FleetManagement
from modeling.simulation.SystemController.Scheduler import Scheduler


class SystemController(DEVSCoupledModel):
    def __init__(self, ID, globalVar):
        super().__init__(ID)

        self.globalVar = globalVar

        # 하위 모델 생성
        fleetManagement = FleetManagement("FleetManagement", self.globalVar)
        scheduler = Scheduler("Scheduler", self.globalVar)

        self.addModel(fleetManagement)
        self.addModel(scheduler)

        # Input Ports (외부에서 들어오는 포트)
        self.addInputPort("amrPosition")  # AMR 위치 정보
        self.addInputPort("informDone")   # Equipment 작업 완료
        self.addInputPort("informFree")   # Equipment 준비 완료
        self.addInputPort("undockingComplete_I")  # AMR 언도킹 완료

        # Output Ports (외부로 나가는 포트)
        self.addOutputPort("jobAssign")   # 작업 배정

        # External Input Coupling (외부 → 내부)
        # AMR 위치 정보 → FleetManagement
        self.addExternalInputCoupling(
            "amrPosition", fleetManagement, "amrPosition")
        self.addExternalInputCoupling(
            "undockingComplete_I", fleetManagement, "undockingComplete_I")
        # Equipment 정보 → Scheduler
        self.addExternalInputCoupling("informDone", scheduler, "informDone")
        self.addExternalInputCoupling("informFree", scheduler, "informFree")

        # Internal Coupling (내부 연결)
        # FleetManagement → Scheduler
        self.addInternalCoupling(
            fleetManagement, "fleetInfo", scheduler, "fleetInfo")

        # Scheduler → FleetManagement (작업 할당 정보)
        self.addInternalCoupling(
            scheduler, "taskAssign", fleetManagement, "taskAssign")

        # External Output Coupling (내부 → 외부)
        # Scheduler → 외부
        self.addExternalOutputCoupling(scheduler, "jobAssign", "jobAssign")

        # FleetManagement → 외부 (AMR 명령)
        self.addOutputPort("amrCommand")
        self.addExternalOutputCoupling(
            fleetManagement, "amrCommand", "amrCommand")
        self.addExternalOutputCoupling(
            fleetManagement, "amrGoCommand", "amrGoCommand")
        self.addExternalOutputCoupling(scheduler, "Complete_O", "Complete_O")
        self.addExternalOutputCoupling(
            scheduler, "Complete_job_O", "Complete_job_O")
