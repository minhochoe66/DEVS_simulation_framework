from SimulationEngine.ClassicDEVS.DEVSCoupledModel import DEVSCoupledModel
from modeling.simulation.PhysicalSystem.atomic.Sensor import Sensor
from SimulationEngine.Utility.Configurator import Configurator
from modeling.simulation.PhysicalSystem.atomic.Global_Planner import GlobalPlanner
from modeling.simulation.PhysicalSystem.atomic.Local_Planner import LocalPlanner


class Planner_AMR(DEVSCoupledModel):
    def __init__(self, ID, objConfiguration: Configurator, globalVar, algorithm):
        super().__init__(ID)
        self.algorithm = algorithm

        self.objConfiguration = objConfiguration
        self.globalVar = globalVar
        gpp = GlobalPlanner(ID+'_GPP', objConfiguration,
                            globalVar, self.algorithm)
        lpp = LocalPlanner(ID+'_LPP', objConfiguration, globalVar)
        self.addModel(gpp)
        self.addModel(lpp)

        # Input Ports
        self.addInputPort("Task_I")  # FleetManagement로부터 작업 지시
        self.addInputPort("amrCommand")  # FleetManagement로부터 AMR 명령
        # FleetManagement로부터 단순 이동 명령 (jobID 없음)
        self.addInputPort("amrGoCommand")
        self.addInputPort("MyManeuverState_I")
        self.addInputPort("OtherManeuverState_I")
        self.addInputPort("jobExchange_I")  # Equipment로부터 작업 교환 완료

        # Output Ports
        self.addOutputPort("RequestManeuver_O")
        self.addOutputPort("EmergencyBackup_O")
        self.addOutputPort("Complete_O")
        self.addOutputPort("Docking_O")
        self.addOutputPort("UndockingComplete_O")  # 언도킹 완료 신호

        # FleetManagement -> GlobalPlanner (작업 지시)
        self.addExternalInputCoupling("Task_I", gpp, "Task_I")

        self.addExternalInputCoupling("amrGoCommand", gpp, "amrGoCommand")
        # FleetManagement -> Maneuver (AMR 명령)
        self.addExternalInputCoupling("amrCommand", lpp, "amrCommand")
        # Maneuver -> GlobalPlanner
        self.addExternalInputCoupling(
            "MyManeuverState_I", gpp, "ManeuverState_I")

        # Maneuver -> LocalPlanner
        self.addExternalInputCoupling(
            "MyManeuverState_I", lpp, "ManeuverState_I")

        # Sensor -> Planners (다른 에이전트/장애물 정보)
        self.addExternalInputCoupling(
            "OtherManeuverState_I", lpp, "OtherManeuverState_I")
        self.addExternalInputCoupling(
            "OtherManeuverState_I", gpp, "OtherManeuverState_I")

        # Equipment -> LocalPlanner (작업 교환 완료)
        self.addExternalInputCoupling(
            "jobExchange_I", lpp, "jobExchange_I")
        self.addExternalInputCoupling(
            "UndockingComplete_I", lpp, "UndockingComplete_I")
        # Output Coupling
        self.addExternalOutputCoupling(
            lpp, "RequestManeuver_O", "RequestManeuver_O")
        self.addExternalOutputCoupling(
            lpp, "EmergencyBackup_O", "EmergencyBackup_O")
        # 도킹 명령/완료 신호 외부로 노출 (필요시 상위에서 Maneuver와 연결)
        self.addExternalOutputCoupling(lpp, "Docking_O", "Docking_O")
        self.addExternalOutputCoupling(
            lpp, "EquipmentDocking", "EquipmentDocking")
        self.addExternalOutputCoupling(lpp, "Undocking_O", "Undocking_O")
        self.addExternalOutputCoupling(lpp, "Complete_O", "Complete_O")
        self.addExternalOutputCoupling(
            lpp, "UndockingComplete_O", "UndockingComplete_O")
        # Internal Coupling
        # GlobalPlanner → LocalPlanner (경로 전달)
        self.addInternalCoupling(
            gpp, "GlobalWaypoint_O", lpp, "GlobalWaypoint_I")

        # LocalPlanner → GlobalPlanner (재계획 요청)
        self.addInternalCoupling(lpp, "Replan", gpp, "Replan")
        self.addInternalCoupling(lpp, "Docking_O", gpp, "Docking_I")
        self.addInternalCoupling(
            lpp, "DockingComplete", gpp, "DockingComplete")
        self.addInternalCoupling(lpp, "Undocking_O", gpp, "Undocking_I")
