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
        self.addInputPort("Task_I")  # transport order from FleetManagement
        self.addInputPort("amrCommand")  # AMR command from FleetManagement
        # bare move command, no jobID
        self.addInputPort("amrGoCommand")
        self.addInputPort("MyManeuverState_I")
        self.addInputPort("OtherManeuverState_I")
        self.addInputPort("jobExchange_I")  # job handover complete, from Equipment

        # Output Ports
        self.addOutputPort("RequestManeuver_O")
        self.addOutputPort("EmergencyBackup_O")
        self.addOutputPort("Complete_O")
        self.addOutputPort("Docking_O")
        self.addOutputPort("UndockingComplete_O")  # undocking complete

        # FleetManagement -> GlobalPlanner (transport order)
        self.addExternalInputCoupling("Task_I", gpp, "Task_I")

        self.addExternalInputCoupling("amrGoCommand", gpp, "amrGoCommand")
        # FleetManagement -> Maneuver (AMR command)
        self.addExternalInputCoupling("amrCommand", lpp, "amrCommand")
        # Maneuver -> GlobalPlanner
        self.addExternalInputCoupling(
            "MyManeuverState_I", gpp, "ManeuverState_I")

        # Maneuver -> LocalPlanner
        self.addExternalInputCoupling(
            "MyManeuverState_I", lpp, "ManeuverState_I")

        # Sensor -> Planners (peer robots and obstacles)
        self.addExternalInputCoupling(
            "OtherManeuverState_I", lpp, "OtherManeuverState_I")
        self.addExternalInputCoupling(
            "OtherManeuverState_I", gpp, "OtherManeuverState_I")

        # Equipment -> LocalPlanner (job handover complete)
        self.addExternalInputCoupling(
            "jobExchange_I", lpp, "jobExchange_I")
        self.addExternalInputCoupling(
            "UndockingComplete_I", lpp, "UndockingComplete_I")
        # Output Coupling
        self.addExternalOutputCoupling(
            lpp, "RequestManeuver_O", "RequestManeuver_O")
        self.addExternalOutputCoupling(
            lpp, "EmergencyBackup_O", "EmergencyBackup_O")
        # Expose the docking command and completion signal to the parent model
        self.addExternalOutputCoupling(lpp, "Docking_O", "Docking_O")
        self.addExternalOutputCoupling(
            lpp, "EquipmentDocking", "EquipmentDocking")
        self.addExternalOutputCoupling(lpp, "Undocking_O", "Undocking_O")
        self.addExternalOutputCoupling(lpp, "Complete_O", "Complete_O")
        self.addExternalOutputCoupling(
            lpp, "UndockingComplete_O", "UndockingComplete_O")
        # Internal Coupling
        # GlobalPlanner -> LocalPlanner (global path)
        self.addInternalCoupling(
            gpp, "GlobalWaypoint_O", lpp, "GlobalWaypoint_I")

        # LocalPlanner -> GlobalPlanner (replan request)
        self.addInternalCoupling(lpp, "Replan", gpp, "Replan")
        self.addInternalCoupling(lpp, "Docking_O", gpp, "Docking_I")
        self.addInternalCoupling(
            lpp, "DockingComplete", gpp, "DockingComplete")
        self.addInternalCoupling(lpp, "Undocking_O", gpp, "Undocking_I")
