from SimulationEngine.ClassicDEVS.DEVSCoupledModel import DEVSCoupledModel
from modeling.simulation.PhysicalSystem.atomic.Maneuver import Maneuver
from modeling.simulation.PhysicalSystem.Vehicle_Planner import Planner_AMR
from modeling.simulation.PhysicalSystem.atomic.Sensor import Sensor


class AMR(DEVSCoupledModel):
    def __init__(self, ID, objConfiguration, globalVar, algorithm):
        super().__init__(ID)

        self.objConfiguration = objConfiguration
        self.globalVar = globalVar
        self.vehicleID = ID
        self.algorithm = algorithm
        sensor = Sensor(self.vehicleID+'_sensor', self.objConfiguration)
        maneuver = Maneuver(self.vehicleID+'_maneuver',
                            self.objConfiguration, self.globalVar)
        planner_amr = Planner_AMR(
            self.vehicleID+'_planner_amr', self.objConfiguration, self.globalVar, self.algorithm)

        self.addModel(sensor)
        self.addModel(maneuver)
        self.addModel(planner_amr)

        # Input Ports
        self.addInputPort("amrCommand")  # transport order from FleetManagement
        self.addInputPort("amrGoCommand")  # bare move command, no job attached
        self.addInputPort("OtherManeuverState_I")
        self.addInputPort("jobExchange_I")  # job handover complete, from Equipment

        # Output Ports
        self.addOutputPort("MyManeuverState_O")
        self.addOutputPort("Complete_O")

        # External Input Coupling
        self.addExternalInputCoupling("amrCommand", planner_amr, "Task_I")
        self.addExternalInputCoupling(
            "Complete_job_I", sensor, "Complete_job_I")

        self.addExternalInputCoupling(
            "amrGoCommand", planner_amr, "amrGoCommand")
        self.addExternalInputCoupling(
            "amrCommand", maneuver, "amrCommand")  # also routed straight to Maneuver
        self.addExternalInputCoupling(
            "OtherManeuverState_I", sensor, "OtherManeuverState_I")  # peer robot poses
        self.addExternalInputCoupling(
            "jobExchange_I", planner_amr, "jobExchange_I")
        self.addExternalInputCoupling(
            "UndockingComplete_I", planner_amr, "UndockingComplete_I")
        # External Output Coupling
        self.addExternalOutputCoupling(
            maneuver, "MyManeuverState_O", "MyManeuverState_O")
        self.addExternalOutputCoupling(maneuver, "Complete_O", "Complete_O")
        self.addExternalOutputCoupling(planner_amr, "Complete_O", "Complete_O")

        # self.addExternalOutputCoupling(planner_amr, "Docking_O", "Docking_O")
        self.addExternalOutputCoupling(
            planner_amr, "EquipmentDocking", "EquipmentDocking")
        self.addExternalOutputCoupling(
            planner_amr, "UndockingComplete_O", "UndockingComplete_O")
        self.addInternalCoupling(
            planner_amr, "RequestManeuver_O", maneuver, "RequestManeuver_I")

        self.addInternalCoupling(
            planner_amr, "Docking_O", maneuver, "Docking_I")
        self.addInternalCoupling(
            planner_amr, "Undocking_O", maneuver, "Undocking_I")

        self.addInternalCoupling(
            maneuver, "MyManeuverState_O", planner_amr, "MyManeuverState_I")
        self.addInternalCoupling(
            sensor, "OtherManeuverState_O", planner_amr, "OtherManeuverState_I")
        self.addInternalCoupling(
            planner_amr, "Complete_O", maneuver, "Complete_I")
        self.addInternalCoupling(
            planner_amr, "Complete_O", sensor, "Complete_I")
