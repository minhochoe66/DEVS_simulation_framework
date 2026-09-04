from SimulationEngine.ClassicDEVS.DEVSCoupledModel import DEVSCoupledModel
from modeling.simulation.SystemController.FleetManagement import FleetManagement
from modeling.simulation.SystemController.Scheduler import Scheduler


class SystemController(DEVSCoupledModel):
    def __init__(self, ID, globalVar):
        super().__init__(ID)

        self.globalVar = globalVar

        # Submodels
        fleetManagement = FleetManagement("FleetManagement", self.globalVar)
        scheduler = Scheduler("Scheduler", self.globalVar)

        self.addModel(fleetManagement)
        self.addModel(scheduler)

        # Input Ports
        self.addInputPort("amrPosition")  # AMR pose
        self.addInputPort("informDone")   # Equipment finished a job
        self.addInputPort("informFree")   # Equipment is free
        self.addInputPort("undockingComplete_I")  # AMR undocking complete

        # Output Ports
        self.addOutputPort("jobAssign")   # job assignment

        # External Input Coupling
        # AMR pose -> FleetManagement
        self.addExternalInputCoupling(
            "amrPosition", fleetManagement, "amrPosition")
        self.addExternalInputCoupling(
            "undockingComplete_I", fleetManagement, "undockingComplete_I")
        # Equipment state -> Scheduler
        self.addExternalInputCoupling("informDone", scheduler, "informDone")
        self.addExternalInputCoupling("informFree", scheduler, "informFree")

        # Internal Coupling
        # FleetManagement -> Scheduler
        self.addInternalCoupling(
            fleetManagement, "fleetInfo", scheduler, "fleetInfo")

        # Scheduler -> FleetManagement (task assignment)
        self.addInternalCoupling(
            scheduler, "taskAssign", fleetManagement, "taskAssign")

        # External Output Coupling
        self.addExternalOutputCoupling(scheduler, "jobAssign", "jobAssign")

        # FleetManagement -> outside (AMR commands)
        self.addOutputPort("amrCommand")
        self.addExternalOutputCoupling(
            fleetManagement, "amrCommand", "amrCommand")
        self.addExternalOutputCoupling(
            fleetManagement, "amrGoCommand", "amrGoCommand")
        self.addExternalOutputCoupling(scheduler, "Complete_O", "Complete_O")
        self.addExternalOutputCoupling(
            scheduler, "Complete_job_O", "Complete_job_O")
