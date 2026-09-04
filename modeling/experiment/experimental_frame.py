from SimulationEngine.ClassicDEVS.DEVSCoupledModel import DEVSCoupledModel
from modeling.experiment.atomic.Data_generator import Data_generator
from modeling.experiment.atomic.Data_collector import Data_collector


class ExperimentalFrame(DEVSCoupledModel):

    def __init__(self, ID, globalVar, iteration_num=None, scenario_label=None):
        super().__init__(ID)
        self.globalVar = globalVar
        self.iteration_num = iteration_num
        self.scenario_label = scenario_label

        data_generator = Data_generator("data_generator", self.globalVar)
        self.data_collector = Data_collector(
            "data_collector",
            objConfiguration=self.globalVar.objConfiguration,
            globalVar=self.globalVar,
            iteration_num=iteration_num,
            scenario_label=scenario_label
        )
        self.addModel(data_generator)
        self.addModel(self.data_collector)

        self.addOutputPort("job")

        self.addExternalOutputCoupling(data_generator, "job", "job")

        self.addInputPort("MyManeuverState_I")
        self.addInputPort("Complete_I")

        self.addExternalInputCoupling(
            "MyManeuverState_I", self.data_collector, "MyManeuverState_I")

        self.addExternalInputCoupling(
            "Complete_I", self.data_collector, "Complete_I")

    def get_data_collector(self):
        """Return the Data_collector used by the Monte Carlo analysis."""
        return self.data_collector
