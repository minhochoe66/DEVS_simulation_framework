from SimulationEngine.ClassicDEVS.DEVSCoupledModel import DEVSCoupledModel
from SharedData.GlobalVar import GlobalVar
from modeling.experiment.experimental_frame import ExperimentalFrame
from modeling.simulation.simulation_model import SimulationModel


class AMRSimModel(DEVSCoupledModel):

    def __init__(self, objConfiguration, iteration_num=None, scenario_label=None):
        super().__init__("AMRSimModel")

        self.objConfiguration = objConfiguration
        self.iteration_num = iteration_num
        self.scenario_label = scenario_label

        # Configuration
        equipmentInfo = objConfiguration.getConfiguration("equipmentInfo")
        seqInfo = objConfiguration.getConfiguration("seqInfo")
        performanceInfo = objConfiguration.getConfiguration("performanceInfo")
        vehicleInfo = objConfiguration.getConfiguration("vehicleInfo")
        numJob = objConfiguration.getConfiguration("numJob")
        numVehicle = objConfiguration.getConfiguration("numVehicles")

        self.globalVar = GlobalVar(
            isTerminalOn=objConfiguration.getConfiguration("isTerminalOn"),
            equipmentInfo=equipmentInfo,
            seqInfo=seqInfo,
            performanceInfo=performanceInfo,
            vehicleInfo=vehicleInfo,
            numJob=numJob,
            numVehicle=numVehicle,
            objConfiguration=objConfiguration
        )

        # 하위 모델 구성
        self.EF = ExperimentalFrame(
            "EF", self.globalVar, iteration_num=iteration_num, scenario_label=scenario_label)
        SM = SimulationModel("SM", self.globalVar)
        self.addModel(self.EF)
        self.addModel(SM)

        # EF의 job 출력을 SM의 job 입력으로 연결
        self.addInternalCoupling(self.EF, "job", SM, "job")
        self.addInternalCoupling(
            SM, "MyManeuverState_O", self.EF, "MyManeuverState_I")

        self.addInternalCoupling(SM, "Complete_O", self.EF, "Complete_I")

    def get_iteration_results(self):
        """이번 반복의 성능 지표를 반환한다."""
        data_collector = self.EF.get_data_collector()
        return data_collector.get_iteration_results()
