from SimulationEngine.ClassicDEVS.DEVSAtomicModel import DEVSAtomicModel
from modeling.Message.MsgManeuverState import MsgManeuverState
from SimulationEngine.Utility.Configurator import Configurator


class PoseStorage:
    def __init__(self):
        self.data = []

    def add_or_update_pose(self, pose):
        for i, entry in enumerate(self.data):
            if entry[0] == pose.strID:
                # Update existing entry
                self.data[i] = (pose.strID, pose.x, pose.y,
                                pose.yaw, pose.lin_vel, pose.ang_vel)
                return
        # Add new entry if ID does not exist
        self.data.append((pose.strID, pose.x, pose.y,
                         pose.yaw, pose.lin_vel, pose.ang_vel))

    def __str__(self):
        ret = ""
        for entry in self.data:
            ret += f'ID: {entry[0]}, Position: ({entry[1]}, {entry[2]}), Yaw: {entry[3]}, Linear Velocity: {entry[4]}, Angular Velocity: {entry[5]}\n'
        return ret


class Sensor(DEVSAtomicModel):
    def __init__(self, ID, objConfiguration):
        super().__init__(ID)

        self.objConfiguration = objConfiguration
        self.pose_storage = PoseStorage()

        self.addInputPort("OtherManeuverState_I")
        self.addInputPort("Complete_job_I")
        self.addOutputPort("OtherManeuverState_O")

        self.addStateVariable("state", 'INIT')

    def funcExternalTransition(self, strPort, objEvent):
        state = self.getStateValue('state')
        if strPort == 'OtherManeuverState_I':
            if state != 'WAIT':
                if objEvent.strID != self.ID.split('_', 1)[0]:
                    if self.getStateValue('state') == 'INIT':
                        self.pose_storage.add_or_update_pose(objEvent)
                        self.setStateValue('state', 'ACTIVE')
                    else:
                        self.pose_storage.add_or_update_pose(objEvent)
                        self.continueTimeAdvance()
            else:
                pass
        elif strPort == 'Complete_job_I':
            self.setStateValue('state', 'WAIT')

    def funcOutput(self):
        state = self.getStateValue('state')
        if state == "ACTIVE":
            if len(self.pose_storage.data) != 0:
                for i in range(len(self.pose_storage.data)):
                    objSensorMessage = MsgManeuverState(
                        self.pose_storage.data[i][0],
                        self.pose_storage.data[i][1],
                        self.pose_storage.data[i][2],
                        self.pose_storage.data[i][3],
                        self.pose_storage.data[i][4]
                    )
                    self.addOutputEvent(
                        "OtherManeuverState_O", objSensorMessage)

    def funcInternalTransition(self):
        state = self.getStateValue('state')
        if state == 'ACTIVE':
            self.setStateValue('state', 'ACTIVE')

    def funcTimeAdvance(self):
        state = self.getStateValue('state')
        if state == "INIT":
            return float('inf')
        elif state == "WAIT":
            return float('inf')
        elif state == "ACTIVE":
            return 0.1

    def funcSelect(self):
        pass
