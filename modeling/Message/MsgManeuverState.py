class MsgManeuverState:

    def __init__(self, strID, dblPositionX, dblPositionY, dblSpeed=None, dblYaw=None, path=None, is_returning=False, transportPhase=None, goalNodeID=None):
        self.strID = strID
        self.dblPositionX = dblPositionX
        self.dblPositionY = dblPositionY
        self.dblSpeed = dblSpeed
        self.dblYaw = dblYaw
        self.path = path
        self.is_returning = is_returning
        self.transportPhase = transportPhase  # "TO_FROM" or "TO_DESTINATION"
        self.goalNodeID = goalNodeID  # current goal node ID, e.g. "A-1_IN"

    def __str__(self):
        ret = ""
        ret += "Manuever Message : " + str(self.strID) + " : (" + \
            str(self.dblPositionX) + "," + str(self.dblPositionY) + ")"
        return ret
