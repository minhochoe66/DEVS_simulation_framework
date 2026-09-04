class MsgArrive:
    def __init__(self, strID, goal_x, goal_y, task_phase=None, job_id=None):
        self.strID = strID
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.task_phase = task_phase
        self.job_id = job_id

        