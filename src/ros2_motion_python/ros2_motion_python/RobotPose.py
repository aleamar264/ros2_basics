class RobotPose():

    def __init__(self, x:float = 0.0, y:float = 0.0, theta:float = 0.0) -> None:
        self.x = x
        self.y = y
        self.theta = theta
    
    def get_x(self) -> float:
        return self.x

    def set_x(self, x: float):
        self.x = x

    def get_y(self) -> float:
        return self.y

    def set_y(self, y: float):
        self.y = y

    def get_theta(self) -> float:
        return self.theta

    def set_theta(self, theta: float):
        self.theta = theta

    def get_pose(self):
        return self

    def __str__(self) -> str:
        return f"RobotPose(x={self.x}, y={self.y}, theta={self.theta})"