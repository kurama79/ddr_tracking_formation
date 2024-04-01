import numpy as np

class Camera(object):

    def __init__(self, robot, range):

        self.robot_ = robot
        self.id_ = robot.id_
        self.range_ = range

        self.leader_ = False
        self.blind_ = False
        self.direction_ = 0.0
        self.current_reference_ = 0.0

        self.path_ = [self.direction_]

    def camera_control(self, dt):

        k = 3.0 # Gain

        # x = self.robot_.position_[2][0] + self.direction_
        x = self.direction_
        y = x

        ref = self.current_reference_
        diff_ref = 0.0

        error = y - ref

        # Controllers
        v = k * error
        # u = diff_ref - self.robot_.control_[0][0] + v
        u = diff_ref + v

        x = x + u * dt

        cicles = x / (2 * np.pi)
        intCicles = int(cicles)
        rest = cicles - intCicles
        x = 2 * np.pi * rest

        if abs(x - self.robot_.position_[2][0]) >= self.range_:

            self.blind_ = True
            x = (self.robot_.position_[2][0] + self.range_) * ((abs(x - self.robot_.position_[2][0])) / (x - self.robot_.position_[2][0]))

        self.direction_ = x
        # self.direction_ = x + u * dt
        self.path_.append(self.direction_)

        print("The camera error is: ", error)

    def get_reference(self, user):

        # userPositionX = user.position_[0][0]
        # userPositionY = user.position_[1][0]

        # robotPositionX = self.robot_.position_[0][0]
        # robotPositionY = self.robot_.position_[1][0]

        # x = robotPositionX - userPositionX
        # y = robotPositionY - userPositionY

        # theta = np.arctan(y / x)
        # self.current_reference_ = theta + np.pi

        self.current_reference_ = self.robot_.theta_ref_ + np.pi

    def is_leader(self, user):

        if self.blind_:
            self.leader_ = False
            return

        userDirection = user.position_[2][0]
        
        if userDirection <= self.robot_.theta_ref_ + self.range_ and userDirection >= self.robot_.theta_ref_ - self.range_:
            self.leader_ = True