import numpy as np

class Obstacle(object):

    def __init__(self, postion, radius=0.5, sigma=0.3):

        self.position_ = postion
        self.radius_ = radius

        self.sigma_ = sigma

    def get_repulsive_force(self, robot, rep=0.25):

        # dist_value = ( 1 / (self.sigma_ * np.sqrt(2 * np.pi))) * np.exp(-(self.distance(self.position_, position)**2 / (2*self.sigma_*self.sigma_)) )
        # return dist_value

        pos = [robot.position_[0][0], robot.position_[1][0]]

        delta = np.array(self.position_) - np.array(pos)
        distanceSq = np.sum(delta**2)
        forceMagnitude = rep / distanceSq
        forceDirection = -delta / np.linalg.norm(delta)
        force = forceMagnitude * forceDirection

        return force

    def distance(self, p1, p2):
        return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)