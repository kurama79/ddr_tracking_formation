import numpy as np

from user import User
from robot import *
from obstacles import Obstacle

class Simulator():

    def __init__(self, mapDimensions, deltaTime, numRobots, formationDistance):

        self.initilize_ = True
        
        self.map_dimensions_ = mapDimensions
        self.delta_time_ = deltaTime
        self.time_ = 0.0
        self.number_robots_ = numRobots
        self.formation_distance_ = formationDistance

        self.user_ = None
        self.robots_ = []
        self.obstacles_ = []
        self.cameras_ = []

    def initialize_cameras(self):

        thetaRef = (2 * np.pi) / self.number_robots_
        range = thetaRef / 2

        for robot in self.robots_:
            
            camera = Camera(robot, range)
            self.cameras_.append(camera)

    def add_obstacles(self):

        # Adding one obstacle
        pos = [-2.0, 6.0]
        obs = Obstacle(pos)
        self.obstacles_.append(obs)

        pos = [-1.7, -1.0]
        obs = Obstacle(pos)
        self.obstacles_.append(obs)

        pos = [7.8, 1.0]
        obs = Obstacle(pos)
        self.obstacles_.append(obs)

        pos = [5.0, 9.0]
        obs = Obstacle(pos)
        self.obstacles_.append(obs)

        # pos = [-1.0, -3.1]
        # obs = Obstacle(pos)
        # self.obstacles_.append(obs)

        # pos = [-2.5, 3.5]
        # obs = Obstacle(pos)
        # self.obstacles_.append(obs)

        # pos = [7.0, 1.7]
        # obs = Obstacle(pos)
        # self.obstacles_.append(obs)

    def generate_and_add_robots(self):

        thetaRef = (2 * np.pi) / self.number_robots_

        for i in range(self.number_robots_):

            pos_x = self.user_.position_[0][0] + self.formation_distance_ * np.cos(i * thetaRef)
            pos_y = self.user_.position_[1][0] + self.formation_distance_ * np.sin(i * thetaRef)
            theta = thetaRef + np.pi

            position = np.array([[pos_x], [pos_y], [theta], [vel]])

            robot = Robot_DDR(i, thetaRef, position, self.formation_distance_, range)

            self.robots_.append(robot)

            del robot

    def generate_and_add_robots_with_camera(self):

        thetaRef = (2 * np.pi) / self.number_robots_
        cameraRotation = thetaRef / 1.0
        # cameraRange = (5 * np.pi) / 6

        for i in range(self.number_robots_):

            pos_x = self.user_.position_[0][0] + self.formation_distance_ * np.cos(i * thetaRef)
            pos_y = self.user_.position_[1][0] + self.formation_distance_ * np.sin(i * thetaRef)
            theta = thetaRef + np.pi
            alpha = theta

            position = np.array([[pos_x], [pos_y], [theta], [alpha]])

            robot = Robot_DDR_Camera(i, thetaRef, position, self.formation_distance_, cameraRotation)

            self.robots_.append(robot)

            del robot

    def generate_and_add_robots_with_pseudo_camera(self):

        thetaRef = (2 * np.pi) / self.number_robots_
        cameraRotation = thetaRef / 1.0
        # cameraRange = (5 * np.pi) / 6

        for i in range(self.number_robots_):

            # pos_x = self.user_.position_[0][0] + (self.formation_distance_) * np.cos(i * thetaRef)
            # pos_y = self.user_.position_[1][0] + (self.formation_distance_) * np.sin(i * thetaRef)
            pos_x = self.user_.position_[0][0] + (self.formation_distance_*2.7) * np.cos((i * thetaRef) + np.random.uniform(0, np.pi/2))
            pos_y = self.user_.position_[1][0] + (self.formation_distance_*2.7) * np.sin((i * thetaRef) + np.random.uniform(0, np.pi/2))

            # theta = self.user_.position_[2][0] 
            theta = thetaRef * i
            # alpha = (thetaRef * i) + np.pi
            alpha = 0.0

            position = np.array([[pos_x], [pos_y], [theta], [alpha]])

            if i == self.number_robots_ - 1:
                neighbor = int(0)
            else:
                neighbor = int(i + 1)

            robot = Robot_DDR_Pseudo_Camera(i, thetaRef, position, self.formation_distance_, cameraRotation, self.map_dimensions_, neighbor)

            self.robots_.append(robot)

            del robot

    def generate_user(self):

        x = (self.map_dimensions_[0][0] + self.map_dimensions_[0][1]) / 2
        y = (self.map_dimensions_[1][0] + self.map_dimensions_[1][1]) / 2
        theta = 0.0

        position = np.array([[x], [y], [theta]])

        self.user_ = User(position, self.map_dimensions_)

        del position

    def clean_leaders(self):

        leaderCount = 0
        for camera in self.cameras_:
            
            if leaderCount == 0 and camera.leader_:
                leaderCount += 1

            elif camera.leader_:
                camera.leader_ = False

    def close_obstacle(self, robot):

        x = robot.position_[0][0]
        y = robot.position_[1][0]

        for obs in self.obstacles_:            

            if robot.distance([x, y], obs.position_) < 2.0:
                return True
        
        return False

    def robot_evasion(self, robot):

        # robot.evasion_moves(self.delta_time_)
        robot.collision_ = True
        # min_value = np.inf
        # best_move = robot.evasions_[0]

        # for move in robot.evasions_:

        #     move_value = robot.get_attraction_force(move, self.user_)

        #     for obs in self.obstacles_:
        #         move_value += obs.get_repulsive_force(move)

        #     if move_value < min_value:

        #         min_value = move_value
        #         best_move = move

        # return best_move

        vAvoid = robot.get_attraction_force(self.user_)

        for obs in self.obstacles_:
            vAvoid += obs.get_repulsive_force(robot)

        return vAvoid


    def two_points_euclidian_distance(self, p1, p2):

        x = p2[0][0] - p1[0][0]
        y = p2[1][0] - p1[1][0]

        return np.sqrt(x**2 + y**2)

    # Step of the simulation
    def step(self):

        self.user_.moves(self.delta_time_)
        if self.user_.goal_reached():
            self.user_.new_goal()

        for robot in self.robots_:

            if self.close_obstacle(robot):
                v_avoid = self.robot_evasion(robot)

            else:
                v_avoid = [0.0, 0.0]

            # robot.tracking_control(self.delta_time_)
            # robot.set_leader(self.user_)
            robot.polar_position_controller(self.user_, self.robots_[robot.neighbor_id_], self.delta_time_, self.number_robots_, v_avoid)
            robot.camera_control(self.user_, self.delta_time_)
        # input()

        self.time_ += self.delta_time_
