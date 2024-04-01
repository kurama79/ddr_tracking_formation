# Clase para genera el objeto usuario

import numpy as np 
import random
import shapely.geometry

class User(object):

	def __init__(self, position, roomLimits):

		# Initial values
		self.roomLimits_ = roomLimits

		# Variables
		self.position_ = position
		self.controls_ = np.array([[0.0], [0.0]]) # Linear and Angular velocities
		self.current_goal_ = np.array([[0.0], [0.0]])

		# Data to save
		self.path_ = [[position[0][0], position[1][0], position[2][0]]]

	# Generates the user movement
	def moves(self, dt):

		'''
		We generate a control to track a reference (generated randomly along the map)

		Model used to generates the movement od the user 
				x(k+1) = x(k) + cos(theta) * u_1 * dt
				y(k+1) = y(k) + sin(theta) * u_1 * dt
				theta(k+1) = theta(k) + u_2 * dt
		'''

		d = 1.0 # Control point to generate an invertible matrix (decoupling matrix)
		saturation = 2.1 # m/s

		K = np.array([[3, 0], [0, 5]]) # Controller Gains

		X = self.position_ # States
		Y = np.array([[X[0][0] + d*np.cos(X[2][0])], [X[1][0] + d*np.sin(X[2][0])]]) # Outs of the system

		ref = self.current_goal_ # References
		diff_ref = np.array([[0.0], [0.0]]) # Reference differentiable

		errors = Y - ref # Errors between the outs and the references

		M_inv = np.array([[np.cos(X[2][0]), np.sin(X[2][0])],
						  [-np.sin(X[2][0]/d), np.cos(X[2][0]/d)]]) # Decoupling matrix

		V = -K@errors # Auxiliar controllers

		u = M_inv @ (diff_ref + V) # Control by auxiliary controls

		# Saturation
		if u[0][0] > saturation:
			u[0][0] = saturation
		if u[0][0] < -saturation:
			u[0][0] = -saturation

		if u[1][0] > saturation:
			u[1][0] = saturation
		if u[1][0] < -saturation:
			u[1][0] = -saturation

		# Close loop system
		G = np.array([[np.cos(X[2][0]), 0], [np.sin(X[2][0]), 0], [0, 1]])

		self.position_ = X + (G @ u) * dt
		self.controls_ = u

		# X = X + (G @ u) * dt
		# self.position_ = X

		self.path_.append([self.position_[0][0], self.position_[1][0], self.position_[2][0]])

	def goal_reached(self):
		
		if self.two_points_euclidian_distance(self.position_, self.current_goal_) <= 1.1:
			print("\tThe goal was reached!! \n \tIt will change...!")
			return True
			
		return False

	def new_goal(self):
		
		x = random.uniform(self.roomLimits_[0][0], self.roomLimits_[0][1])
		y = random.uniform(self.roomLimits_[1][0], self.roomLimits_[1][1])
		
		self.current_goal_ = np.array([[x], [y]])

	def robot_setting(self):

		self.path_.append([self.position_[0][0], self.position_[1][0], self.position_[2][0]])

	# Funcion para forma del usuario
	def Shape(self, pos):

		x = pos[0]
		y = pos[1]
		direction = pos[2]

		p1 = (x + 0.3*np.cos(direction-(np.pi/2+0.1)), y + 0.3*np.sin(direction-(np.pi/2+0.1)))
		p2 = (x + 0.1*np.cos(direction-(np.pi/2)), y + 0.1*np.sin(direction-(np.pi/2)))
		p3 = (x + 0.13*np.cos(direction), y + 0.13*np.sin(direction))
		p5 = (x + 0.3*np.cos(direction+(np.pi/2+0.1)), y + 0.3*np.sin(direction+(np.pi/2+0.1)))
		p4 = (x + 0.1*np.cos(direction+(np.pi/2)), y + 0.1*np.sin(direction+(np.pi/2)))

		shape = shapely.geometry.LineString([p1, p2, p3, p4, p5]).buffer(0.08)

		return shape
		
	def two_points_euclidian_distance(self, p1, p2):
		
		x = p2[0][0] - p1[0][0]
		y = p2[1][0] - p1[1][0]
		
		return np.sqrt(x**2 + y**2)