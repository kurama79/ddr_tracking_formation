import numpy as np
import shapely.geometry

class Robot_DDR(object):

	def __init__(self, id, theta, position, distance):

		# Initial values
		self.id_ = id 
		self.radius_ = 0.4 # Medida propuesta 0.3 pero se suma 0.1 por rango
		self.theta_ref_ = self.id_ * theta
		self.formation_distance_ = distance

		# Variables
		self.position_ = position
		self.control_ = np.array([[0.0], [0.0]])
		self.current_reference_ = np.array([[0.0, 0.0], [0.0, 0.0]]) # [[pos_x, vel_x], [pos_y, vel_y]]

		# Data to save
		self.path_ = [[position[0][0], position[1][0], position[2][0]]]

	def tracking_control(self, dt):

		'''
			We generate reference based on displacement-based formation control

				robot.position = user.position + ref
		'''

		# Gain K
		K = np.array([[2, 3, 0, 0],
				  	  [0, 0, 4, 5]])

		X = self.position_
		Y = np.array([[X[0][0]],
				  	  [X[1][0]]])

		ref = self.current_reference_[:,0:1]
		diff_ref = self.current_reference_[:,1:2]
		diff2_ref = 0.0001

		# Errors
		e1 = Y[0][0] - ref[0][0]
		e2 = X[3][0] * np.cos(X[2][0]) - diff_ref[0][0]
		e3 = Y[1][0] - ref[1][0]
		e4 = X[3][0] * np.sin(X[2][0]) - diff_ref[1][0]

		errors = np.array([[e1],
						   [e2],
						   [e3],
						   [e4]])

		M_inv = np.array([[-np.sin(X[2][0])/X[3][0], np.cos(X[2][0])/X[3][0]],
						  [np.cos(X[2][0]), np.sin(X[2][0])]])

		V = -K@errors # Auxiliar controllers

		u = M_inv @ (diff2_ref + V) # Control by auxiliary controls

		# Close loop system
		F = np.array([[X[3][0] * np.cos(X[2][0])],
					  [X[3][0] * np.sin(X[2][0])],
					  [0],
					  [0]])
		G = np.array([[0, 0],
				  	  [0, 0],
				  	  [1, 0],
				  	  [0, 1]])

		cicles = X[2][0] / (2 * np.pi)
		intCicles = int(cicles)
		rest = cicles - intCicles
		X[2][0] = 2 * np.pi * rest

		self.position_ = X + (F + (G @ u)) * dt
		self.control_ = u
		self.path_.append([self.position_[0][0], self.position_[1][0], self.position_[2][0]])

	def set_references(self, user):

		# Set the references based on the user data

		pos_x = user.position_[0][0] + self.formation_distance_ * np.cos(self.theta_ref_)
		vel_x = user.controls_[0][0] * np.cos(user.position_[2][0])

		pos_y = user.position_[1][0] + self.formation_distance_ * np.sin(self.theta_ref_)
		vel_y = user.controls_[0][0] * np.sin(user.position_[2][0])

		self.current_reference_ = np.array([[pos_x, vel_x], [pos_y, vel_y]])

	# Forma del robot
	def TriangleShape(self, position):

		p1 = [position[0] + self.radius_*np.cos(position[2]), position[1] + self.radius_*np.sin(position[2])]
		p2 = [position[0] + self.radius_*np.cos(position[2]-(np.pi/2)), position[1] + self.radius_*np.sin(position[2]-(np.pi/2))]
		p3 = [position[0] + self.radius_*np.cos(position[2]+(np.pi/2)), position[1] + self.radius_*np.sin(position[2]+(np.pi/2))]
		shape = shapely.geometry.Polygon([(p1[0], p1[1]), (p2[0], p2[1]), (p3[0], p3[1])])

		return shape

	# Funcion distancia
	def distance(self, p1, p2):
		return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

class Robot_DDR_Camera(object):

	def __init__(self, id, theta, position, distance, cameraRotation):

		# Initial values
		self.id_ = id 
		self.radius_ = 0.4 # Medida propuesta 0.3 pero se suma 0.1 por rango
		self.theta_ref_ = self.id_ * theta
		self.formation_distance_ = distance
		self.camera_rotation_ = cameraRotation
		self.camera_view_range_ = np.pi / 6

		# Variables
		self.position_ = position
		self.control_ = np.array([[0.0], [0.0]])
		self.current_reference_ = np.array([[0.0], [0.0], [0.0]]) # [[pos_x], [pos_y], [cam_dir]]
		self.camara_blind_ = False

		# Data to save
		self.path_ = [[position[0][0], position[1][0], position[2][0], position[3][0]]]

	def tracking_control(self, dt):

		'''
			We generate reference based on displacement-based formation control

				robot.position = user.position + ref
		'''

		d = 1.0 # Control point to generate an invertible matrix (decoupling matrix)

		# Gain K
		K = np.array([[2, 0, 0],
				  	  [0, 3, 0],
					  [0, 0, 4]])

		X = self.position_
		Y = np.array([[X[0][0]],
				  	  [X[1][0]],
					  [X[3][0]]])

		ref = np.array([[self.current_reference_[0][0]], [self.current_reference_[0][1]], [self.current_reference_[0][2]]])
		diff_ref = np.array([[self.current_reference_[1][0]], [self.current_reference_[1][1]], [0.0]])

		# Errors
		errors = Y - ref

		M_inv = np.array([[np.cos(X[2][0]), np.sin(X[2][0]), 0],
						  [-np.sin(X[2][0])/d, np.cos(X[2][0])/d, 0],
						  [np.sin(X[2][0])/d, -np.cos(X[2][0])/d, 1]])

		V = -K@errors # Auxiliar controllers

		u = M_inv @ (diff_ref + V) # Control by auxiliary controls

		# Close loop system
		G = np.array([[np.cos(X[2][0]), 0, 0],
				  	  [np.sin(X[2][0]), 0, 0],
				  	  [0, 1, 0],
				  	  [0, 1, 1]])

		X = X + (G @ u) * dt

		cicles = X[2][0] / (2 * np.pi)
		intCicles = int(cicles)
		rest = cicles - intCicles
		X[2][0] = 2 * np.pi * rest

		# print("Diferencia: ", abs(X[3][0] - X[2][0]))
		# print("Rotacion camara: ", self.camera_rotation_)

		# if abs(X[3][0] - X[2][0]) >= self.camera_rotation_:

		# 	X[3][0] = (X[2][0] + self.camera_rotation_) * ((abs(X[3][0] - X[2][0])) / (X[3][0] - X[2][0]))

		# 	# Asking if the user is in the camera view range
		# 	if abs(ref[2][0] - X[3][0]) <= self.camera_view_range_:
		# 		self.camara_blind_ = False
			
		# 	else:
				
		# 		self.camara_blind_ = True
		# 		# Reset Camera
		# 		X[3][0] = X[2][0]
		# 		self.leader_ = False

		self.position_ = X
		self.control_ = u
		self.path_.append([self.position_[0][0], self.position_[1][0], self.position_[2][0], self.position_[3][0]])

	def set_references(self, user):

		# Set the references based on the user data

		pos_x = user.position_[0][0] + self.formation_distance_ * np.cos(self.theta_ref_)
		vel_x = user.controls_[0][0] * np.cos(user.position_[2][0])

		pos_y = user.position_[1][0] + self.formation_distance_ * np.sin(self.theta_ref_)
		vel_y = user.controls_[0][0] * np.sin(user.position_[2][0])

		alpha = self.theta_ref_ + np.pi

		self.current_reference_ = [[pos_x, pos_y, alpha], [vel_x, vel_y]]

	# Forma del robot
	def TriangleShape(self, position):

		p1 = [position[0] + self.radius_*np.cos(position[2]), position[1] + self.radius_*np.sin(position[2])]
		p2 = [position[0] + self.radius_*np.cos(position[2]-(np.pi/2)), position[1] + self.radius_*np.sin(position[2]-(np.pi/2))]
		p3 = [position[0] + self.radius_*np.cos(position[2]+(np.pi/2)), position[1] + self.radius_*np.sin(position[2]+(np.pi/2))]
		shape = shapely.geometry.Polygon([(p1[0], p1[1]), (p2[0], p2[1]), (p3[0], p3[1])])

		return shape

	# Funcion distancia
	def distance(self, p1, p2):
		return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

# Agregando el control con matriz pseudoinversa #######################################################################################################################
class Robot_DDR_Pseudo_Camera(object):

	def __init__(self, id, theta, position, distance, cameraRotation, roomLimits, neighborId):

		# Initial values
		self.id_ = id 
		self.roomLimits_ = roomLimits
		self.radius_ = 0.4 # Medida propuesta 0.3 pero se suma 0.1 por rango
		self.view_radius_ = 3.5
		# self.theta_ref_ = self.id_ * theta
		self.theta_ref_ = theta
		self.formation_distance_ = distance
		self.camera_rotation_ = cameraRotation
		self.camera_view_range_ = np.pi / 6
		self.theta_field_ = theta / 2
		self.wheel_radius = 0.5
		self.h_ = 0.7
		self.d_ = 1.0
		self.neighbor_id_ = neighborId

		# Variables
		self.position_ = position
		self.control_ = np.array([[0.0], [0.0]])
		# self.current_reference_ = np.array([[0.0], [0.0], [0.0]]) # [[pos_x], [pos_y], [cam_dir]]
		# self.camara_blind_ = False
		self.leader_ = [True]
		self.current_reference_ = [[1.0, 1.0, 1.0, 1.0], [1.0, 1.0, 1.0]]
		self.collision_ = False

		# Data to save
		self.path_ = [[position[0][0], position[1][0], position[2][0], position[3][0]]]
		self.path_ = []
		self.controls_ = []
		self.errors_ = [[0.0, 0.0, 0.0]]

	def polar_position_controller(self, user, neighbor, dt, n, v_avoid):

		'''
			DDR controller based on polar coordinates and control point to avoid zero dynamic of the model
		'''
		if not self.collision_:

			# States
			x_disp = self.position_[0][0] - user.position_[0][0]
			y_disp = self.position_[1][0] - user.position_[1][0]

			# Neighbor States
			x_neighbor = neighbor.position_[0][0] - user.position_[0][0]
			y_neighbor = neighbor.position_[1][0] - user.position_[1][0]
			aux_angle = neighbor.position_[2][0]

			# Polar coordinates
			r_robot = np.sqrt(x_disp**2 + y_disp**2)
			r_neighbor = np.sqrt(x_neighbor**2 + y_neighbor**2)

			theta_robot = np.arctan2(y_disp, x_disp)
			theta_neighbor = np.arctan2(y_neighbor, x_neighbor)

			v_1 = np.array([x_disp, y_disp])
			v_2 = np.array([x_neighbor, y_neighbor])

			if n == 2:
				alpha = 1 / (np.cos( np.dot(v_1, v_2) / (np.linalg.norm(v_1)*np.linalg.norm(v_2)) ))
			else:
				alpha = 2 * np.arctan2( np.cross(v_1, v_2), (np.linalg.norm(v_1)*np.linalg.norm(v_2)) + np.dot(v_1, v_2) )

			# The outputs of the system are the polar coordinate then
			X = np.array([[self.position_[0][0]],
						[self.position_[1][0]],
						[self.position_[2][0]]])

			Y = np.array([[r_robot],
						[alpha]])

			# References
			ref = np.array([[self.formation_distance_],
							[self.theta_ref_]])

			# Gains
			K = np.array([[5, 0],
						[0, 5]])

			# New variables to describe the dynamic
			# r_a = [(x_disp / r_robot) * ((self.wheel_radius * np.cos(X[2][0]) / 2) - (self.h_ * self.wheel_radius * np.sin(X[2][0]) / self.d_)) + 
			# 	   (y_disp / r_robot) * ((self.wheel_radius * np.sin(X[2][0]) / 2) + (self.h_ * self.wheel_radius * np.cos(X[2][0]) / self.d_))]
			r_a = [ (x_disp / r_robot) * np.cos(X[2][0]) + (y_disp / r_robot) * np.sin(X[2][0]) ]

			# r_b = [(x_disp / r_robot) * ((self.wheel_radius * np.cos(X[2][0]) / 2) + (self.h_ * self.wheel_radius * np.sin(X[2][0]) / self.d_)) + 
			# 	   (y_disp / r_robot) * ((self.wheel_radius * np.sin(X[2][0]) / 2) - (self.h_ * self.wheel_radius * np.cos(X[2][0]) / self.d_))]
			r_b = [ self.h_ * ((y_disp / r_robot) * np.cos(X[2][0]) - (x_disp / r_robot) * np.sin(X[2][0])) ]

			# Theta_a = [(-y_disp / r_robot**2) * ((self.wheel_radius * np.cos(X[2][0]) / 2) - (self.h_ * self.wheel_radius * np.sin(X[2][0]) / self.d_)) + 
			# 	       (x_disp / r_robot**2) * ((self.wheel_radius * np.sin(X[2][0]) / 2) + (self.h_ * self.wheel_radius * np.cos(X[2][0]) / self.d_))]
			Theta_a = [ (x_disp / r_robot**2) * np.sin(X[2][0]) - (y_disp / r_robot**2) * np.cos(X[2][0]) ]

			# Theta_b = [(-y_disp / r_robot**2) * ((self.wheel_radius * np.cos(X[2][0]) / 2) + (self.h_ * self.wheel_radius * np.sin(X[2][0]) / self.d_)) + 
			# 	       (x_disp / r_robot**2) * ((self.wheel_radius * np.sin(X[2][0]) / 2) - (self.h_ * self.wheel_radius * np.cos(X[2][0]) / self.d_))]
			Theta_b = [ self.h_ * (x_disp / r_robot**2) * np.cos(X[2][0]) + (y_disp / r_robot**2) * np.sin(X[2][0]) ]

			# Theta_a_neighbor = [(-y_neighbor / r_neighbor**2) * ((self.wheel_radius * np.cos(aux_angle) / 2) - (self.h_ * self.wheel_radius * np.sin(aux_angle) / self.d_)) + 
			# 	                (x_neighbor / r_neighbor**2) * ((self.wheel_radius * np.sin(aux_angle) / 2) + (self.h_ * self.wheel_radius * np.cos(aux_angle) / self.d_))]
			Theta_a_neighbor = [ (x_neighbor / r_neighbor**2) * np.sin(X[2][0]) - (y_neighbor / r_neighbor**2) * np.cos(X[2][0]) ]

			# Theta_b_neighbor = [(-y_neighbor / r_neighbor**2) * ((self.wheel_radius * np.cos(aux_angle) / 2) + (self.h_ * self.wheel_radius * np.sin(aux_angle) / self.d_)) + 
			# 	       			(x_neighbor / r_neighbor**2) * ((self.wheel_radius * np.sin(aux_angle) / 2) - (self.h_ * self.wheel_radius * np.cos(aux_angle) / self.d_))]
			Theta_b_neighbor = [ self.h_ * (x_neighbor / r_neighbor**2) * np.cos(X[2][0]) + (y_neighbor / r_neighbor**2) * np.sin(X[2][0]) ]

			# Errors
			errors = Y - ref

			# Inverse M matrix
			M_inv = np.array([[(Theta_b_neighbor[0] - Theta_b[0])/(r_a[0] * (Theta_b_neighbor[0] - Theta_b[0]) - r_b[0] * (Theta_a_neighbor[0] - Theta_a[0])), (r_b[0])/(r_b[0] * (Theta_a_neighbor[0] - Theta_a[0]) - r_a[0] * (Theta_b_neighbor[0] - Theta_b[0]))],
							[(Theta_a_neighbor[0] - Theta_a[0])/(r_b[0] * (Theta_a_neighbor[0] - Theta_a[0]) - r_a[0] * (Theta_b_neighbor[0] - Theta_b[0])), (r_a[0])/(r_a[0] * (Theta_b_neighbor[0] - Theta_b[0]) - r_b[0] * (Theta_a_neighbor[0] - Theta_a[0]))]])

			# Control
			v = -K @ errors # Auxiliar
			u = M_inv @ v # Control

			self.errors_.append([errors[0][0], errors[1][0], 0.0])

		else:

			# print('Evasion!!!')
			# input()

			X = np.array([[self.position_[0][0]],
						[self.position_[1][0]],
						[self.position_[2][0]]])

			K = np.array([[5, 0],
						[0, 5]])

			u = np.array([[v_avoid[0] * np.cos(X[2][0]) + v_avoid[1] * np.sin(X[2][0])],
						  [(1 / self.d_) * (-v_avoid[0] * np.sin(X[2][0]) + v_avoid[1] * np.cos(X[2][0]))]])

			self.collision_ = False

		u = np.array([[1/self.wheel_radius, self.d_/(2*self.wheel_radius)], [1/self.wheel_radius, -self.d_/(2*self.wheel_radius)]]) @ u

		# Close loop system
		G = np.array([[ ((self.wheel_radius * np.cos(X[2][0])) / 2) - ((self.h_ * self.wheel_radius * np.sin(X[2][0])) / self.d_),  ((self.wheel_radius * np.cos(X[2][0])) / 2) + ((self.h_ * self.wheel_radius * np.sin(X[2][0])) / self.d_) ],
					  [ ((self.wheel_radius * np.sin(X[2][0])) / 2) + ((self.h_ * self.wheel_radius * np.cos(X[2][0])) / self.d_),  ((self.wheel_radius * np.sin(X[2][0])) / 2) - ((self.h_ * self.wheel_radius * np.cos(X[2][0])) / self.d_) ],
					  [ self.wheel_radius / self.d_, -self.wheel_radius / self.d_ ]])
		# G = np.array([ [np.cos(X[2][0]), -self.h_*np.sin(X[2][0])],
		# 			   [np.sin(X[2][0]), self.h_*np.cos(X[2][0])],
		# 			   [0, 1] ])

		X = X + (G @ u) * dt

		self.position_[0][0] = X[0][0]
		self.position_[1][0] = X[1][0]
		self.position_[2][0] = X[2][0]

		self.controls_.append([u[0][0], u[1][0], 0.0])

	def camera_control(self, user, dt):

		'''
			Rotational Camera Control
		'''

		viewAngle = self.position_[3][0]

		omgeaRobot = (self.wheel_radius / self.d_) * (self.controls_[-1][0] - self.controls_[-1][1])
		vRobot = (self.wheel_radius/2)*(self.controls_[-1][0] + self.controls_[-1][1])

		x = user.position_[0][0] - self.position_[0][0]
		y = user.position_[1][0] - self.position_[1][0]
		r = x**2 + y**2

		ref = np.arctan2(y, x)
		ref_diff = (x/r)*(vRobot*np.sin(self.position_[2][0]) + omgeaRobot*self.d_*np.cos(self.position_[2][0])) - (y/r)*(vRobot*np.cos(self.position_[2][0]) - omgeaRobot*self.d_*np.sin(self.position_[2][0]))
		# ref_diff = 0

		# Gain 
		k = 20

		e = viewAngle - ref

		u = -(k*e) + ref_diff

		viewAngle = viewAngle + (omgeaRobot + u) * dt
		# viewAngle = viewAngle + (u) * dt

		if abs(e) > 2.0:
				e = self.errors_[-1][-1] * (e/abs(e))

		self.position_[3][0] = viewAngle
		self.controls_[-1][-1] = u
		self.errors_[-1][-1] = e
		self.path_.append([self.position_[0][0], self.position_[1][0], self.position_[2][0], self.position_[3][0]])

		print('Error: ', self.errors_[-1])

	def tracking_control(self, dt):

		'''
			We generate reference based on displacement-based formation control

				robot.position = user.position + ref
		'''

		d = 1.0 # Control point to generate an invertible matrix (decoupling matrix)

		# Gain K
		K = np.array([[4, 0, 0, 0],
				  	  [0, 4, 0, 0],
					  [0, 0, 4, 0],
					  [0, 0, 0, 5]])

		X = self.position_
		Y = np.array([[X[0][0]],
				  	  [X[1][0]],
					  [X[2][0]],
					  [X[3][0]]])

		ref = np.array([[self.current_reference_[0][0]], [self.current_reference_[0][1]], [self.current_reference_[0][2]], [self.current_reference_[0][3]]])
		diff_ref = np.array([[self.current_reference_[1][0]], [self.current_reference_[1][1]], [self.current_reference_[1][2]], [0.0]])

		# Errors
		errors = Y - ref

		M_inv = np.array([[np.cos(X[2][0]), np.sin(X[2][0]), 0, 0],
						  [0, 0, 1, 0],
						  [0, 0, -1, 1]])

		V = -K@errors # Auxiliar controllers

		u = M_inv @ (diff_ref + V) # Control by auxiliary controls

		# Close loop system
		G = np.array([[np.cos(X[2][0]), 0, 0],
				  	  [np.sin(X[2][0]), 0, 0],
				  	  [0, 1, 0],
					  [0, 1, 1]])

		X = X + (G @ u) * dt

		cicles = X[2][0] / (2 * np.pi)
		intCicles = int(cicles)
		rest = cicles - intCicles
		X[2][0] = 2 * np.pi * rest

		self.position_ = X
		self.control_ = u

		self.path_.append([self.position_[0][0], self.position_[1][0], self.position_[2][0], self.position_[3][0]])
		self.controls_.append([u[0][0], u[1][0], u[2][0]])
		self.errors_.append([errors[0][0], errors[1][0], errors[2][0], errors[3][0]])

		# self.leader_.append(False)

	def set_references(self, user):

		# Set the references based on the user data

		pos_x = user.position_[0][0] + self.formation_distance_ * np.cos(self.theta_ref_)
		vel_x = user.controls_[0][0] * np.cos(user.position_[2][0])

		pos_y = user.position_[1][0] + self.formation_distance_ * np.sin(self.theta_ref_)
		vel_y = user.controls_[0][0] * np.sin(user.position_[2][0])

		omega = user.controls_[1][0]

		# theta = user.position_[2][0]
		theta = self.theta_ref_ + np.pi

		cicles = theta / (2 * np.pi)
		intCicles = int(cicles)
		rest = cicles - intCicles
		theta = 2 * np.pi * rest

		# alpha = self.theta_ref_ + np.pi
		alpha = np.arctan2( user.position_[1][0] - self.position_[1][0], user.position_[0][0] - self.position_[0][0] )
		
		if abs(alpha - self.position_[3][0]) > 3.14:
			
			if np.sign(alpha) > 0:
				alpha = alpha - (2 * np.pi)
			
			else:
				alpha = (2 * np.pi) + abs(alpha)

		self.current_reference_ = [[pos_x, pos_y, theta, alpha], [vel_x, vel_y, omega]]

	def get_attraction_force(self, user, att=1.2):

		# goal = [self.current_reference_[0][0] + self.current_reference_[1][0]*2.0, self.current_reference_[0][1] + self.current_reference_[1][1]*2.0]

		x_aux = self.position_[0][0] - user.position_[0][0]
		y_aux = self.position_[1][0] - user.position_[1][0]
		theta = np.arctan2(y_aux, x_aux)

		x = user.position_[0][0] + self.formation_distance_*np.cos(theta)
		y = user.position_[1][0] + self.formation_distance_*np.sin(theta)

		goal = [x, y]

		delta = np.array(goal) - np.array([self.position_[0][0], self.position_[1][0]])
		forceMagnitude = att * np.linalg.norm(delta)
		forceDirection = delta / np.linalg.norm(delta)
		force = forceMagnitude * forceDirection

		# sigma = 3.0

		# dist_value = -( 1 / (sigma * np.sqrt(2 * np.pi))) * np.exp(-(self.distance(goal, position)**2 / (2 * sigma * sigma)) )

		# return dist_value

		return force

	def evasion_moves(self, dt):

		angle_step = 2 * np.pi / 6
		angle = - angle_step
		self.evasions_ = []

		for _ in range(6):

			angle += angle_step
			self.evasions_.append([1 * np.cos(angle) + self.position_[0][0], 1 * np.sin(angle) + self.position_[1][0]])

	def set_leader(self, user):

		alpha = self.theta_ref_
		alpha_cicle = alpha - 2 * np.pi
		thetaUser = user.position_[2][0]

		cicles = thetaUser / (2 * np.pi)
		intCicles = int(cicles)
		rest = cicles - intCicles
		thetaUser = 2 * np.pi * rest

		dif = abs(alpha - thetaUser)
		dif_cicle = abs(alpha_cicle - thetaUser)
		# sig = np.sign(thetaUser - alpha)

		if dif <= self.theta_field_ or dif_cicle < self.theta_field_:
			self.leader_.append(True)
		
		elif dif == self.theta_field_ or dif_cicle == self.theta_field_:
			
			self.leader_.append(True)
			self.position_[3][0] += sig * 0.1

		else:
			self.leader_.append(False)

	def TriangleShape(self, position):

		p1 = [position[0] + self.radius_*np.cos(position[2]), position[1] + self.radius_*np.sin(position[2])]
		p2 = [position[0] + self.radius_*np.cos(position[2]-(np.pi/2)), position[1] + self.radius_*np.sin(position[2]-(np.pi/2))]
		p3 = [position[0] + self.radius_*np.cos(position[2]+(np.pi/2)), position[1] + self.radius_*np.sin(position[2]+(np.pi/2))]
		shape = shapely.geometry.Polygon([(p1[0], p1[1]), (p2[0], p2[1]), (p3[0], p3[1])])

		return shape

	def rectangle_shape(self, position):

		center_x = position[0] - self.h_*np.cos(position[2])
		center_y = position[1] - self.h_*np.sin(position[2])

		p1 = [center_x + self.radius_*np.cos(position[2]-(np.pi/2)), center_y + self.radius_*np.sin(position[2]-(np.pi/2))]
		p2 = [center_x + self.radius_*np.cos(position[2]+(np.pi/2)), center_y + self.radius_*np.sin(position[2]+(np.pi/2))]
		p3 = [position[0] + self.radius_*np.cos(position[2]-(np.pi/2)), position[1] + self.radius_*np.sin(position[2]-(np.pi/2))]
		p4 = [position[0] + self.radius_*np.cos(position[2]+(np.pi/2)), position[1] + self.radius_*np.sin(position[2]+(np.pi/2))]

		shape = shapely.geometry.Polygon([(p1[0], p1[1]), (p3[0], p3[1]), (p4[0], p4[1]), (p2[0], p2[1])])

		return shape

	def viewShape(self, position):

		p1 = [position[0], position[1]]
		p2 = [position[0] + self.view_radius_*np.cos(position[3]-self.camera_view_range_), position[1] + self.view_radius_*np.sin(position[3]-self.camera_view_range_)]
		p3 = [position[0] + self.view_radius_*np.cos(position[3]+self.camera_view_range_), position[1] + self.view_radius_*np.sin(position[3]+self.camera_view_range_)]
		shape = shapely.geometry.Polygon([(p1[0], p1[1]), (p2[0], p2[1]), (p3[0], p3[1])])

		return shape

	# Funcion distancia
	def distance(self, p1, p2):
		return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)