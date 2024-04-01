# Mostramos el movimientos de los robots

import numpy as np
from collections import deque
import matplotlib.pyplot as plt 
import matplotlib.patches as patches
import matplotlib.cm as cmx
# import matplotlib.colors as colors
import matplotlib.animation as animation
import matplotlib as mpl

from descartes import PolygonPatch

def visualize_dynamic(robots, user, obstacles, animated_time=120): # Falta Texto

	history_len = int(len(robots[0].path_) / 5)
	colors = cmx.rainbow(np.linspace(0, 1, len(robots)))
	side = 0.4
	frames = len(user.path_)

	if len(robots[0].path_) < frames:
		frames = len(robots[0].path_)

	fig1, ax1 = plt.subplots()
	ax1.set_title('Animation of the simulation')
	ax1.set_xlim(-robots[0].roomLimits_[0][1]*0.7, robots[0].roomLimits_[0][1]*1.0)
	ax1.set_ylim(-robots[0].roomLimits_[1][1]*0.7, robots[0].roomLimits_[1][1]*1.0)

	# Generamos los puntos y los trazos
	user_shape = user.Shape((0, 0, 0))
	user_patch = PolygonPatch(user_shape, fc='orange', alpha=0.5, zorder=2)
	user_head = plt.Circle((user.path_[0][0], user.path_[0][1]), 0.2, fc='orange', alpha=5)

	user_text = ax1.text(user.path_[-1][0]+0.2, user.path_[-1][1], 'User', fontsize=10, zorder=3)

	obstacles_polygon = []

	for obs in obstacles:

		obstacles_polygon.append(plt.Circle((obs.position_[0], obs.position_[1]), obs.radius_, fc='black', alpha=5))
		ax1.add_patch(obstacles_polygon[-1])

	trace = []
	histories_x = []
	histories_y = []
	cars = []
	cars_back = []
	views = []
	# circles = []
	texts = []
	for i, robot in enumerate(robots):
		trace.append(ax1.plot([], [], '-', color=colors[i], lw=1)[0])
		histories_x.append(deque(maxlen=history_len))
		histories_y.append(deque(maxlen=history_len))

		texts.append(ax1.text(robot.path_[0][0]+side, robot.path_[0][1], 'ID:%s'%i,
				fontsize=10, fontweight='bold', zorder=3))

		# if i == 0:
		# 	leader_text = ax1.text(robot.path_[0][0]+side-0.2, robot.path_[0][1]+side+0.1, 'Leader',
		# 		fontsize=10, zorder=3)

		car = robot.TriangleShape((0, 0, 0))
		car_back = robot.rectangle_shape((0, 0, 0))
		view = robot.viewShape((0, 0, 0, 0))

		cars.append(PolygonPatch(car, fc=colors[i], alpha=1.0, zorder=2))
		cars_back.append(PolygonPatch(car_back, fc=colors[i], alpha=1.0, zorder=2))
		views.append(PolygonPatch(view, fc='yellow', alpha=0.1, zorder=2))
		# circles.append(plt.Circle((robot.path_[0][0], robot.path_[0][1]), robot.radius_ , fc=colors[i], alpha=0.7))

	# Funciones para animacion
	def init():

		ax1.add_patch(user_patch)
		ax1.add_patch(user_head)

		for i in range(len(robots)):
			ax1.add_patch(cars[i])
			ax1.add_patch(cars_back[i])
			ax1.add_patch(views[i])
			# ax1.add_patch(circles[i])

		return []

	def animate(index):

		# print(index)

		# Posicion de usuario
		# theta = user.path_[index][2] - (np.pi/2)
		theta = user.path_[index][2]
		x = user.path_[index][0]
		y = user.path_[index][1]

		r = mpl.transforms.Affine2D().rotate(theta)
		t = mpl.transforms.Affine2D().translate(x, y)
		tra = r + t + ax1.transData

		user_patch.set_transform(tra)
		user_head.center = (user.path_[index][0], user.path_[index][1])
		user_text.set_position((user.path_[index][0]+0.2, user.path_[index][1]))

		# Posicion de robots
		for i, robot in enumerate(robots):

			this_x = robot.path_[index][0]
			this_y = robot.path_[index][1]

			if index == 0:

				histories_x[i].clear()
				histories_y[i].clear()

			histories_x[i].appendleft(this_x)
			histories_y[i].appendleft(this_y)

			trace[i].set_data(histories_x[i], histories_y[i])
			texts[i].set_position((robot.path_[index][0]+side, robot.path_[index][1]))

			# if robot.leader_[index]:
			# 	leader_text.set_position((robot.path_[index][0]+side-0.2, robot.path_[index][1]+side-1.5))
				# print(robot.id_)
				# input()

			# Transformacion de poligono
			theta = robot.path_[index][2]
			alpha = robot.path_[index][3]
			# theta = robot.path_[index][2] - robot.path_[index-1][2]
			x = robot.path_[index][0]
			y = robot.path_[index][1]

			r = mpl.transforms.Affine2D().rotate(theta)
			r_alpha = mpl.transforms.Affine2D().rotate(alpha)
			t = mpl.transforms.Affine2D().translate(x, y)

			tra = r + t + ax1.transData
			tra_camara = r_alpha + t + ax1.transData

			cars[i].set_transform(tra)		
			cars_back[i].set_transform(tra)		
			views[i].set_transform(tra_camara)		
			# circles[i].center = (robot.path_[index][0], robot.path_[index][1])
		
		# return cars, views, circles, trace, user_patch, user_patch
		return cars, views, cars_back, trace, user_patch, user_patch

	ani = animation.FuncAnimation(fig1, animate, init_func=init, frames=frames, interval=animated_time)
	ani.save('Simulation.mp4', fps=9)
	plt.show()

# End
def visualize_state_4(robots, user, obstacles):

	print('Graficnado...')

	colors = cmx.rainbow(np.linspace(0, 1, len(robots)))
	side = 0.4

	fig, ax = plt.subplots()
	ax.set_title('Multi-robot formation without Obstacles')
	ax.set_xlim(-user.roomLimits_[0][1]*1.2, user.roomLimits_[0][1]*1.2)
	ax.set_ylim(-user.roomLimits_[1][1]*1.2, user.roomLimits_[1][1]*1.2)

	obstacles_polygon = []

	for obs in obstacles:

		obstacles_polygon.append(plt.Circle((obs.position_[0], obs.position_[1]), obs.radius_, fc='black', alpha=5))
		ax.add_patch(obstacles_polygon[-1])

	# Agregando al usuario a la grafica
	user_shape = user.Shape(user.path_[-1])
	user_patch = PolygonPatch(user_shape, fc='orange', alpha=0.5, zorder=2)
	user_head = plt.Circle((user.path_[-1][0], user.path_[-1][1]), 0.2, fc='orange', alpha=5)

	ax.add_patch(user_head)
	ax.add_patch(user_patch)
	ax.text(user.path_[-1][0]+0.2, user.path_[-1][1], 'User', fontsize=10, zorder=3)

	# Agregando los robots a la grafica
	for robotID, robot in enumerate(robots):

		aux_triangle = robot.TriangleShape(robot.path_[-1])
		aux_patch = PolygonPatch(aux_triangle, fc=colors[robotID], alpha=1, zorder=2)
		aux_view = robot.viewShape(robot.path_[-1])
		aux_patch_view = PolygonPatch(aux_view, fc='yellow', alpha=0.1, zorder=2)
		aux_circle = plt.Circle((robot.path_[-1][0], robot.path_[-1][1]), robot.radius_, fc=colors[robotID], alpha=0.7)

		ax.add_patch(aux_patch)
		ax.add_patch(aux_patch_view)
		ax.add_patch(aux_circle)

		# Graficando ID de Robot
		ax.text(robot.path_[-1][0]+side, robot.path_[-1][1], 'ID:%s'%robotID,
				fontsize=10, fontweight='bold', zorder=3)

		# if robot.leader_[-1]:
		# 	ax.text(robot.path_[-1][0]+side-0.2, robot.path_[-1][1]+side+0.1, 'Leader',
		# 		fontsize=10, zorder=3)

	for robot in robots:

		for pos in robot.path_:
			ax.plot(pos[0], pos[1], '.', color='red')

	for point in user.path_:
		ax.plot(point[0], point[1], '.', color='orange')

	plt.show()

# Middle
def visualize_state_3(robots, user, obstacles):

	print('Graficnado...')

	index = int( 2 * len(robots[0].path_) / 3)

	colors = cmx.rainbow(np.linspace(0, 1, len(robots)))
	side = 0.4

	fig, ax = plt.subplots()
	ax.set_title('Multi-robot formation without Obstacles')
	ax.set_xlim(-user.roomLimits_[0][1]*1.2, user.roomLimits_[0][1]*1.2)
	ax.set_ylim(-user.roomLimits_[1][1]*1.2, user.roomLimits_[1][1]*1.2)

	obstacles_polygon = []

	for obs in obstacles:

		obstacles_polygon.append(plt.Circle((obs.position_[0], obs.position_[1]), obs.radius_, fc='black', alpha=5))
		ax.add_patch(obstacles_polygon[-1])

	# Agregando al usuario a la grafica
	user_shape = user.Shape(user.path_[index])
	user_patch = PolygonPatch(user_shape, fc='orange', alpha=0.5, zorder=2)
	user_head = plt.Circle((user.path_[index][0], user.path_[-1][1]), 0.2, fc='orange', alpha=5)

	ax.add_patch(user_head)
	ax.add_patch(user_patch)
	ax.text(user.path_[index][0]+0.2, user.path_[index][1], 'User', fontsize=10, zorder=3)

	# Agregando los robots a la grafica
	for robotID, robot in enumerate(robots):

		aux_triangle = robot.TriangleShape(robot.path_[index])
		aux_patch = PolygonPatch(aux_triangle, fc=colors[robotID], alpha=1, zorder=2)
		aux_view = robot.viewShape(robot.path_[index])
		aux_patch_view = PolygonPatch(aux_view, fc='yellow', alpha=0.1, zorder=2)
		aux_circle = plt.Circle((robot.path_[index][0], robot.path_[index][1]), robot.radius_, fc=colors[robotID], alpha=0.7)

		ax.add_patch(aux_patch)
		ax.add_patch(aux_patch_view)
		ax.add_patch(aux_circle)

		# Graficando ID de Robot
		ax.text(robot.path_[index][0]+side, robot.path_[index][1], 'ID:%s'%robotID,
				fontsize=10, fontweight='bold', zorder=3)

		# if robot.leader_[index]:
		# 	ax.text(robot.path_[index][0]+side-0.2, robot.path_[index][1]+side+0.1, 'Leader',
		# 		fontsize=10, zorder=3)

	for robot in robots:

		for i in range(int(index/2), index):
			ax.plot(robot.path_[i][0], robot.path_[i][1], '.', color='red')

	for i in range(int(index/2), index):
		ax.plot(user.path_[i][0], user.path_[i][1], '.', color='orange')

def visualize_state_2(robots, user, obstacles):

	print('Graficnado...')

	index = int(len(robots[0].path_) / 3)

	colors = cmx.rainbow(np.linspace(0, 1, len(robots)))
	side = 0.4

	fig, ax = plt.subplots()
	ax.set_title('Multi-robot formation without Obstacles')
	ax.set_xlim(-user.roomLimits_[0][1]*1.2, user.roomLimits_[0][1]*1.2)
	ax.set_ylim(-user.roomLimits_[1][1]*1.2, user.roomLimits_[1][1]*1.2)

	obstacles_polygon = []

	for obs in obstacles:

		obstacles_polygon.append(plt.Circle((obs.position_[0], obs.position_[1]), obs.radius_, fc='black', alpha=5))
		ax.add_patch(obstacles_polygon[-1])

	# Agregando al usuario a la grafica
	user_shape = user.Shape(user.path_[index])
	user_patch = PolygonPatch(user_shape, fc='orange', alpha=0.5, zorder=2)
	user_head = plt.Circle((user.path_[index][0], user.path_[-1][1]), 0.2, fc='orange', alpha=5)

	ax.add_patch(user_head)
	ax.add_patch(user_patch)
	ax.text(user.path_[index][0]+0.2, user.path_[index][1], 'User', fontsize=10, zorder=3)

	# Agregando los robots a la grafica
	for robotID, robot in enumerate(robots):

		aux_triangle = robot.TriangleShape(robot.path_[index])
		aux_patch = PolygonPatch(aux_triangle, fc=colors[robotID], alpha=1, zorder=2)
		aux_view = robot.viewShape(robot.path_[index])
		aux_patch_view = PolygonPatch(aux_view, fc='yellow', alpha=0.1, zorder=2)
		aux_circle = plt.Circle((robot.path_[index][0], robot.path_[index][1]), robot.radius_, fc=colors[robotID], alpha=0.7)

		ax.add_patch(aux_patch)
		ax.add_patch(aux_patch_view)
		ax.add_patch(aux_circle)

		# Graficando ID de Robot
		ax.text(robot.path_[index][0]+side, robot.path_[index][1], 'ID:%s'%robotID,
				fontsize=10, fontweight='bold', zorder=3)

		# if robot.leader_[index]:
		# 	ax.text(robot.path_[index][0]+side-0.2, robot.path_[index][1]+side+0.1, 'Leader',
		# 		fontsize=10, zorder=3)

	for robot in robots:

		for i in range(int(index/2), index):
			ax.plot(robot.path_[i][0], robot.path_[i][1], '.', color='red')

	for i in range(int(index/2), index):
		ax.plot(user.path_[i][0], user.path_[i][1], '.', color='orange')

# Beggin
def visualize_state_1(robots, user, obstacles):

	print('Graficnado...')

	# index = int(len(robots[0].path_) / 6)
	index = int(0)

	colors = cmx.rainbow(np.linspace(0, 1, len(robots)))
	side = 0.4

	fig, ax = plt.subplots()
	ax.set_title('Multi-robot formation without Obstacles')
	ax.set_xlim(-user.roomLimits_[0][1]*1.2, user.roomLimits_[0][1]*1.2)
	ax.set_ylim(-user.roomLimits_[1][1]*1.2, user.roomLimits_[1][1]*1.2)

	obstacles_polygon = []

	for obs in obstacles:

		obstacles_polygon.append(plt.Circle((obs.position_[0], obs.position_[1]), obs.radius_, fc='black', alpha=5))
		ax.add_patch(obstacles_polygon[-1])

	# Agregando al usuario a la grafica
	user_shape = user.Shape(user.path_[index])
	user_patch = PolygonPatch(user_shape, fc='orange', alpha=0.5, zorder=2)
	user_head = plt.Circle((user.path_[index][0], user.path_[index][1]), 0.2, fc='orange', alpha=5)

	ax.add_patch(user_head)
	ax.add_patch(user_patch)
	ax.text(user.path_[index][0]+0.2, user.path_[index][1], 'User', fontsize=10, zorder=3)

	# Agregando los robots a la grafica
	for robotID, robot in enumerate(robots):

		aux_triangle = robot.TriangleShape(robot.path_[index])
		aux_patch = PolygonPatch(aux_triangle, fc=colors[robotID], alpha=1, zorder=2)
		aux_view = robot.viewShape(robot.path_[index])
		aux_patch_view = PolygonPatch(aux_view, fc='yellow', alpha=0.1, zorder=2)
		aux_circle = plt.Circle((robot.path_[index][0], robot.path_[index][1]), robot.radius_, fc=colors[robotID], alpha=0.7)

		ax.add_patch(aux_patch)
		ax.add_patch(aux_patch_view)
		ax.add_patch(aux_circle)

		# Graficando ID de Robot
		ax.text(robot.path_[index][0]+side, robot.path_[index][1], 'ID:%s'%robotID,
				fontsize=10, fontweight='bold', zorder=3)

		# if robot.leader_[index]:
		# 	ax.text(robot.path_[index][0]+side-0.2, robot.path_[index][1]+side+0.1, 'Leader',
		# 		fontsize=10, zorder=3)

	for robot in robots:

		for i in range(index):
			ax.plot(robot.path_[i][0], robot.path_[i][1], '.', color='red')

	for i in range(index):
		ax.plot(user.path_[i][0], user.path_[i][1], '.', color='orange')

	plt.show()