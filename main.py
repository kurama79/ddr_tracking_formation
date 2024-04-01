'''
User follower by multi-robots formation.

	V-2.0 - Improving the formation control to ensure that the robot keep the formation.
		- A non-linear controler to follow a reference is added to the DDRs.

By L. Enrique Ruiz-Fernandez - 06/2023
'''

import numpy as np
from copy import copy

import robot
import plot

from simulator import Simulator

# Simulator parameters
mapDimensions = np.array([[-6, 15], [-6, 15]])
deltaTime = 0.05
numRobots = 3 # Number of robots
formationDistance = 3.0

simulator = Simulator(mapDimensions, deltaTime, numRobots, formationDistance)

def auxiliar_plot(simulator, n):

	import matplotlib.pyplot as plt 
	import matplotlib.cm as cmx

	if n == 1:

		fig_user, ax_user = plt.subplots()
		ax_user.set_title('Solo estados el ususario')
		ax_user.set_xlim(simulator.map_dimensions_[0][0], simulator.map_dimensions_[0][1])
		ax_user.set_ylim(simulator.map_dimensions_[1][0], simulator.map_dimensions_[1][1])

		for pos in simulator.user_.path_:
			ax_user.plot(pos[0], pos[1], '.', color='red')
			print("The direction of the user is: ", pos[2])

	if n == 2:

		fig_user, ax_user = plt.subplots()
		ax_user.set_title('usuario y robots')
		ax_user.set_xlim(simulator.map_dimensions_[0][0]*1.2, simulator.map_dimensions_[0][1]*1.2)
		ax_user.set_ylim(simulator.map_dimensions_[1][0]*1.2, simulator.map_dimensions_[1][1]*1.2)

		for pos in simulator.user_.path_:
			ax_user.plot(pos[0], pos[1], '.', color='black')

		colors = cmx.rainbow(np.linspace(0, 1, len(simulator.robots_)))
		for i, robot in enumerate(simulator.robots_):
			for pos in robot.path_:
				ax_user.plot(pos[0], pos[1], '.', color=colors[i])

	if n == 3:

		# fig_user, ax_user = plt.subplots()
		# ax_user.set_title('Target and Robots')
		# ax_user.set_xlim(simulator.map_dimensions_[0][0]*1.2, simulator.map_dimensions_[0][1]*1.2)
		# ax_user.set_ylim(simulator.map_dimensions_[1][0]*1.2, simulator.map_dimensions_[1][1]*1.2)
		# ax_user.set_xlabel('x Coord')
		# ax_user.set_ylabel('y Coord')

		# obstacle = plt.Circle((simulator.obstacles_[0].position_[0], simulator.obstacles_[0].position_[1]), simulator.obstacles_[0].radius_, fc='black', alpha=1.0)
		# ax_user.add_patch(obstacle)

		# fig_v, ax_v = plt.subplots()
		# ax_v.set_title('Controles lineales')
		# fig_w, ax_w = plt.subplots()
		# ax_w.set_title('Controles angulares')
		# fig_c, ax_c = plt.subplots()
		# ax_c.set_title('Controles camara')

		fig_e, ax_e = plt.subplots(3, 1)

		# Axes settings
		fig_e.suptitle('Errors')
		ax_e[2].set_xlabel('time (10^-2 sec)')
		ax_e[0].set_ylabel('e_r')
		ax_e[1].set_ylabel('e_a')
		ax_e[2].set_ylabel('e_c')
		ax_e[0].set_xlim(-1.0, len(simulator.robots_[0].errors_)+1.0)
		ax_e[1].set_xlim(-1.0, len(simulator.robots_[0].errors_)+1.0)
		ax_e[2].set_xlim(-1.0, len(simulator.robots_[0].errors_)+1.0)
		ax_e[0].set_ylim(-10, 10)
		ax_e[1].set_ylim(-4, 4)
		ax_e[2].set_ylim(-4, 4)
		# fig_e, ax_ey = plt.subplots()
		# ax_ey.set_title('Error en y')
		# fig_eth, ax_eth = plt.subplots()
		# ax_eth.set_title('Error en theta')
		# fig_ealp, ax_ealp = plt.subplots()
		# ax_ealp.set_title('Alpha errors')

		# ax_user.plot(simulator.user_.path_[0][0], simulator.user_.path_[0][1], '.', color='black', label='Target')
		# for pos in simulator.user_.path_:
		# 	ax_user.plot(pos[0], pos[1], '.', color='black')

		colors = cmx.rainbow(np.linspace(0, 1, len(simulator.robots_)*3))
		# labels = []
		# for i in range(len(simulator.robots_)):
		# 	labels.append('DDR {}'.format(i+1)) 
			
		for i, robot in enumerate(simulator.robots_):

			# ax_user.plot(robot.path_[0][0], robot.path_[0][1], '.', color=colors[i], label=labels[i])

			# for pos in robot.path_:

			# 	# x = pos[0] + 0.5 * np.cos(simulator.cameras_[i].direction_)
			# 	# y = pos[1] + 0.5 * np.sin(simulator.cameras_[i].direction_)

			# 	x = pos[0] + 0.5 * np.cos(pos[3])
			# 	y = pos[1] + 0.5 * np.sin(pos[3])
				
			# 	x_r = pos[0] + 0.5 * np.cos(pos[2])
			# 	y_r = pos[1] + 0.5 * np.sin(pos[2])

				# ax_user.plot([pos[0], x_r], [pos[1], y_r], '-', color='k')

				# x_r = pos[0] + 0.5 * np.cos(simulator.cameras_[i].current_reference_)
				# y_r = pos[1] + 0.5 * np.sin(simulator.cameras_[i].current_reference_)

				# x_r = pos[0] + 0.5 * np.cos(robot.current_reference_[0][3])
				# y_r = pos[1] + 0.5 * np.sin(robot.current_reference_[0][3])
				
				# ax_user.plot(pos[0], pos[1], '.', color=colors[i])
				# ax_user.plot([pos[0], x], [pos[1], y], '-', color=colors[i])
				# ax_user.plot([pos[0], x_r], [pos[1], y_r], '-', color='b')

			# for j, control in enumerate(robot.controls_):

			# 	ax_v.plot(j, control[0], '.', color=colors[i])
			# 	ax_w.plot(j, control[1], '.', color=colors[i])
			# 	ax_c.plot(j, control[2], '.', color=colors[i])

			e_r = []
			e_a = []
			e_c = []
			# e_a = []
			for k, error in enumerate(robot.errors_):

				e_r.append(error[0])
				e_a.append(error[1])
				e_c.append(error[2])
				# e_a.append(error[3])

			print(e_r)

			x = np.linspace(0, len(robot.errors_), len(robot.errors_))

			ax_e[0].plot(x, e_r, label='DDR {}'.format(i))
			ax_e[1].plot(x, e_a, label='DDR {}'.format(i))
			ax_e[2].plot(x, e_c, label='DDR {}'.format(i))
			# ax_ealp.plot(x, e_a)

	plt.legend()
	plt.show()

if __name__ == '__main__':

	# Initilize robots and user
	simulator.generate_user()

	# Add Robots
	# simulator.generate_and_add_robots()
	# simulator.initialize_cameras()
	# simulator.generate_and_add_robots_with_camera()
	simulator.generate_and_add_robots_with_pseudo_camera()
	# simulator.add_obstacles()

	while (1):

		simulator.step()

		print("Time: ", simulator.time_)
		ans = simulator.time_
		ans %= 3
		if  ans <= 0.05:
			ans = input('Would you like to continue with the simulation?: [Y/N] ')
			if ans == 'N' or ans == 'n':
				break

	auxiliar_plot(simulator, 3)

	# plot.visualize_state_1(simulator.robots_, simulator.user_, simulator.obstacles_)
	# plot.visualize_state_2(simulator.robots_, simulator.user_, simulator.obstacles_)
	# plot.visualize_state_3(simulator.robots_, simulator.user_, simulator.obstacles_)
	# plot.visualize_state_4(simulator.robots_, simulator.user_, simulator.obstacles_)
	input('Ahora se vera animado')
	plot.visualize_dynamic(simulator.robots_, simulator.user_, simulator.obstacles_)