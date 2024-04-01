import numpy as np
import matplotlib.pyplot as plt

rep = 0.1
att = 0.2

def attractive_force(position, target, strength=att):
    delta = target - position
    force_magnitude = strength * np.linalg.norm(delta)
    force_direction = delta / np.linalg.norm(delta)
    force = force_magnitude * force_direction
    return force

def repulsive_force(position, obstacle, strength=rep):
    delta = obstacle - position
    distance_sq = np.sum(delta**2)
    force_magnitude = strength / distance_sq
    force_direction = -delta / np.linalg.norm(delta)
    force = force_magnitude * force_direction
    return force

def total_force(position, target, obstacles, attractive_strength=att, repulsive_strength=rep):
    force = attractive_force(position, target, attractive_strength)
    for obstacle in obstacles:
        force += repulsive_force(position, obstacle, repulsive_strength)
    return force

def update_position(position, theta, target, obstacles, learning_rate=0.03, attractive_strength=att, repulsive_strength=rep, d=1.0, h=1.0):

    force = total_force(position, target, obstacles, attractive_strength, repulsive_strength)
    
    v_x = force[0]
    v_y = force[1]

    v = v_x*np.cos(theta) + v_y*np.sin(theta)
    omega = (1/d) * (-v_x*np.sin(theta) + v_y*np.cos(theta))

    # print(v*np.cos (theta))

    # position += learning_rate * force
    position[0] += learning_rate * v*np.cos(theta) - omega*h*np.sin(theta)
    position[1] += learning_rate * v*np.sin(theta) + omega*h*np.cos(theta)
    theta += learning_rate*omega

    return np.array(position), theta

# Coordenadas iniciales del objeto, el objetivo y los obstáculos
start_position = np.array([0.0, 0.0])
theta = np.pi/3
target_position = np.array([5.0, 5.0])
obstacle_positions = [np.array([3.0, 2.0]), np.array([4.5, 4.5]), np.array([1.0, 2.0])]
# obstacle_positions = [np.array([1.0, 2.0])]

# Número de iteraciones para actualizar la posición del objeto
num_iterations = 800

# Lista para almacenar las coordenadas del objeto en cada iteración
positions = [start_position]
orientations = [theta]

# Actualización iterativa de la posición del objeto
for i in range(num_iterations):
    start_position, theta = update_position(start_position, theta, target_position, obstacle_positions)
    positions.append(start_position)
    orientations.append(theta)
    # print(start_position)

positions = np.array(positions)

# print(positions)

# Visualización del movimiento del objeto con los obstáculos
plt.figure(figsize=(6, 6))
plt.plot(positions[:, 0], positions[:, 1], 'ko')
plt.plot(target_position[0], target_position[1], 'ro', label='Objetivo')
plt.plot(np.array(obstacle_positions)[:, 0], np.array(obstacle_positions)[:, 1], 'bo', label='Obstáculos')
plt.xlabel('Coordenada x')
plt.ylabel('Coordenada y')
plt.title('Movimiento del objeto con obstáculos hacia el objetivo')
plt.legend()
plt.grid(True)
plt.show()