"""
just a first simulation to check the well behavior of the code
"""

# importing libraries
import numpy as np
from core_calculation.body_definition import Body
from core_calculation.core_simulation import Simulation
from core_calculation.force_definition import *

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # enables 3D plots


# defining bodies


# defining simulation parameters
dt = 0.01  # time step in seconds
simulation_time = 2.0  # total simulation time in seconds
num_steps = int(simulation_time / dt)

# simu = Simulation(bodies=bodies, dt=dt, forces_to_consider=[gravitational_force])
# simu = Simulation(bodies=bodies, forces_to_consider=[(gravitational_force, {"g" : 100})], dt=dt)
simu = Simulation(json_file="scenarii_examples/cannon_balls.json")
bodies = simu.bodies

# running the simulation
historic_positions = {body.name: [] for body in bodies}
historic_speeds = {body.name: [] for body in bodies}
for step in range(num_steps):
    simu.step()
    for body in bodies:
        historic_positions[body.name].append(body.position.copy())
        historic_speeds[body.name].append(np.linalg.norm(body.velocity))

# plotting the results
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for body_name, positions in historic_positions.items():
    positions = np.array(positions)
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label=body_name)
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('3D Trajectories of Bodies')
ax.legend()

fig2 = plt.figure()
ax2 = fig2.add_subplot(111)
for body_name, speeds in historic_speeds.items():
    ax2.plot(np.arange(num_steps) * dt, speeds, label=body_name)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Speed (m/s)')
ax2.set_title('Speed of Bodies Over Time')
plt.show()