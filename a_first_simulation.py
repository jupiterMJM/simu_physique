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
# balloon = Body(mass=0.5, name='balloon')
# bodies = [balloon]

# heavy_ball = Body(mass=10, name='heavy_ball', init_position=np.array([-3, 0, 10], dtype='float64'))
# light_ball = Body(mass=1, name='light_ball', init_position=np.array([3, 0, 10], dtype='float64'))
# bodies = [heavy_ball, light_ball]

cannon_0 = Body(mass=5, name='cannon_0', init_velocity=np.array([5, -2, 0], dtype='float64'))
cannon_pi3 = Body(mass=5, name='cannon_pi3', init_velocity=np.array([5, -1, np.tan(np.pi/3)*5], dtype='float64'))
cannon_pi4 = Body(mass=5, name='cannon_pi4', init_velocity=np.array([5, 1, np.tan(np.pi/4)*5], dtype='float64'))
cannon_pi2 = Body(mass=5, name='cannon_pi2', init_velocity=np.array([0, 0, 5], dtype='float64'))
cannon_3pi8 = Body(mass=5, name='cannon_3pi8', init_velocity=np.array([5, 2, np.tan(3*np.pi/8)*5], dtype='float64'))
bodies = [cannon_0, cannon_pi3, cannon_pi4, cannon_pi2, cannon_3pi8]
# bodies = [cannon_0, cannon_pi2]

# defining simulation parameters
dt = 0.01  # time step in seconds
simulation_time = 2.0  # total simulation time in seconds
num_steps = int(simulation_time / dt)

simu = Simulation(bodies=bodies, dt=dt, forces_to_consider=[gravitational_force])

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