"""
this file will run and monitor the simulation
it will provide an easy way to send the result to a plotting function
:auteur: Maxence BARRE
:date: 2025
"""

# importing libraries
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from core_calculation.body_definition import Body
from core_calculation.core_simulation import Simulation
from core_calculation.force_definition import *
import zmq
import time
from tqdm import tqdm

def generate_message(simu:Simulation):
    """
    this function will generate a message to send through ZMQ
    :param simu: the simulation object (got all the parameters and bodies)
    :return: an array containing the message to send
    :note: why an array? bcs ZMQ is optimized for sending arrays
    array will be 6x(N+1) where N is the number of bodies
    first column will be general parameters that can be useful (dt, current time, ...)
    then each column will be a body with its parameters (position and velocity)
    TODO on the first communication think about sending all the information about the bodies (mass, name, ...) but only once
    """
    msg_array = np.zeros((6, len(simu.bodies)), dtype='float64')
    # general parameters
    msg_array[0, 0] = simu.dt
    msg_array[1, 0] = simu.current_time
    msg_array[2, 0] = len(simu.bodies)

    for i, body in enumerate(simu.bodies):
        # print(body.position.flatten())
        msg_array[0:3, i] = body.position.flatten()
        msg_array[3:6, i] = body.velocity.flatten()

    return msg_array


cannon_0 = Body(mass=5, name='cannon_0', init_velocity=np.array([5, -2, 0], dtype='float64'))
bodies = [cannon_0]

# defining simulation parameters
dt = 0.01  # time step in seconds
simulation_time = 1000.0  # total simulation time in seconds
num_steps = int(simulation_time / dt)
simu = Simulation(bodies=bodies, dt=dt, forces_to_consider=[gravitational_force])
print(generate_message(simu))

# setting up ZMQ for communication
context = zmq.Context()
socket = context.socket(zmq.PUB)  # Publisher
socket.bind("tcp://*:5555")
time.sleep(1)  # Wait a moment to ensure the socket is ready

input("Press Enter to start the simulation...")
# running the simulation
for step in tqdm(range(num_steps)):
    simu.step()
    for body in bodies:
        arr = generate_message(simu)
        socket.send(arr, copy=False)  # Send the array without copying