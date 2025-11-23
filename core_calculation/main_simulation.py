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
from zmq import Socket
import time
from tqdm import tqdm
import threading    # to manage the communication in a separate thread (especially for the ping of connection)
import pickle

#############################################################################
## USER PARAMETERS
## note: change here to modify behaviour of the calculation
#############################################################################
json_file = r"scenarii_examples\9_toupie.json"
# dt and time_simulation are defined in the json file

# speed_simulation = 1000000 # expected ratio of real time vs simulation time (e.g. 2 means the simulation will run twice faster than real time)
# eg: 1/2 will mean that the simulation will run at half the speed of real time
# "max" means the simulation will run as fast as possible (no waiting time)

verbose=False
send_in_array = False  # whether to send the data in an array (optimized for ZMQ) or in a dictionary (easier to read)

freq_send_data_zmq = 20  # in Hz
#############################################################################


#############################################################################
## GLOBAL VARIABLE used throughout the code NOT FOR USER
## note: you do not need to modify what is here
#############################################################################
do_heartbeat = True
hear_for_client = True
simulation_is_running = False

# Flag to indicate whether the simulation should start
start_simulation = threading.Event()

# setting up ZMQ for communication; one socket for sending data, one for receiving commands
context = zmq.Context()
socket = context.socket(zmq.PUB)  # Publisher
socket.bind("tcp://*:5557")
sub_socket = context.socket(zmq.SUB)
sub_socket.connect("tcp://localhost:5555")
sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages

time_last_sent = time.time()
#############################################################################


#############################################################################
## GLOBAL FUNCTION
## note : function used to monitor simulation's behavior but not the simulation itself
#############################################################################
def generate_message(simu:Simulation, send_in_array=True):
    """
    this function will generate a message to send through ZMQ
    :param simu: the simulation object (got all the parameters and bodies)
    :return: an array containing the message to send
    :note: why an array? bcs ZMQ is optimized for sending arrays
    array will be 6x(N+1) where N is the number of bodies
    first column will be general parameters that can be useful (dt, current time, ...)
    then each column will be a body with its parameters (positionx3 and velocityx3 and internal potential energyx1)
    TODO on the first communication think about sending all the information about the bodies (mass, name, ...) but only once
    """
    # 3 for position, 3 for velocity, 1 for potential energy, 4 for quaternion elements
    if send_in_array :
        msg_array = np.zeros((3+3+1+4, len(simu.bodies)+1), dtype='float64')
        # general parameters
        msg_array[0, 0] = simu.dt
        msg_array[1, 0] = simu.current_time
        msg_array[2, 0] = len(simu.bodies)

        potential_all_bodies = simu.compute_potentiel_energy()
        for i, body in enumerate(simu.bodies):
            # print(body.position.flatten())
            msg_array[0:3, i+1] = body.position.flatten()
            msg_array[3:6, i+1] = body.velocity.flatten()
            msg_array[6, i+1] = potential_all_bodies[body.name]
        if body.representation == "3D_solid_body":
            msg_array[7:7+4, i+1] = body.local_basis.quaternion.flatten()
        else:
            msg_array[7:7+4, i+1] = np.array([np.nan, np.nan, np.nan])
        # print("msg_array:", msg_array)
        return msg_array
    else:
        msg_dict = {
            "dt": simu.dt,
            "current_time": simu.current_time,
            "num_bodies": len(simu.bodies),
            "bodies": {}
        }
        potential_all_bodies = simu.compute_potentiel_energy()
        for body in simu.bodies:
            msg_dict["bodies"][body.name] = {
                "position": body.position.flatten().tolist(),
                "velocity": body.velocity.flatten().tolist(),
                "potential_energy": float(potential_all_bodies[body.name])
            }
        return msg_dict

def heartbeat():
    socket_heartbeat: Socket = context.socket(zmq.PUB)
    socket_heartbeat.bind("tcp://*:5556")
    time.sleep(1)
    while do_heartbeat:
        if not simulation_is_running:
            socket_heartbeat.send_multipart([b"heartbeat/", b"not_running"])
        else:
            socket_heartbeat.send_multipart([b"heartbeat/", b"ping"])
        time.sleep(1)
    socket_heartbeat.setsockopt(zmq.LINGER, 1000)  # waiting for everything to be sent
    socket_heartbeat.close() # Close the socket
    print("[INFO] Heartbeat thread terminated.")

def wait_for_client_message():
    """Wait for a message from the ZMQ client to start the simulation."""
    

    print("Waiting for a message from the client...")
    while hear_for_client:
        
        try:
            topic, message = sub_socket.recv_multipart(flags=zmq.NOBLOCK)  # Non-blocking receive
            # print(topic, message)
            if message == b"start":
                print("Received 'START' message from client. Starting simulation...")
                start_simulation.set()  # Signal to start the simulation

            elif topic == b"control/" and message == b"giveme_info":
                json_dict = simu.all_info_json()
                if send_in_array:
                    json_dict["msg_format"] = "array"
                else:
                    json_dict["msg_format"] = "json"
                socket.send_string("info/", flags=zmq.SNDMORE)
                socket.send_json(json_dict)
        except zmq.Again:
            time.sleep(0.1)  # Avoid busy-waiting
        except KeyboardInterrupt:
            start_simulation.set()
            print("[INFO] Simulation interrupted by user while waiting for client message.")
            break

def wait_for_user_input():
    """Wait for the user to press Enter to start the simulation."""
    print("Press Enter to start the simulation...")
    
    try:
        input()  # Wait for user input
        if not start_simulation.is_set():
            print("User pressed Enter. Starting simulation...")
            start_simulation.set()  # Signal to start the simulation
    except:
        start_simulation.set()
        print("[INFO] Simulation interrupted by user while waiting for input.")
#############################################################################  


#############################################################################
## MAIN CODE
## note: this is where the simulation is defined and run
#############################################################################

# # DEFINITION OF THE SIMULATION
# # defining simulation parameters
simu = Simulation(json_file=json_file)
# simulation_time = simu.simulation_time
# dt = simu.dt
# num_steps = int(simulation_time / dt)
# simu = Simulation(bodies=bodies, dt=dt, forces_to_consider=[gravitational_force])



# computing the time to wait at each step to get the expected ratio of real time vs simulation time
# if speed_simulation == "max":
#     time_to_wait_for_ratio = 0
# else:
#     aim_time_per_step = simu.dt / speed_simulation
#     actual_time_per_step = simu.benchmark_step()
#     time_to_wait_for_ratio = max(aim_time_per_step - actual_time_per_step, 0)

# START HEARTBEAT THREAD
thread_hb = threading.Thread(target=heartbeat, daemon=True).start()

# WAITING FOR THE START COMMAND
# condition before beginning the simulation : either the user start in the main simulation cmd or in the zmq-client cmd
# Wait until the simulation is triggered
try:
    client_thread = threading.Thread(target=wait_for_client_message, daemon=True)
    input_thread = threading.Thread(target=wait_for_user_input, daemon=True)
    client_thread.start()
    input_thread.start()
    start_simulation.wait()
except KeyboardInterrupt:
    print("[INFO] Simulation interrupted by user before starting.")
    do_heartbeat = False
    start_simulation.set()  # Ensure any waiting threads can proceed
    time.sleep(2)
    socket.close()
    sub_socket.close()
    time.sleep(1)
    context.term()
    time.sleep(2)
    sys.exit(0)

# RUN THE SIMULATION
try:
    simulation_is_running = True
    for _ in simu.run():
        # input()
        if time.time() - time_last_sent >= 1.0 / freq_send_data_zmq:
            time_last_sent = time.time()
             # Generate and send the message
            arr = generate_message(simu, send_in_array=send_in_array)
            if not send_in_array:   # arr is a json
                socket.send_string("data/", flags=zmq.SNDMORE)
                socket.send_json(arr)
            else:
                socket.send_multipart([b"data/", memoryview(arr)])  # Send the array without copying
        # if time_to_wait_for_ratio > 0:
        #     time.sleep(time_to_wait_for_ratio)
            
            
except KeyboardInterrupt:
    print("[INFO] Simulation interrupted by user on core calculation.")
    print("[INFO] Closing the ZMQ connection.")
    
finally:
    socket.send_multipart([b"control/", b"shutdown"])
    socket.setsockopt(zmq.LINGER, 5000)  # waiting for everything to be sent
    socket.close() # Close the socket

    sub_socket.close()
    do_heartbeat = False
    hear_for_client = False
    time.sleep(2)
    # thread should be closed automatically by now

    context.term()  # Terminate the ZMQ context
    print("[INFO] ZMQ connection closed.")