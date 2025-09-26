"""
this is the main file for the GUI of the simulation 
TODO find out why i cannot compare the topic directly with a bytes object
"""

# plotteur.py
import zmq
import matplotlib.pyplot as plt
import numpy as np
import threading    # to manage the communication in a separate thread (especially for the ping of connection)
import time

# def monitor_heartbeat(timeout=3):
#     global last_heartbeat
#     while True:
#         if time.time() - last_heartbeat > timeout:
#             print("⚠️  Heartbeat perdu : publisher injoignable ?")
#         time.sleep(1)


context = zmq.Context()
socket = context.socket(zmq.SUB)   # Subscriber
print("Connecting to server...")
socket.connect("tcp://localhost:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, "")  # S'abonner à tout
print("Connected to server.")

while True:
    topic, msg = socket.recv_multipart(copy=False)

    if str(topic) == "control/":
        if str(msg) == "shutdown":
            print("Shutdown command received. Exiting...")
            socket.close()
            context.term()
            break
            
    arr = np.frombuffer(msg.buffer, dtype="float64").reshape(6, -1)
    # arr = np.frombuffer(memoryview(msg), dtype=np.float64)
    # print(arr)

print("[Closed] GUI terminated")