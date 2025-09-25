# plotteur.py
import zmq
import matplotlib.pyplot as plt
import numpy as np

context = zmq.Context()
socket = context.socket(zmq.SUB)   # Subscriber
print("Connecting to server...")
socket.connect("tcp://localhost:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, "")  # S'abonner à tout
print("Connected to server.")

while True:
    msg = socket.recv(copy=False)
    arr = np.frombuffer(memoryview(msg), dtype=np.float64)
    print(arr)