"""
this is the main file for the GUI of the simulation 
TODO improve first connexion to pass some info btw the core simu and the GUI
"""

# plotteur.py
import zmq
import matplotlib.pyplot as plt
import numpy as np
import threading    # to manage the communication in a separate thread (especially for the ping of connection)
import time


# Global variable
last_heartbeat = time.time()


def monitor_heartbeat(timeout=3):
    global last_heartbeat
    while True:
        if time.time() - last_heartbeat > timeout:
            print("⚠️  Heartbeat perdu : publisher injoignable ?")
        time.sleep(1)


context = zmq.Context()
sub_heartbeat = context.socket(zmq.SUB)   # Subscriber
print("Connecting to server...")
sub_heartbeat.connect("tcp://localhost:5556")
sub_heartbeat.setsockopt_string(zmq.SUBSCRIBE, "")  # S'abonner à tout
print("Connected to server.")

sub_data = context.socket(zmq.SUB)
sub_data.connect("tcp://localhost:5557")
sub_data.setsockopt_string(zmq.SUBSCRIBE, "")

poller = zmq.Poller()
poller.register(sub_heartbeat, zmq.POLLIN)
poller.register(sub_data, zmq.POLLIN)

return_data_socket = context.socket(zmq.PUB)
return_data_socket.bind("tcp://*:5555")
print("Return data socket bound to port 5555.")



threading.Thread(target=monitor_heartbeat, daemon=True).start()

input("Press Enter to start the GUI and the simulation...")
return_data_socket.send_multipart([b"control/", b"start"])

while True:
    events = dict(poller.poll(1000))  # timeout 1s
    if sub_heartbeat in events:
        frames = sub_heartbeat.recv_multipart()
    elif sub_data in events:
        frames = sub_data.recv_multipart()
    else:
        print("No message received within the timeout period.")
        continue
    topic, msg = frames[0], frames[1]

    if len(frames) != 2:
        print(f"Message mal formé reçu, saut de ce message. (message de longueur {len(frames)}) :")
        for elem in frames:
            try:
                print(str(elem))
            except:
                print("Not printable element")
        break
    if topic == b"control/":
        if msg == b"shutdown":
            print("Shutdown command received. Exiting...")
            poller.unregister(sub_heartbeat)
            poller.unregister(sub_data)
            sub_heartbeat.close()
            sub_data.close()
            context.term()
            break
        else:
            print(f"Unknown control message: {msg}")
            continue
    
    elif topic == b"heartbeat/":
        print("Heartbeat received.")
        last_heartbeat = time.time()

    elif topic == b"data/":
        print("Data received.")
        arr = np.frombuffer(msg, dtype="float64").reshape(6, -1)
    
    else:
        print(f"Unknown topic: {topic}")
        continue


print("[Closed] GUI terminated")