"""
this is the main file for the GUI of the simulation 
TODO improve first connexion to pass some info btw the core simu and the GUI
"""

# plotteur.py
import zmq
import numpy as np
import threading    # to manage the communication in a separate thread (especially for the ping of connection)
import time
import pyqtgraph as pg

############################################################
## GLOBAL VARIABLE
############################################################
last_heartbeat = time.time()
is_simulation_running = threading.Event()
stop_all_threads = threading.Event()
############################################################

############################################################
## GLOBAL FUNCTION that will be put in a thread
############################################################
def monitor_heartbeat(timeout=3):
    global last_heartbeat
    while not stop_all_threads.is_set():
        if time.time() - last_heartbeat > timeout:
            print("⚠️  Heartbeat perdu : publisher injoignable ?")
        time.sleep(1)
    print("[CLOSE] Thread monitor_heartbeat terminated.")

def monitor_recv():
    """
    this function gathers in a thread all the function that receives the data from the core simulation
    """
    global is_simulation_running, last_heartbeat
    while not stop_all_threads.is_set():
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
                stop_all_threads.set()
                break
            else:
                print(f"Unknown control message: {msg}")
                continue
        
        elif topic == b"heartbeat/":
            print(f"Heartbeat received. {msg}")
            last_heartbeat = time.time()
            if not  is_simulation_running.is_set():
                if msg == b"not_running":
                    print("Simulation is not running.")
                elif msg == b"ping":
                    print("Simulation has started.")
                    is_simulation_running.set()

        elif topic == b"data/":
            print("Data received.")
            arr = np.frombuffer(msg, dtype="float64").reshape(6, -1)
        
        else:
            print(f"Unknown topic: {topic}")
            continue

    print("[CLOSE] Thread monitor_recv terminated.")

############################################################

############################################################
## ZMQ setup
############################################################
context = zmq.Context()
# socket to subscribe to heartbeat (monitoring the connection)
sub_heartbeat = context.socket(zmq.SUB)   # Subscriber
sub_heartbeat.connect("tcp://localhost:5556")
sub_heartbeat.setsockopt_string(zmq.SUBSCRIBE, "")  # S'abonner à tout

# socket to subscribe to data receival
sub_data = context.socket(zmq.SUB)
sub_data.connect("tcp://localhost:5557")
sub_data.setsockopt_string(zmq.SUBSCRIBE, "")

# poller to manage socket both from heartbeat and data
poller = zmq.Poller()
poller.register(sub_heartbeat, zmq.POLLIN)
poller.register(sub_data, zmq.POLLIN)

# socket to send data to the core simulation (to control it)
return_data_socket = context.socket(zmq.PUB)
return_data_socket.bind("tcp://*:5555")
print("Return data socket bound to port 5555.")

# starting the thread to run in the background (and no blocking the main thread)
thread_hb = threading.Thread(target=monitor_heartbeat, daemon=True)
thread_hb.start()
thread_recv = threading.Thread(target=monitor_recv, daemon=True)
thread_recv.start()
############################################################


time.sleep(1)
if not is_simulation_running.is_set():
    # TODO deal with the case : when GUI launched before simulation, and simulation launched in its cmd
    input("Press Enter to start the GUI and the simulation...")
    return_data_socket.send_multipart([b"control/", b"start"])


print("im here")
try:
    # Instead of blocking forever, check periodically
    while thread_hb.is_alive() or thread_recv.is_alive():
        thread_hb.join(timeout=0.5)
        thread_recv.join(timeout=0.5)
except KeyboardInterrupt:
    print("\nKeyboardInterrupt caught! Stopping threads...")
    stop_all_threads.set()
    # Ensure threads terminate
    thread_hb.join()
    thread_recv.join()

print("Exiting main program...")
poller.unregister(sub_heartbeat)
poller.unregister(sub_data)
sub_heartbeat.close()
sub_data.close()
return_data_socket.close()
context.term()
print("[Closed] GUI terminated")