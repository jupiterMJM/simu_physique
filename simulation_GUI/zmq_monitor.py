"""
this file contains the class that receives and treats the data from ZMQ
this class will be running in another thread (Qthread) to work in parallel of the pyqtGraph GUI
:author: Maxence Barré
:date: 2025
"""

# importing the modules
import zmq
import numpy as np
import time
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import QApplication
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import sys


context = zmq.Context()



class ZMQReceiver(QThread):
    """Background thread that listens to ZMQ and emits new data."""
    data_received = pyqtSignal(object)  # signal will send numpy array

    def __init__(self, timeout=5):
        super().__init__()
        self.zmq_running = False
        self.simu_running = False
        self.timeout = timeout

        self.last_heartbeat = time.time()
        self.check_for_heartbeat = False

        self.setup_zmq_recv()
        self.zmq_running = True
        

    def setup_zmq_recv(self):
        # socket to subscribe to heartbeat (monitoring the connection)
        self.sub_heartbeat = context.socket(zmq.SUB)   # Subscriber
        self.sub_heartbeat.connect("tcp://localhost:5556")
        self.sub_heartbeat.setsockopt_string(zmq.SUBSCRIBE, "")  # S'abonner à tout

        # socket to subscribe to data receival
        self.sub_data = context.socket(zmq.SUB)
        self.sub_data.connect("tcp://localhost:5557")
        self.sub_data.setsockopt_string(zmq.SUBSCRIBE, "")

        # poller to manage socket both from heartbeat and data
        self.poller = zmq.Poller()
        self.poller.register(self.sub_heartbeat, zmq.POLLIN)
        self.poller.register(self.sub_data, zmq.POLLIN)
        
    
    def run(self):
        while self.zmq_running:
            # checking the heartbeat
            if time.time() - self.last_heartbeat > self.timeout and self.check_for_heartbeat:
                print("No heartbeat received within timeout period. Assuming simulation is down.")
                self.simu_running = False
                self.zmq_running = False
                break
            try:
                events = dict(self.poller.poll(1000))  # timeout 1s
                if self.sub_heartbeat in events:
                    frames = self.sub_heartbeat.recv_multipart()
                elif self.sub_data in events:
                    frames = self.sub_data.recv_multipart()
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
                        self.simu_running = False
                        self.zmq_running = False
                        break
                    else:
                        print(f"Unknown control message: {msg}")
                        continue
                
                elif topic == b"heartbeat/":
                    print(f"Heartbeat received. {msg}")
                    self.check_for_heartbeat = True
                    self.last_heartbeat = time.time()
                    

                elif topic == b"data/":
                    # print("Data received.")
                    arr = np.frombuffer(msg, dtype="float64").reshape(6, -1)
                    # print("arr", arr)
                    self.data_received.emit(arr)  # send to GUI thread
                
                else:
                    print(f"Unknown topic: {topic}")
                
                
                
            except Exception as e:
                print("ZMQ error:", e)

    def stop(self):
        self.simu_running = False
        self.zmq_running = False
        self.poller.unregister(self.sub_heartbeat)
        self.poller.unregister(self.sub_data)
        self.sub_heartbeat.close()
        self.sub_data.close()
        self.quit()