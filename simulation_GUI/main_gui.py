"""
this is the main file for the GUI of the simulation 
TODO improve first connexion to pass some info btw the core simu and the GUI
"""

# plotteur.py
import zmq
import numpy as np
import time
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import QApplication
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import sys

############################################################
## GLOBAL VARIABLE
############################################################
last_heartbeat = time.time()
check_for_heartbeat = False
context = zmq.Context()
############################################################

############################################################
## CLASS ZMQ RECEIVER : will run in a separate thread to not block the GUI and will monitor zmq connection
############################################################
class ZMQReceiver(QThread):
    """Background thread that listens to ZMQ and emits new data."""
    data_received = pyqtSignal(object)  # signal will send numpy array

    def __init__(self, timeout=5):
        super().__init__()
        self.zmq_running = False
        self.simu_running = False
        self.timeout = timeout

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
        global last_heartbeat, check_for_heartbeat
        while self.zmq_running:
            # checking the heartbeat
            if time.time() - last_heartbeat > self.timeout and check_for_heartbeat:
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
                    check_for_heartbeat = True
                    last_heartbeat = time.time()
                    

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
############################################################


class PlotterGUI:
    def __init__(self):
        self.app = QApplication(sys.argv)

        # Create 3D view
        self.view = gl.GLViewWidget()
        self.view.show()
        self.view.setWindowTitle('3D Live Plot (Threaded ZMQ)')
        self.view.setCameraPosition(distance=20)

        # Grid
        g = gl.GLGridItem()
        g.scale(2, 2, 1)
        self.view.addItem(g)

        # Scatter plot
        self.scatter = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), color=(1, 0, 0, 1), size=5)
        self.view.addItem(self.scatter)

        # Background ZMQ thread
        self.receiver = ZMQReceiver(timeout=5)
        self.receiver.data_received.connect(self.update_plot)
        self.receiver.start()

        # Timer just keeps Qt responsive (not strictly required if using signals)
        self.timer = QTimer()
        self.timer.start(100)

    def update_plot(self, matrix_from_zmq):
        """Called in GUI thread when ZMQ thread emits new data."""
        points = matrix_from_zmq[0:3, 1:].T
        # print(matrix_from_zmq)
        self.scatter.setData(pos=points)

    def run(self):
        sys.exit(self.app.exec_())

    def stop(self):
        self.receiver.stop()



############################################################
## TODO modify the return data socket bcs not used anymore for now
# socket to send data to the core simulation (to control it)
return_data_socket = context.socket(zmq.PUB)
return_data_socket.bind("tcp://*:5555")
print("Return data socket bound to port 5555.")
############################################################


# time.sleep(1)
# if not is_simulation_running.is_set():
#     # TODO deal with the case : when GUI launched before simulation, and simulation launched in its cmd
#     input("Press Enter to start the GUI and the simulation...")
#     return_data_socket.send_multipart([b"control/", b"start"])


time.sleep(1)
if __name__ == "__main__":
    plotter = PlotterGUI()
    try:
        plotter.run()
    finally:
        plotter.stop()
    return_data_socket.close()
    context.term()
    print("[Closed] GUI terminated")