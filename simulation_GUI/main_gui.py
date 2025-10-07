"""
this is the main file for the GUI of the simulation 
TODO check how the import of context behaves (effet de bord? full acces à la variable prke on passe l'adresse mémoire?)
TODO improve first connection to share knowledge and param on the simulation
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
from zmq_monitor import ZMQReceiver, context


class PlotterGUI:
    def __init__(self, receiver: ZMQReceiver = None, dict_simu=None, plot_traj=False):
        # TODO modify the array buffer (not beautiful and not scalable for now)
        self.nb_of_bodies = len(dict_simu['objects'])
        self.dict_simu = dict_simu
        self.plot_traj = plot_traj
        if self.plot_traj:
            self.N = 50
            bodies = dict_simu['objects']
            print(bodies)
            init_pos = np.array([elt["init_position"] for key, elt in bodies.items()])
            self.all_history = {elt["name"]: np.zeros((self.N, 3)) for key, elt in bodies.items()} # to store trajectory history
            for i, elt in enumerate(bodies.items()):
                cle, dico = elt
                name = dico["name"]
                self.all_history[name][0, :] = init_pos[i, :]
        self.num_point = 0
        
        self.buffer_once_completed = False
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

        # # Scatter plot
        self.scatter = gl.GLScatterPlotItem(pos=init_pos, color=(1, 0, 0, 1), size=5)
        self.view.addItem(self.scatter)

        self.all_trajectory = {}
        for i, elt in enumerate(bodies.items()):
            body_trajectory = gl.GLLinePlotItem(
                pos=np.zeros((2, 3)),       # need at least two points to draw the first line
                color=(1, 1, 0, 1),
                width=2,
                antialias=True,
                mode="line_strip"
            )
            cle, dico = elt
            self.all_trajectory[dico["name"]] = body_trajectory
            self.view.addItem(body_trajectory)

        # Background ZMQ thread
        self.receiver = receiver
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
        if self.plot_traj:
            for i, elt in enumerate(self.dict_simu['objects'].items()):
                # print("elt:", elt)
                cle, dico = elt
                name = dico["name"]
                histo_to_plot = self.all_history[name]
                if self.buffer_once_completed:
                    histo_to_plot[self.num_point, :] = points[i, :]
                    # # TODO essayer de modif le np.vstack pour faire un truc plus propre (eviter de creer un nouveau tableau à chaque fois)
                    self.all_trajectory[name].setData(pos=np.vstack([histo_to_plot[self.num_point+1:], histo_to_plot[:self.num_point+1]]))
                else:
                    histo_to_plot[self.num_point, :] = points[i, :]
                    self.all_trajectory[name].setData(pos=histo_to_plot[:self.num_point+1])
            self.num_point = self.num_point + 1
            if self.num_point >= self.N:
                self.buffer_once_completed = True
            self.num_point = self.num_point if self.num_point < self.N else 0


            

    def run(self):
        sys.exit(self.app.exec_())

    def stop(self):
        self.receiver.stop()



############################################################
## TODO modify the return data socket bcs not used anymore for now
# socket to send data to the core simulation (to control it)

print("Return data socket bound to port 5555.")
############################################################


# time.sleep(1)
# if not is_simulation_running.is_set():
#     # TODO deal with the case : when GUI launched before simulation, and simulation launched in its cmd
#     input("Press Enter to start the GUI and the simulation...")
#     return_data_socket.send_multipart([b"control/", b"start"])


time.sleep(1)
if __name__ == "__main__":
    receiver = ZMQReceiver()
    print("[Starting] function receiver obtain_info_before_run()")
    json_dict = receiver.obtain_info_before_run()
    print(json_dict)
    print(type(json_dict))
    plotter = PlotterGUI(receiver=receiver, dict_simu=json_dict, plot_traj=True)
    try:
        
        plotter.run()
    finally:
        plotter.stop()
    return_data_socket.close()
    context.term()
    print("[Closed] GUI terminated")