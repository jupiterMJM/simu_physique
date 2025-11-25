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
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt
from PyQt5 import QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import sys
from zmq_monitor import ZMQReceiver, context
from annexe_main_gui import *
from collections import deque
from scipy.spatial.transform import Rotation as R

receive_as_array = False

class PlotterGUI:
    def __init__(self, receiver: ZMQReceiver = None, dict_simu=None, plot_traj=False):
        """
        initalite the plotter GUI
        :param receiver: the ZMQReceiver object that will receive data from the simulation
        :param dict_simu: the dictionary containing the simulation parameters and bodies
        :param plot_traj: bool, whether to plot the trajectory of the bodies
        """
        # TODO modify the array buffer (not beautiful and not scalable for now)
        self.nb_of_bodies = len(dict_simu['objects'])
        self.dict_simu = dict_simu
        self.plot_traj = plot_traj
        bodies = dict_simu['objects']
        init_pos = np.array([elt["init_position"] for key, elt in bodies.items()])
        all_basis = {key : elt["initial_base"] for key, elt in bodies.items()}

        # note : eventhough we define historic variables, those might not be used if specified otherwise
        self.N = 1000
        self.kinetic_history = {name: deque(maxlen=self.N*3) for name in self.dict_simu['objects'].keys()}
        self.potential_history = {name: deque(maxlen=self.N*3) for name in self.dict_simu['objects'].keys()}
        self.time_current_history = deque(maxlen=self.N*3)
        self.all_history = {elt["name"]: np.zeros((self.N, 3)) for key, elt in bodies.items()} # to store trajectory history
        self.num_point = 0
        self.buffer_once_completed = False  # to know if the buffer is filled at least once
        

        ###########################################################################################
        ## CREATION OF THE FIRST WINDOW : position of the bodies in 3D
        ###########################################################################################

        ## I/ Create the application
        self.app = QApplication(sys.argv)
        # Create 3D view
        self.view = gl.GLViewWidget()
        self.view.show()
        self.view.setWindowTitle('3D Live Plot (Threaded ZMQ)')
        # Grid
        g = gl.GLGridItem()
        g.scale(2, 2, 1)
        self.view.addItem(g)

        if self.dict_simu["plotting"] == []:
            self.dict_simu["plotting"] = ["3D"]
        if "3D" in self.dict_simu["plotting"]:
            ## II/ Plot the initial positions of the bodies
            self.scatter = gl.GLScatterPlotItem(pos=init_pos, color=(1, 0, 0, 1), size=5)
            self.view.addItem(self.scatter)
            # Adjust the camera to ensure all points are visible
            max_extent = np.max(np.linalg.norm(init_pos, axis=1))  # Calculate the maximum distance from origin
            # maybe not the best way to do it but it works for now
            if max_extent == 0 or max_extent < self.view.cameraPosition().length():
                pass
            else:
                self.view.setCameraPosition(distance=max_extent * 2, elevation=20, azimuth=30)


        ## III/ Do the plot of the basis of each body (if needed)
        self.basis_lines = {}
        for i, elt in enumerate(bodies.items()):
            basis_line_body = list()
            cle, dico = elt
            name = dico["name"]
            if dico["representation"] == "3D_solid_body":
                quaternion_base = np.array(all_basis[cle])
                base = R.from_quat(quaternion_base, scalar_first=True).as_matrix()
                # print(base)
                origin = init_pos[i, :]

                # Create lines for each axis
                colors = [(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)]
                for j in range(3):
                    axis = base[:, j]
                    line_data = np.array([origin, origin + axis])
                    axis_line = gl.GLLinePlotItem(pos=line_data, color=colors[j], width=2, antialias=True)
                    self.view.addItem(axis_line)
                    basis_line_body.append(axis_line)
                self.basis_lines[name] = basis_line_body
                



            ## IV/ Plot of trajectories (if needed)
            if self.plot_traj:
                self.all_trajectory = {}
                for i, elt in enumerate(bodies.items()):
                    cle, dico = elt
                    name = dico["name"]
                    self.all_history[name][0, :] = init_pos[i, :]

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

            self.add_cartesian_base()
        ###########################################################################################


        ###########################################################################################
        ## CREATION OF THE SECOND WINDOW : kinetic energy of the bodies
        ###########################################################################################
        if "energy" in self.dict_simu["plotting"]:

            # Create the second window for kinetic energy
            self.energy_window = pg.GraphicsLayoutWidget(show=True)
            self.energy_window.setWindowTitle("Energies of Bodies")
            self.energy_plot = self.energy_window.addPlot(title="Energies")
            self.energy_plot.addLegend()
            self.kinetic_energy_curves = {
                name: self.energy_plot.plot(pen=pg.mkPen(color, style=QtCore.Qt.DashLine), name=f"K_{name}")
                for name, color in zip(self.dict_simu['objects'].keys(), ['r', 'g', 'b', 'y', 'm', 'c'])
            }
            self.potential_energy_curves = {
                name: self.energy_plot.plot(pen=pg.mkPen(color, style=QtCore.Qt.DotLine), name=f"P_{name}")
                for name, color in zip(self.dict_simu['objects'].keys(), ['r', 'g', 'b', 'y', 'm', 'c'])
            }
            self.mechanical_energy_curves = {
                name: self.energy_plot.plot(pen=pg.mkPen(color), name=f"E_{name}")
                for name, color in zip(self.dict_simu['objects'].keys(), ['r', 'g', 'b', 'y', 'm', 'c'])
            }
            self.total_mechanical_energy_syst_curve = self.energy_plot.plot(pen=pg.mkPen('w', width=2), name="Total mech. energy")
            self.energy_plot.setLabel('left', 'Energy', units='J')
            self.energy_plot.setLabel('bottom', 'Time', units='s')
            self.energy_plot.showGrid(x=True, y=True)

        ###########################################################################################

        

        # SOME STUFF left to do
        # Background ZMQ thread
        self.receiver = receiver
        # self.receiver.data_received.connect(self.update_plot)
        # self.receiver.data_received.connect(self.update_energy_plot)
        self.receiver.data_received.connect(self.update_all_plots)
        self.receiver.start()

        # Timer just keeps Qt responsive (not strictly required if using signals)
        self.camera_sync_timer = QTimer()
        self.camera_sync_timer.timeout.connect(self.sync_inset_camera)
        self.camera_sync_timer.start(50)  # toutes les 50 ms

    def update_all_plots(self, info_from_zmq):
        """
        this function will allow us to update all the plots in one call
        the main usecase is to save history of positions and speeds more easily
        """
        
        if receive_as_array:
            points_position = info_from_zmq[0:3, 1:].T
            points_velocity = info_from_zmq[3:6, 1:].T
            potential_energy_bodies = info_from_zmq[6, 1:]
            basis_matrices_quaternion = info_from_zmq[7:7+4, 1:]
            self.time_current_history.append(info_from_zmq[1, 0])

        else: # receive info_from_zmq as a dict
            points_position = np.array([info_from_zmq["bodies"][name]["position"] for name in self.dict_simu['objects'].keys()])
            points_velocity = np.array([info_from_zmq["bodies"][name]["velocity"] for name in self.dict_simu['objects'].keys()])
            potential_energy_bodies = np.array([info_from_zmq["bodies"][name]["potential_energy"] for name in self.dict_simu['objects'].keys()])
            basis_matrices_quaternion = np.array([info_from_zmq["bodies"][name]["quaternion"] for name in self.dict_simu['objects'].keys()]).T
            self.time_current_history.append(info_from_zmq["current_time"])

        for i, elt in enumerate(self.dict_simu['objects'].items()):
            cle, dico = elt
            self.all_history[dico["name"]][self.num_point, :] = points_position[i, :]
            self.kinetic_history[dico["name"]].append(0.5 * dico["mass"] * np.sum(points_velocity[i, :]**2))
            self.potential_history[dico["name"]].append(potential_energy_bodies[i])
        
        self.num_point = self.num_point + 1
        if self.num_point >= self.N:
            self.buffer_once_completed = True
        self.num_point = self.num_point if self.num_point < self.N else 0
        # and finally update the plots
        if "3D" in self.dict_simu["plotting"]:
            self.update_plot(points_position, basis_matrices_quaternion)
        if "energy" in self.dict_simu["plotting"]:
            self.update_energy_plot()

    def update_plot(self, body_position, basis_matrices_quaternion):
        """Called in GUI thread when ZMQ thread emits new data."""

        # FIRST : extraction of the information that are relevant to us
        # points = matrix_from_zmq[0:3, 1:].T
        
        # print(basis_matrices_quaternion)
        # print(basis_matrices)

        # print(matrix_from_zmq)
        self.scatter.setData(pos=body_position)
        # update basis lines if needed
        for i, elt in enumerate(self.dict_simu['objects'].items()):
            cle, dico = elt
            name = dico["name"]
            if dico["representation"] == "3D_solid_body":
                origin = body_position[i, :]
                base_quaternion = basis_matrices_quaternion[:, i]
                base = R.from_quat(base_quaternion, scalar_first=True).as_matrix()
                # Update lines for each axis
                for j in range(3):
                    axis = base[:, j]
                    line_data = np.array([origin, origin + axis])
                    self.basis_lines[name][j].setData(pos=line_data)
        if self.plot_traj:
            for i, elt in enumerate(self.dict_simu['objects'].items()):
                # print("elt:", elt)
                cle, dico = elt
                name = dico["name"]
                histo_to_plot = self.all_history[name]
                if self.buffer_once_completed:
                    # histo_to_plot[self.num_point, :] = points[i, :]
                    # # TODO essayer de modif le np.vstack pour faire un truc plus propre (eviter de creer un nouveau tableau à chaque fois)
                    self.all_trajectory[name].setData(pos=np.vstack([histo_to_plot[self.num_point:], histo_to_plot[:self.num_point]]))
                else:
                    # histo_to_plot[self.num_point, :] = points[i, :]
                    self.all_trajectory[name].setData(pos=histo_to_plot[:self.num_point])
            # self.num_point = self.num_point + 1
            # if self.num_point >= self.N:
            #     self.buffer_once_completed = True
            # self.num_point = self.num_point if self.num_point < self.N else 0

    def update_energy_plot(self):
        """
        Update the kinetic energy plot with the current velocities of the bodies.
        :param velocities: A dictionary with body names as keys and velocity vectors as values.
        """
        time_axis = list(self.time_current_history)
        for name, curve in self.kinetic_energy_curves.items():
            kinetic_energy = list(self.kinetic_history[name])
            curve.setData(time_axis, kinetic_energy)
        for name, curve in self.potential_energy_curves.items():
            potential_energy = list(self.potential_history[name])
            curve.setData(time_axis, potential_energy)
        
        list_for_total_mech_energy = []
        for name, curve in self.mechanical_energy_curves.items():
            mechanical_energy = [k + p for k, p in zip(self.kinetic_history[name], self.potential_history[name])]
            list_for_total_mech_energy.append(mechanical_energy)
            curve.setData(time_axis, mechanical_energy)
        if len(list_for_total_mech_energy) > 0:
            total_mechanical_energy = [sum(energies) for energies in zip(*list_for_total_mech_energy)]
        self.total_mechanical_energy_syst_curve.setData(time_axis, total_mechanical_energy)
        

    def add_cartesian_base(self):
        """
        Add a small 3D Cartesian base in the bottom-left corner of the main 3D window.
        """
        # Create a new GLViewWidget for the inset
        self.inset_view = gl.GLViewWidget()
        self.inset_view.setFixedSize(150, 150)  # Set the size of the inset
        self.inset_view.opts['distance'] = 5  # Set the camera distance for the inset
        self.inset_view.opts["elevation"] = self.view.opts.get("elevation", 0)
        self.inset_view.opts["azimuth"] = self.view.opts.get("azimuth", 0)
        # print("hellllllllllllo", self.view.cameraParams())
        self.inset_view.setWindowTitle('Cartesian Base')

        # Create the Cartesian base axes
        origin = np.array([0, 0, 0])
        x_axis = np.array([[0, 0, 0], [1, 0, 0]])  # X-axis
        y_axis = np.array([[0, 0, 0], [0, 1, 0]])  # Y-axis
        z_axis = np.array([[0, 0, 0], [0, 0, 1]])  # Z-axis

        # Add the axes to the inset view
        x_line = gl.GLLinePlotItem(pos=x_axis, color=(1, 0, 0, 1), width=2, antialias=True)  # Red for X
        y_line = gl.GLLinePlotItem(pos=y_axis, color=(0, 1, 0, 1), width=2, antialias=True)  # Green for Y
        z_line = gl.GLLinePlotItem(pos=z_axis, color=(0, 0, 1, 1), width=2, antialias=True)  # Blue for Z

        self.inset_view.addItem(x_line)
        self.inset_view.addItem(y_line)
        self.inset_view.addItem(z_line)

        # Embed the inset into the bottom-left corner of the main window
        layout = QVBoxLayout(self.view)
        layout.setContentsMargins(10, 10, 0, 0)  # Adjust margins to position the inset
        layout.addWidget(self.inset_view, alignment=Qt.AlignLeft | Qt.AlignBottom)
            

    def sync_inset_camera(self):
        # print("im here")
        params = self.view.cameraParams()
        self.inset_view.setCameraParams(elevation=params['elevation'], azimuth=params['azimuth'])
        # print(params["azimuth"], params["elevation"])
        # print(self.view.opts.get("azimuth", 0), self.view.opts.get("elevation", 0), self.view.opts.get("roll", 0))
        # self.inset_view.opts['elevation'] = self.view.opts.get("elevation", 0)
        # self.inset_view.opts['azimuth'] = self.view.opts.get("azimuth", 0)
        # self.inset_view.opts['roll'] = params.get('roll', 0)
        # self.inset_view.opts['distance'] = params['distance']
        # self.inset_view.opts['center'] = params['center']

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
    receiver = ZMQReceiver(receive_as_array=receive_as_array)
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