"""
:author: Maxence Barré
:date: 2025
:description: this file will integrate Newton's law to calculate the position and velocity of
a body at each time step.
:note: the integration method used here is the Velocity Verlet method.
:note: we place ourseslves in a 3D space with x, y and z axis.
TODO pour la gestion des param dans le __init__ transformer en:
    def f(x=None, y=None, **kwargs):
...     print(x, y, kwargs)
>>> c = {"y":2, "x":1, "z":3}
>>> f(**c)
1 2 {'z': 3}
"""

# importing libraries
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from core_calculation.body_definition import Body
from core_calculation.force_definition import *
import json

class Simulation:
    """
    this class will provide a way to integrate and simulate the motion of a/several body/ies
    under the influence of forces.
    """

    def __init__(self, bodies:list[Body]=None, dt:np.float64=None, forces_to_consider:list | list[tuple]=None, json_file:str=None):
        """
        initiate the simulation
        :param bodies: list of Body objects, the bodies to simulate
        :param dt: float, time step in seconds
        :param forces_to_consider: list of callable, the forces to consider in the simulation
        :param json_file: str, path to a json file containing the simulation parameters
        :return: None
        :note: if json_file is provided, it will override the other parameters
            the aim is to provide an easy way to only have to pass a "simple" json file to define the whole simulation
        """
        self.current_time = 0.0
        if json_file is None:
            self.bodies = bodies
            self.dt = dt
            if type(forces_to_consider[0]) is not tuple:
                # e.g. : forces_to_consider=[gravitational_force]
                self.forces_to_consider = {elt.__name__ : (elt, {}) for elt in forces_to_consider}
            else:
                # e.g.: forces_to_consider=[(gravitational_force, {"g" : 9.81})]
                self.forces_to_consider = {elt[0].__name__ : (elt[0], elt[1]) for elt in forces_to_consider}
            # elif type(forces_to_consider) is dict:
            #     assert all([callable(elt[0]) and type(elt[1]) is dict for elt in forces_to_consider.items()]), "forces_to_consider must be a list of callable or a dict of callable and parameters"
        else:
            with open(json_file, 'r') as f:
                params = json.load(f)

            self.dt = params["parameters"]["dt"]

            # constructing the bodies
            self.bodies = []
            print(params["objects"])
            for key, body_params in params["objects"].items():
                print(key, body_params)
                body = Body(**body_params)
                print(body)
                self.bodies.append(body)

            # constructing the forces
            self.forces_to_consider = {}
            for key, value in params["forces"].items():
                self.forces_to_consider[key] = (eval(key), value)  # we assume that the force function is defined in the local scope




    def compute_forces(self):
        """
        compute all the forces acting on each body
        :note: used to be a separate function to ease the integration in the code
        """
        forces = {body.name: np.zeros(3, dtype='float64') for body in self.bodies}
        for force_key in self.forces_to_consider:
            force_func, params = self.forces_to_consider[force_key]
            force_results = force_func(self.bodies, **params)
            for body_name, force_vector in force_results.items():
                forces[body_name] += force_vector
        return forces
    

    def step(self):
        """
        this function will perform a single time step oof the simulation
        :note: it will applies the Velocity Verlet integration method.
        """
        self.current_time += self.dt
        # first, we compute the forces acting on each body
        forces = self.compute_forces()

        # then, we update the position and velocity of each body
        for body in self.bodies:
            # compute acceleration
            acceleration = forces[body.name] / body.mass

            # update position
            body.position += body.velocity * self.dt + 0.5 * acceleration * self.dt ** 2

        # compute new forces (for velocity update)
        new_forces = self.compute_forces()

        for body in self.bodies:
            # compute new acceleration
            new_acceleration = new_forces[body.name] / body.mass

            # update velocity
            body.velocity += 0.5 * (acceleration + new_acceleration) * self.dt
                