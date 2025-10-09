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
import time
from core_calculation.force_definition_physical_interaction import *

class Simulation:
    """
    this class will provide a way to integrate and simulate the motion of a/several body/ies
    under the influence of forces.
    """

    def __init__(self, bodies:list[Body]=None, dt:np.float64=None, forces_to_consider:list | list[tuple]=None, class_forces_to_consider:list=None, json_file:str=None):
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
            self.class_forces_to_consider = class_forces_to_consider

        else:
            with open(json_file, 'r') as f:
                params = json.load(f)

            self.dt = params["parameters"]["dt"]

            # constructing the bodies
            self.bodies = []
            # print(params["objects"])
            for key, body_params in params["objects"].items():
                # print(key, body_params)
                body = Body(**body_params)
                # print(body)
                self.bodies.append(body)

            # constructing the forces
            self.forces_to_consider = {}
            for key, value in params["forces"].items():
                self.forces_to_consider[key] = (eval(key), value)  # we assume that the force function is defined in the local scope

            # constructing the class forces
            self.class_forces_to_consider = []
            if "class_forces" in params:
                for cle, class_force_params in params["class_forces"].items():
                    class_name = class_force_params.pop("class_name")
                    point1 = class_force_params.pop("point1")
                    point2 = class_force_params.pop("point2")
                    if type(point1) is str:
                        point1 = next(body for body in self.bodies if body.name == point1)
                    else:
                        point1 = np.array(point1, dtype='float64')
                    class_force_params["point1"] = point1

                    if type(point2) is str:
                        point2 = next(body for body in self.bodies if body.name == point2)
                    else:
                        point2 = np.array(point2, dtype='float64')
                    class_force_params["point2"] = point2
                    class_force = eval(class_name)(**class_force_params)
                    print(class_force)
                    self.class_forces_to_consider.append(class_force)



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

        # if there are class forces to consider (e.g. spring forces)
        for complex_force in self.class_forces_to_consider:
            force_results = complex_force.compute_force()
            for body_name, force_vector in force_results.items():
                if body_name:  # if body_name is False, it means the force is applied to a fixed point in space
                    forces[body_name] += force_vector
        return forces
    

    def step(self, update_bodies:bool=True, verbose=False):
        """
        this function will perform a single time step oof the simulation
        :note: it will applies the Velocity Verlet integration method.
        """
        if verbose: print("_"*20)
        self.current_time += self.dt
        # first, we compute the forces acting on each body
        forces = self.compute_forces()

        # then, we update the position and velocity of each body
        for body in self.bodies:
            # compute acceleration
            acceleration = forces[body.name] / body.mass
            if verbose: print("[ACCEL]", body.name, acceleration)
            if update_bodies:
                # update position
                if verbose: print(f"[POS UPT] {body.name} from {body.position} to ", end ="")
                body.position = body.position + body.velocity * self.dt + 0.5 * acceleration * self.dt ** 2
                if verbose: print(body.position)
            else:
                # only do the computation but not the update (useful for the benchmark)
                _ = body.position + body.velocity * self.dt + 0.5 * acceleration * self.dt ** 2

        # compute new forces (for velocity update)
        new_forces = self.compute_forces()

        for body in self.bodies:
            # compute new acceleration
            old_acceleration = forces[body.name] / body.mass
            new_acceleration = new_forces[body.name] / body.mass
            if verbose: print("[NEW ACCEL]", body.name, new_acceleration)
            if update_bodies:
                # update velocity
                if verbose : print(f"[VEL UPT] {body.name} from {body.velocity} to ", end="")
                body.velocity =  body.velocity + (old_acceleration + new_acceleration)/2 * self.dt
                if verbose : print(body.velocity)
                # body.velocity =  body.velocity + acceleration * self.dt
            else:
                # only do the computation but not the update (useful for the benchmark)
                _ = body.velocity +  (old_acceleration + new_acceleration)/2 * self.dt
                # _ = body.velocity +  acceleration * self.dt
                
        # print("x"*10)

    def __repr__(self):
        return f"Simulation(dt={self.dt}, bodies={self.bodies}, forces_to_consider={list(self.forces_to_consider.keys())})"
    
    def all_info_json(self):
        """
        this function will generate a json representation of the simulation
        the representation will be exhaustive. one should be able to run the simulation on this file and get the exact
        same results
        :note: two use case for thie function
        - save the simulation parameters to a file
        - send the simulation parameters through ZMQ to a GUI (to improve display)
        :return: str, json representation of the simulation
        """
        simu_dict = {
            "parameters": {
                "dt": self.dt
            },
            "objects": {body.name: {
                "mass": body.mass,
                "name": body.name,
                "init_position": body.position.flatten().tolist(),
                "init_velocity": body.velocity.flatten().tolist()
            } for body in self.bodies},
            "forces": {key: params for key, (func, params) in self.forces_to_consider.items()}
        }
        return simu_dict
    
    def benchmark_step(self, n_steps:int=1000):
        """
        this function will benchmark the time taken to perform n_steps of the simulation
        :param n_steps: int, number of steps to perform
        :return: float, average time per step (in seconds and in the current state of the computer)
        """
        start_time = time.time()
        for _ in range(n_steps):
            self.step(update_bodies=False)
        end_time = time.time()
        return (end_time - start_time)/n_steps  # return average time per step
    
    def compute_potentiel_energy(self):
        """
        compute the potential energy of each body submitted to the force
        will be useful to compute the total energy of the system and monitor the well behaviour of the system
        """
        potential_energy = {body.name: 0 for body in self.bodies}
        for force_key in self.forces_to_consider:
            force_func, params = self.forces_to_consider[force_key]
            potential_of_force = force_func(self.bodies, **params, potential=True)
            if potential_of_force == -1: continue
            for body_name, single_potential in potential_of_force.items():
                potential_energy[body_name] += single_potential

        # if there are class forces to consider (e.g. spring forces)
        for complex_force in self.class_forces_to_consider:
            potential_of_force = complex_force.compute_potential()
            if potential_of_force == -1: continue
            for cle, single_potential in potential_of_force.items():
                potential_energy[cle] += single_potential
        return potential_energy
    

if __name__ == "__main__":
    simu = Simulation(json_file="scenarii_examples\cannon_balls.json")
    print(simu)
    print(json.dumps(simu.all_info_json(), indent=4))