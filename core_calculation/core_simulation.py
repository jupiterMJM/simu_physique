"""
:author: Maxence Barré
:date: 2025
:description: this file will integrate Newton's law to calculate the position and velocity of
a body at each time step.
:note: the integration method used here is the Velocity Verlet method.
:note: we place ourseslves in a 3D space with x, y and z axis.
"""

# importing libraries
import numpy as np
from core_calculation.body_definition import Body

class Simulation:
    """
    this class will provide a way to integrate and simulate the motion of a/several body/ies
    under the influence of forces.
    """

    def __init__(self, bodies:list[Body], dt:np.float64, forces_to_consider:list[callable]):
        self.bodies = bodies
        self.dt = dt


    def compute_forces(self):
        """
        compute all the forces acting on each body
        :note: used to be a separate function to ease the integration in the code
        """
        forces = {body.name: np.zeros(3, dtype='float64') for body in self.bodies}
        for force in self.forces_to_consider:
            force_results = force(self.bodies)
            for body_name, force_vector in force_results.items():
                forces[body_name] += force_vector
        return forces
    
    
    def step(self):
        """
        this function will perform a single time step oof the simulation
        :note: it will applies the Velocity Verlet integration method.
        """
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

            # compute new acceleration
            new_acceleration = new_forces[body.name] / body.mass

            # update velocity
            body.velocity += 0.5 * (acceleration + new_acceleration) * self.dt
                
        