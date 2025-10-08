"""
some forces cannot be simply explained by just a function but need a more complex physical description
for example, a spring force depends on the position of two bodies and the spring constant
Plus we do not want to "link" the spring at all the bodies but only at two (or None)
That's is why we create the following class to define such complex forces.
:author: Maxence Barré
:date:2025
Below you will find a list of the available forces:
- Spring
"""

# importing libraries
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np

from core_calculation.body_definition import Body



class Spring:
    def __init__(self, point1:np.ndarray|Body, point2:np.ndarray|Body, k:float, rest_length:float=0):
        """
        this class will define a spring force between two points (or bodies)
        :param point1: first point or body where the spring is attached
        :param point2: second point or body where the spring is attached
        :param k: spring constant (N/m)
        :param rest_length: rest length of the spring (m)
        :note if point1 or point2 is a Body, the spring will be attached to the body's position (and therefore will move over time)
        if point1 or point2 is a np.ndarray, the spring will be attached to a fixed point in space
        """
        self.point1 = point1
        self.point2 = point2
        self.k = k
        self.rest_length = rest_length

    def compute_force(self) -> dict[str, np.ndarray]:
        """
        compute the spring force acting on the two points (or bodies)
        :return: a 2 elements dict with body names as keys and force vectors as values
        """
        # get positions of the points
        if isinstance(self.point1, Body):
            pos1 = self.point1.position
            name1 = self.point1.name
        else:
            pos1 = self.point1
            name1 = False

        if isinstance(self.point2, Body):
            pos2 = self.point2.position
            name2 = self.point2.name
        else:
            pos2 = self.point2
            name2 = False

        # compute the vector from point1 to point2
        vec = pos2 - pos1
        length = np.linalg.norm(vec)
        if length == 0:
            force_vec = np.array((0, 0, 0), dtype='float64')
        else:
            direction = vec / length
            # compute the spring force magnitude
            force_magnitude = self.k * (length - self.rest_length)
            # compute the force vector (pointing from point1 to point2)
            force_vec = force_magnitude * direction

        # the force on point1 is -force_vec and on point2 is force_vec (Newton's third law)
        forces = {}
        if name1:
            forces[name1] = force_vec
        if name2:
            forces[name2] = -force_vec
        # print(f"Spring force between {name1} and {name2}: {-force_vec if name1 else ''} {force_vec if name2 else ''}")
        return forces
    
    def __repr__(self):
        info_point1 = self.point1.name if isinstance(self.point1, Body) else self.point1
        info_point2 = self.point2.name if isinstance(self.point2, Body) else self.point2
        return f"Spring(point1={info_point1}, point2={info_point2}, k={self.k}, rest_length={self.rest_length})"
    
    def compute_potential(self):
        """
        compute the potential energy stored in the spring
        :return: potential energy (J)
        """
        # get positions of the points
        if isinstance(self.point1, Body):
            pos1 = self.point1.position
        else:
            pos1 = self.point1

        if isinstance(self.point2, Body):
            pos2 = self.point2.position
        else:
            pos2 = self.point2

        # compute the vector from point1 to point2
        vec = pos2 - pos1
        length = np.linalg.norm(vec)
        # compute the potential energy
        potential_total = 0.5 * self.k * (length - self.rest_length) ** 2

        if isinstance(self.point1, Body) and isinstance(self.point2, Body):
            ratio_for_point1 = self.point2.mass / (self.point1.mass + self.point2.mass)
            potential = {self.point1.name: potential_total * ratio_for_point1, self.point2.name: potential_total * (1 - ratio_for_point1)}
        elif isinstance(self.point1, Body): # point2 is fixed
            potential = {self.point1.name: potential_total}
        elif isinstance(self.point2, Body): # point1 is fixed
            potential = {self.point2.name: potential_total}
        else: # both points are fixed
            potential = {}

        return potential