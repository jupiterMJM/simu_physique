"""
:author: Maxence Barré
:date: 2025
:description: this file provide a definition of a body in a simulation
"""

# importing libraries
import numpy as np

class Body:
    """
    what a body is. It will provide a simple way to define and use a body across the functions and files
    """

    def __init__(self, mass, name, init_position=None, init_velocity=None):
        self.mass = mass
        self.name = name
        self.position = np.array([0, 0, 0], dtype="float64") if init_position is None else init_position
        self.velocity = np.array([0, 0, 0], dtype="float64") if init_velocity is None else init_velocity

    @property
    def kinetic_energy(self):
        return 0.5 * self.mass * np.linalg.norm(self.velocity) ** 2