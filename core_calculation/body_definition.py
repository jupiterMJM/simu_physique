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

    def __init__(self, mass, name, init_condition=(0, 0)):
        self.mass = mass
        self.name = name
        self.position = np.array(init_condition, dtype='float64')
        self.velocity = np.array((0, 0), dtype='float64')

    @property
    def kinetic_energy(self):
        return 0.5 * self.mass * np.linalg.norm(self.velocity) ** 2