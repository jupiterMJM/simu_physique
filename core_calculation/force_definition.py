"""
this file provide the definition of various forces that can be applied to bodies in a simulation
:author: Maxence Barré
:date: 2025
:note: z axis is vertical upwards axis
"""

# importing libraries
import numpy as np
from core_calculation.body_definition import Body


# gravitationnal force definition (weight mg force)
def no_force(bodies:list[Body]) -> dict[str, np.ndarray]:
    """
    this function will compute a zero force acting on each body
    :param bodies: list of bodies in the simulation
    :return: a dictionary with body names as keys and force vectors as values
    """
    forces = {}
    for body in bodies:
        forces[body.name] = np.array((0, 0, 0), dtype='float64')
    return forces

def gravitational_force(bodies:list[Body], g:np.float64=9.81) -> dict[str, np.ndarray]:
    """
    this function will compute the gravitational force acting on each body
    :param bodies: list of bodies in the simulation
    :param g: gravitational acceleration (default is 9.81 m/s^2)
    :return: a dictionary with body names as keys and force vectors as values
    """
    forces = {}
    for body in bodies:
        forces[body.name] = np.array((0, 0, -body.mass * g), dtype='float64')
    return forces