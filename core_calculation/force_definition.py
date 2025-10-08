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

def gravitational_force(bodies:list[Body], g:np.float64=9.81, potential=False) -> dict[str, np.ndarray]:
    """
    this function will compute the gravitational force acting on each body
    :param bodies: list of bodies in the simulation
    :param g: gravitational acceleration (default is 9.81 m/s^2)
    :return: a dictionary with body names as keys and force vectors as values
    """
    if not potential:
        forces = {}
        for body in bodies:
            forces[body.name] = np.array((0, 0, -body.mass * g), dtype='float64')
        return forces
    else:
        potentials = {}
        for body in bodies:
            potentials[body.name] = body.mass * g * body.position[2]
        return potentials

def damping_force(bodies:list[Body], k:np.float64|dict=None):
    """
    this function will compute a damping force acting on each body
    :param bodies: list of bodies in the simulation
    :param k: damping coefficient
        if np.float64: same damping coefficient for all bodies
        if dict: damping coefficient for each body (body name as key and coefficient as value)
            in here if a body name is not in the dict, the damping force will be zero for this body (no damping)
    :return: a dictionary with body names as keys and force vectors as values
    :note: k can be linked to the exponential decay of the total mechanical energy as
    k = m/(2tau)ln(1/r) where tau is the time constant of the exponential decay and r the ratio of energy lost after tau
    """
    forces = {}
    for body in bodies:
        if isinstance(k, dict):
            if body.name in k:
                forces[body.name] = -k[body.name] * body.velocity
            else:
                forces[body.name] = np.array((0, 0, 0), dtype='float64')
        elif isinstance(k, (int, float, np.float64)):
            forces[body.name] = -k * body.velocity
        else:
            forces[body.name] = np.array((0, 0, 0), dtype='float64')
    return forces