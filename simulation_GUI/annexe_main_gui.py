"""
secondary function used in the primary main_gui.py file
"""

import numpy as np


def compute_kinetic_energy(mass:dict, velocity:dict[str, np.ndarray]):
    """
    compute the kinetic energy of each system over the np.ndarray of velocities
    :param mass: a dict with the mass of each body
    :param velocity: a dict with the velocity of each body
    :return: a dict with the kinetic energy of each body
    """
    kinetic_energy = {}
    for name, vel in velocity.items():
        m = mass[name]
        v_squared = np.sum(vel**2, axis=1)  # Compute the squared velocity (v^2)
        kinetic_energy[name] = 0.5 * m * v_squared
    return kinetic_energy