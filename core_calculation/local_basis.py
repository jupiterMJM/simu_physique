"""
this file defines local basis that will be used for the Body class
:note: see marimo file quaternion_hands_on.py
:date: 2025
:author: Maxence Barré
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

def quat_mul(q, p):
    w1, x1, y1, z1 = q
    w2, x2, y2, z2 = p
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


def check_good_definition_transfer_matrix(matrice: np.ndarray):
    """
    Check that the transfer matrix moves from the local basis to the global basis.

    :param matrice: np.array, the 3x3 transfer matrix
    :return:
        True  -> if well-defined change-of-basis matrix
        False -> if it is just the canonical basis (e1, e2, e3)
    :raises:
        ValueError -> if the matrix is not valid for any basis transfer
    """

    matrice = np.asarray(matrice)

    # --- 1. Check shape ---
    if matrice.shape != (3, 3):
        raise ValueError("Transfer matrix must be 3x3.")

    # --- 2. Check invertibility ---
    if np.linalg.det(matrice) == 0:
        raise ValueError("Transfer matrix must be invertible.")

    # # --- 3. Check if it is identity (the bad case) ---
    # if np.allclose(matrice, np.eye(3), atol=1e-8):
    #     return False   # defined as (e1, e2, e3)^T

    # --- 4. Check if it is a proper rotation matrix (orthonormal + det=+1) ---
    # Column-vectors must form an orthonormal basis
    ortho = np.allclose(matrice.T @ matrice, np.eye(3), atol=1e-6)
    det_is_one = np.isclose(np.linalg.det(matrice), 1.0, atol=1e-6)

    if ortho and det_is_one:
        return True

    # --- 5. If not orthonormal, it's invalid ---
    raise ValueError(
        "Matrix is invertible but does not represent a valid orthonormal basis transfer."
    )

class LocalBasis:
    """
    this class will implement an easy way to work with rotation operators to easily implement solid-body in 3D simulation
    :note: this class works as a conclusion of this report, HOWEVER there might be some changes with what is actually used in the project. So refer to the good file for the 'real' implementation.
    :note: any operation of modification of the local basis (eg, an angular velocity will be applied on the quaternion!)
    """

    def __init__(self, list_of_axis:list[np.array]=None, euler_angle:list[float]=None, trait_bryan_angle:list[float]=None, quaternion:np.array=None):
        """
        an easy way to instantiate the local basis
        :note: we offer the way to instantiate the basis with almost every possible way. However, if several argument are passed it will raise an error!
        :param list_of_axis : to initiate with the list of the vector that generate the basis (must be orthonormal) in the global basis
        :param euler_angle : the list of (alpha, beta, gamma) in the ZXZ representation for euler angle in radians
        :param trait_bryan_angle : the list of (psi, theta, phi) in the ZYX representation (TODO : check if it is theta or -theta) !!! in radians
        :param quaternion: a 4*1 or 1*4 (will be flattened any way) array that represent the quaternion, NOTE ABOUT THE CONVENTION : the quaternion must be : [q_w, q_x, q_y, q_z] (eventhough it is not following scipy default way, but it is the "mathematical" way from what i ve seen)
        """
        basis_already_defined=False
        if list_of_axis is not None:
            basis_already_defined = True
            # the basis is initialize with transfer matrix
            if len(list_of_axis) != 3:
                raise Exception(f"[LOCAL_BASIS] Number of elements not expected. Got {len(list_of_axis)} instead of 3")
            e_1, e_2, e_3 = list_of_axis
            matrix_R = np.column_stack((e_1, e_2, e_3))
            print(matrix_R)
            if matrix_R.shape != (3, 3):
                raise Exception(f"[LOCAL_BASIS] Number of elements not expected. Got {matrix_R.shape} instead of (3, 3)")
            if not check_good_definition_transfer_matrix(matrix_R):
                raise Exception(f"[LOCAL_BASIS] Matrix is not a rotation matrix. Initialization failed.")

            temp_local_basis = R.from_matrix(matrix_R)

        if euler_angle is not None:
            if len(euler_angle) != 3:
                raise Exception(f"[LOCAL_BASIS] Number of elements not expected. Got {len(euler_angle)} instead of 3")
            if not basis_already_defined :
                basis_already_defined = True
                temp_local_basis = R.from_euler("ZXZ", euler_angle)
            else:
                raise Exception("[LOCAL_BASIS] You try to initiate local basis by several ways. Remove one of them")

        if trait_bryan_angle is not None:
            if len(trait_bryan_angle) != 3:
                raise Exception(f"[LOCAL_BASIS] Number of elements not expected. Got {len(trait_bryan_angle)} instead of 3")
            if not basis_already_defined :
                basis_already_defined = True
                temp_local_basis = R.from_euler("ZYX", trait_bryan_angle)
            else:
                raise Exception("[LOCAL_BASIS] You try to initiate local basis by several ways. Remove one of them")

        if quaternion is not None:
            quaternion = quaternion.flatten()
            if np.array(quaternion).shape[0] != 4:
                raise Exception(f"[LOCAL_BASIS] Number of elements not expected. Got {len(np.array(quaternion).shape[0])} instead of 4")
            if not basis_already_defined :
                basis_already_defined = True
                temp_local_basis = R.form_quat(quaternion, scalar_first=True)
            else:
                raise Exception("[LOCAL_BASIS] You try to initiate local basis by several ways. Remove one of them")

        _quaternion = temp_local_basis.as_quat(scalar_first=True)
        self._local_basis = R.from_quat(_quaternion, scalar_first=True)
        print("done")


    @property
    def euler_angle(self):
        """
        return euler angle in radians (alpha, beta, gamma) ZXZ convention
        """
        return self._local_basis.as_euler("ZXZ")

    @property
    def trait_bryan_angle(self):
        """
        return trait-bryan angle in radians (also called euler angle) in ZYX convention
        """
        return self._local_basis.as_euler("ZYX")

    @property
    def basis_matrix(self):
        """
        return the transfer matrix FROM the local matrix TO the global matrix
        so in the case the local basis is defined as e_1, e_2, e_3 
        this function returns a 3*3 matrix whose COLUMN are e_1, e_2, e_3
        """
        return self._local_basis.as_matrix()
    
    @property
    def quaternion(self):
        """
        return the quaternion as a numpy array of shape (4,)
        convention : [q_w, q_x, q_y, q_z]
        """
        return self._local_basis.as_quat(scalar_first=True)


    def plot_basis(self, ax=None, scale=1.0, color="r"):
        """
        Plot a 3D orthonormal basis (X, Y, Z), each vector a numpy array of shape (3,).
        Colors: X=red, Y=green, Z=blue.
        """
        if ax is None:
            fig = plt.figure(figsize=(6, 6))
            ax = fig.add_subplot(111, projection='3d')

        origin = np.zeros(3)

        # Draw vectors
        base = self.basis_matrix
        X, Y, Z = base[:, 0], base[:, 1], base[:, 2]
        ax.quiver(*origin, *X, color=color, linestyle="solid")
        ax.quiver(*origin, *Y, color=color, linestyle="dashed")
        ax.quiver(*origin, *Z, color=color, linestyle="dotted")

        # Set limits
        max_range = scale
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([-max_range, max_range])

        # Labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f"3D Basis")

        # Equal aspect ratio
        ax.set_box_aspect([1, 1, 1])

        # plt.legend()
        return ax


