import marimo

__generated_with = "0.17.8"
app = marimo.App(width="medium")


@app.cell
def _():
    import marimo as mo
    return (mo,)


@app.cell
def _(mo):
    mo.md(r"""
    # How to represent rotation in a 3D space?
    In the case of the 3D-simulation project, one must move from point-mass objects to objects with orientation in 3D space. This will be needed to provide deeper information on the physical phenomenons.
    In this marimo file, the following point will be treated :

    - transfer matrix from global to local basis (and the other way around)
    - difference from Euler and Tait–Bryan basis
    - moving from transfer matrix representation to the other in Yaw-Roll-Pitch (ZYX)
    - definition of quaternions
    - from quaternions to everything else
    - conclusion with software optimization
    """)
    return


@app.cell
def _():
    import numpy as np
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from scipy.spatial.transform import Rotation as R
    return R, np, plt


@app.cell
def _(np, plt):
    def plot_basis(*all_basis, ax=None, scale=1.0, label="Basis"):
        """
        Plot a 3D orthonormal basis (X, Y, Z), each vector a numpy array of shape (3,).
        Colors: X=red, Y=green, Z=blue.
        """
        print("hello")
        if ax is None:
            fig = plt.figure(figsize=(6, 6))
            ax = fig.add_subplot(111, projection='3d')

        origin = np.zeros(3)

        # Draw vectors
        color_possible = ("r", "g", "b", "yellow", "grey")
        for i, single_basis in enumerate(all_basis):
            X, Y, Z = single_basis
            ax.quiver(*origin, *X, color=color_possible[i], linestyle="solid")
            ax.quiver(*origin, *Y, color=color_possible[i], linestyle="dashed")
            ax.quiver(*origin, *Z, color=color_possible[i], linestyle="dotted")

        # Set limits
        max_range = scale
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([-max_range, max_range])

        # Labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f"3D Basis: {label}")

        # Equal aspect ratio
        ax.set_box_aspect([1, 1, 1])

        # plt.legend()
        return ax
    return (plot_basis,)


@app.cell
def _(mo):
    mo.md(rf"""
    ## Transfer matrix from one basis to the other
    Let's consider one "main" basis that does not move and that is considered as the main calculation basis. Let's also consider another basis that could be link to a 3D physical body. The question is how to go from one basis to the other (to express calculated vectors for example).

    We will note e_x, e_y, e_z the vectors of the main basis. And we will note e_1, e_2, e_3 the vectors of the local basis.
    We assume that all vectors of each basis are orthonormal.
    Same any vector noted as v_L will be expressed in the local basis while v_G will be expressed in the global basis.
    One should also note that e_1, e_2 and e_3 are expressed in the global basis

    Let's first consider the matrix $$R = (e_1, e_2, e_3)$$
    Reusing the numpy formalism, one has : $R[:, 0] = e_1$, $R[:, 1] = e_2$, $R[:, 2] = e_3$

    This matrix R is therefore the transfer matrix from the local basis towards the global basis, meaning
    $$\forall v_L, v_G = R v_L$$ (with matrix product)

    In addition, one of the main property of R is the fact that the operation of $R^{-1}$ should send a vector in the global basis towards the local basis. Therefore, one has: $$R^T = R^{-1} so \forall v_G, v_L = R^T v_G$$

    In conclusion, one should remember :
    - transfer matrix must be implemented coherently with $R = (e_1, e_2, e_3)$
    - $R = local \rightarrow global$ and $R^T = global \rightarrow local$
    - it will be a great mess if the definition of the transfer matrix is messed up at the beginning
    """)
    return


@app.cell
def _(mo):
    mo.md(r"""
    What has been said before might lead to a question : how to ensure that a transfer matrix (assuming orthonormal) is well defined, meaning that R=(e_1, e_2, e_3) (and not (e_1, e_2, e_3)^T ).
    To do so, there is no solution. the only way is to be sure to well implement the program at the very beginning!
    """)
    return


@app.cell
def _(np):
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

        # --- 3. Check if it is identity (the bad case) ---
        if np.allclose(matrice, np.eye(3), atol=1e-8):
            return False   # defined as (e1, e2, e3)^T

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

    e_1 = np.array([1, 0, 0]).T
    e_2 = np.array([0, 1, 0]).T
    e_3 = np.array([0, 0, 1]).T
    test = np.column_stack((e_1, e_2, e_3))
    print("Should be True: ", check_good_definition_transfer_matrix(test))
    print("Should be True bcs .T: ", check_good_definition_transfer_matrix(test.T))



    e_1 = np.array([1/np.sqrt(2), 1/np.sqrt(2), 0])
    e_2 = np.array([-1/np.sqrt(6), 1/np.sqrt(6), np.sqrt(2/3)])
    e_3 = np.array([1/np.sqrt(3), -1/np.sqrt(3), 1/np.sqrt(3)])
    test = np.column_stack((e_1, e_2, e_3))
    print(test)

    print("Should be True: ", check_good_definition_transfer_matrix(test))
    print("Should be False: ", check_good_definition_transfer_matrix(test.T))
    return check_good_definition_transfer_matrix, e_1, e_2, e_3, test


@app.cell
def _(e_1, e_2, e_3, mo, np, plot_basis, plt):
    e_x = np.array([1, 0, 0])
    e_y = np.array([0, 1, 0])
    e_z = np.array([0, 0, 1])
    plot_basis([e_x, e_y, e_z], [e_1, e_2, e_3])
    mo.mpl.interactive(plt.gcf())
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## Euler Basis vs Tait–Bryan Basis
    ### Euler Basis
    An **Euler basis** is defined by **three intrinsic rotations** where **the first and last rotation axes are the same**.
    Meaning that the rotation pattern follow A-B-A (with A!=B)

    **Properties**
    - Uses rotations about axes expressed in the **rotating (local) frame**.
    - Has a **symmetry**: the first and last axes coincide.
    - Common in several things but i don t care

    The main idea of the rotation process is:
    Rotate around one axis, tilt around a different axis, rotate again around the original axis.


    ### Trait-Bryan Basis
    A **Tait–Bryan basis** is defined by **three intrinsic rotations** where  **all three axes are distinct**. The rotation pattern is therefore now A-B-C (with all three different vectors). (ex: ZYX also named yaw pitch roll)

    **Properties**
    - Three different axes → **no symmetry**.
    - Most intuitive for vehicles and robotics.
    - Matches how aircraft, drones, and robots orient themselves.


    **Small-precision**
    In the following, one will use the following convention:
    - Euler basis will be in ZXZ convention
    - Trait Bryan will be in ZYX convention (also called yaw-pitch-roll)
    """)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    ## Moving from transfer matrix representation to the other in Yaw-Roll-Pitch (ZYX)
    ### Euler basis
    In the ZXZ convention, the process is the following:
    1. initially both the basis are superposed
    2. Z: we turn of angle $\alpha$ around the e_3 (or e_z) axis (so e_1 and e_x are not superposed anymore, same for e_2 and e_y)
    3. X: we turn of angle $\beta$ around the e_1 axis
    4. Z: we turn of angle $\gamma$ around the e_3 axis

    One has (with X:=e_1, Y:=e_2, Z:=e_3)
    $$\alpha= \arccos \!\left( \frac{-Z_2}{\sqrt{1 - Z_3^2}}\right)$$
    $$\beta  = \arccos(Z_3)$$
    $$\gamma = \arccos\!\left( \frac{Y_3}{\sqrt{1 - Z_3^2}} \right)$$
    """)
    return


@app.cell
def _(R, e_1, e_2, e_3, np, test):
    def compute_angles_euler(e_1, e_2, e_3):
        Z2 = e_3[1]
        Z3 = e_3[2]
        Y3 = e_2[2]
        Z1 = e_3[0]
        Z2 = e_3[1]
        X3 = e_1[2]

        # Guard against numerical issues (clip inside [-1, 1])
        denom = np.sqrt(max(1 - Z3**2, 0.0))
        print(denom, Y3 / denom, Y3)

        # alpha = np.arccos(np.clip(-Z2 / denom, -1.0, 1.0))
        alpha = np.arctan2(Z1, -Z2)
        beta  = np.arccos(np.clip(Z3, -1.0, 1.0))
        gamma = np.arctan2(X3, Y3)
        # gamma = np.arccos(Y3 / denom)

        return alpha, beta, gamma


    alpha, beta, gamma = compute_angles_euler(e_1, e_2, e_3)

    t_from_matrix = R.from_matrix(test)
    print(test)
    print("alpha =", alpha)
    print("beta  =", beta)
    print("gamma =", gamma)
    print(t_from_matrix.as_euler("ZXZ"))    # return alpha, beta, gamma !!!! ca on aime!!!!
    return alpha, beta, gamma, t_from_matrix


@app.cell
def _(R, alpha, beta, e_1, e_2, e_3, gamma, mo, np, plot_basis, plt, test):
    # the following function does not work very well, idk why and i don t care bcs module already does the work
    def euler_to_basis(alpha, beta, gamma):
        """
        DOES NOT WORK!!!!!
        Reconstruction of the orthonormal basis (X, Y, Z) from
        Euler angles alpha, beta, gamma consistent with the formulas:

            alpha = arccos( -Z2 / sqrt(1 - Z3^2) )
            beta  = arccos(  Z3 )
            gamma = arccos(  Y3 / sqrt(1 - Z3^2) )

        This corresponds to a ZXZ Euler sequence.
        """

        # Z vector from beta and alpha
        Z3 = np.cos(beta)
        sin_beta = np.sin(beta)

        # Handle singularity when sin(beta)=0
        if abs(sin_beta) < 1e-12:
            raise ValueError("Gimbal lock: sin(beta)=0, Z axis undefined.")

        Z2 = -sin_beta * np.cos(alpha)
        Z1 =  sin_beta * np.sin(alpha)
        Z = np.array([Z1, Z2, Z3])

        # Partial Y vector (Y3 known)
        Y3 = sin_beta * np.cos(gamma)

        # Pick any reference vector not parallel to Z
        if abs(Z3) < 0.9:
            ref = np.array([0.0, 0.0, 1.0])
        else:
            ref = np.array([1.0, 0.0, 0.0])

        # u = normalized projection of ref onto plane ⟂ Z
        u = ref - np.dot(ref, Z) * Z
        u /= np.linalg.norm(u)

        # v = Z × u
        v = np.cross(Z, u)
        v /= np.linalg.norm(v)

        # ----------------------------------------------
        # Now apply gamma rotation in the (u,v) plane:
        #
        # In ZXZ:
        #   after rotating Z, gamma is a rotation around Z
        #   that defines the in-plane direction of Y.
        # ----------------------------------------------

        Y = np.cos(gamma) * v + np.sin(gamma) * u
        Y /= np.linalg.norm(Y)

        # X = Y × Z ensures right-handed orthonormal basis
        X = np.cross(Y, Z)

        # Normalize numerical errors
        X /= np.linalg.norm(X)
        Y /= np.linalg.norm(Y)
        Z /= np.linalg.norm(Z)

        # Return matrix with basis vectors as columns
        return X, Y, Z

    t_matrix = R.from_euler('ZXZ', [alpha, beta, gamma]).as_matrix()
    t1, t2, t3 = t_matrix[:, 0], t_matrix[:, 1], t_matrix[:, 2]
    print(test)
    print("alpha =", alpha)
    print("beta  =", beta)
    print("gamma =", gamma)
    print(euler_to_basis(alpha, beta, gamma))
    print("scipy", t_matrix)
    plot_basis([e_1, e_2, e_3], euler_to_basis(alpha, beta, gamma), [t1, t2, t3])
    # plot_basis(euler_to_basis(alpha, beta, gamma))
    mo.mpl.interactive(plt.gcf())
    return


@app.cell
def _(mo):
    mo.md(r"""
    ### Trait Bryan angle
    In the ZYX convention (yaw-pitch-roll), the process is the following:
    1. initially both the basis are superposed
    2. Z: we turn of angle $\psi$ around the e_3 (or e_z) axis
    3. Y: we turn of angle $\theta$ around the e_2 axis
    4. X : we turn of angle $\phi$ around the e_1 axis
    """)
    return


@app.cell
def _(e_1, e_2, e_3, np, t_from_matrix):
    def compute_angle_tait_bryan(X, Y, _):
        """
        Compute psi, theta, phi from basis vectors X, Y.

        X, Y are NumPy arrays of shape (3,).
        """

        X1, X2, X3 = X
        Y1, Y2, Y3 = Y

        denom = np.sqrt(1 - X3**2)

        psi   = np.arcsin(X2 / denom)
        theta = np.arcsin(-X3)
        phi   = np.arcsin(Y3 / denom)

        return psi, theta, phi

    psi, theta, phi = compute_angle_tait_bryan(e_1, e_2, e_3)
    print("psi ", psi)
    print("theta ", theta)
    print("phi ", phi)
    print(t_from_matrix.as_euler("ZYX"))        # encore une fonction qui fait tout automatiquement! (et en mieux)
    return


@app.cell
def _(mo):
    mo.md(r"""
    ## A quick overview on quaternions
    Quickly, a quaternion is a mathematical object to represent rotation operators in 3D. It is generally more stable than rotation (transfer) matrix and does not suffer of gimbal lock problem (as Euler angle representation does). One will note go deeper in how this work, we will just kinda summarize the main properties:
    1. let's consider a quaternion $q$ and we consider an agular speed $\omega_L$ represented in the local basis with $\omega_L= [\omega_1, \omega_2, \omega_3]$. Then, we have the kinematic quaternionic equation :
    $$\frac{dq}{dt} = \frac{1}{2} q \otimes \omega_q$$
    with $\omega_q = [0, \omega_1, \omega_2, \omega_3]$
    2. To get quaternions from rotation matrix or euler angle, one will use what scipy might offer. especially with:
    """)
    return


@app.cell
def _(t_from_matrix):
    t_from_matrix.as_quat(scalar_first=True)    # quaternionic representation!!!
    # note we ll always specify scalar_first=True to fit with theory work, however specific caution must be kept in mind about this!!!! (that s also why we implement a class)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    ## Implemetation of local basis : a good conclusion
    To end this sort of report/hands-on, we will propose a class implementation of local basis, with transfer matrix, euler and tait-brayans representation, quaternions, plotting....
    """)
    return


@app.cell
def _(R, check_good_definition_transfer_matrix, np, plt):
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

            self._quaternion = temp_local_basis.as_quat(scalar_first=True)
            self._local_basis = R.from_quat(self._quaternion, scalar_first=True)
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
        
        
            

    return (LocalBasis,)


@app.cell
def _(LocalBasis, e_1, e_2, e_3, mo, np, plt):
    t_lb = LocalBasis((e_1, e_2, e_3))
    print(t_lb._local_basis)
    print(t_lb._quaternion)
    print(t_lb.euler_angle)
    print(t_lb.trait_bryan_angle)
    print(t_lb.basis_matrix)

    t_lb2 = LocalBasis(euler_angle=(np.pi/2, 2*np.pi/3, 4))
    print(t_lb2._local_basis)
    print(t_lb2._quaternion)
    print(t_lb2.euler_angle)
    print(t_lb2.trait_bryan_angle)
    print(t_lb2.basis_matrix)

    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection='3d')
    t_lb.plot_basis(ax)
    t_lb2.plot_basis(ax, color="blue")
    mo.mpl.interactive(plt.gcf())
    return


@app.cell
def _():
    return


if __name__ == "__main__":
    app.run()
