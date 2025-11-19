"""
:author: Maxence Barré
:date: 2025
:description: this file provide a definition of a body in a simulation
TODO orthonormaliser la base propre du corps (sinon c trop la merde pour gérer ca)
"""

# importing libraries
import numpy as np

class Body:
    """
    what a body is. It will provide a simple way to define and use a body across the functions and files
    """

    def __init__(self, mass=None, name=None, init_position=None, init_velocity=None,
                 initial_basis: np.array=None, **args): # init_position and init_velocity are at None bcs limitation of np.array that is run only once
        """
        initiate the physical body
        :param mass: float, mass of the body in kg
        :param name: str, name of the body
        :param init_position: np.array, initial position of the body in meters
        :param init_velocity: np.array, initial velocity of the body in m/s
        :param initial_base: np.array, initial base vectors of the body (for rotation)
            enable to represent a 3D body representation with orientation (solid body != point mass)
            must represent the matrix transition from Eulerian "main orthonormal basis" to the body basis
        :param kwargs: (type: dict) provide a way to pass directly all the parameters of the body
        :note: if same parameters are provided in kwargs and as explicit parameters, the explicit parameters will be used
                         and a warning will be raised
        :return: None
        :note: the kwargs will be very useful later when the bodies will have more parameters
        """
        self.mass = mass
        self.name = name

        # NOTE : this should not be used to pass argument
        # WHY? because to pass parameters to the __init__function, you should use the Body(**your_dict_with_params) to pass automatically the params here
        if args:
            print("args received in Body __init__:", args)
            if len(args) > 1:
                raise ValueError("Only one positional argument (dict) is allowed.")
            if not isinstance(args[0], dict):
                raise TypeError("Positional argument must be a dictionary.")
            kwargs = args[0]

            for key, value in kwargs.items():
                # wow i love python!!!!
                if hasattr(self, key):
                    if locals()[key] is not None:
                        print(f"Warning: {key} is provided both in kwargs and as explicit parameter. The explicit parameter will be used.")
                    else:
                        setattr(self, key, value)
                else:
                    raise ValueError(f"Unknown parameter {key} provided in kwargs.")
        
        # setting default values if not provided
        self.position = np.array([0, 0, 0], dtype="float64") if init_position is None else np.array(init_position, dtype="float64")
        self.velocity = np.array([0, 0, 0], dtype="float64") if init_velocity is None else np.array(init_velocity, dtype="float64")
        
        
        if initial_basis:
            self.representation = "3D_solid_body"
            if not check_orthonormal(np.array(initial_basis, dtype="float64")):
                print(f"[WARNING] The provided initial basis for body {self.name} is not orthonormal. It will be orthonormalised using the direct method.")
                basis = direct_orthonormalisation(np.array(initial_basis, dtype="float64"))
                self.basis = np.array(basis, dtype="float64")
            else:
                self.basis = np.array(initial_basis, dtype="float64")
        else:
            self.representation = "point_mass"
            self.basis = np.array([np.nan, np.nan, np.nan], dtype="float64")
            




    @property
    def kinetic_energy(self):
        """
        corps = Body()
        print(corps.kinetic_energy)
        """
        return 0.5 * self.mass * np.linalg.norm(self.velocity) ** 2
    
    def __repr__(self):
        """
        provide a string representation of the body
        :return: str, string representation of the body
        :note: will use the kwarrgs to provide all the parameters of the body
        """
        params = {key: value for key, value in self.__dict__.items() if not key.startswith('_')}
        for key, value in params.items():
            if isinstance(value, np.ndarray):
                params[key] = value.tolist()
        # params_str = ', '.join([f"{key}={value}" for key, value in params.items()])
        return f"Body({params})"
    

    def repr_json(self):
        """
        provide a json representation of the body
        :return: dict, json representation of the body
        """
        return {
                "mass": self.mass,
                "name": self.name,
                "init_position": self.position.flatten().tolist(),
                "init_velocity": self.velocity.flatten().tolist(),
                "representation": self.representation,
                "initial_base": self.basis.flatten().tolist(),
            }
    
    @property
    def euler_angle(self):
        """
        compute the Euler angles of the body from its basis
        :return: np.array, Euler angles of the body in radians
        in the yaw pitch roll (ZYX) convention
        """
        if self.representation != "3D_solid_body":
            raise ValueError(f"Body {self.name} is not a 3D solid body. Cannot compute Euler angles.")
        R = self.basis

        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6
        if not singular:

            theta = np.arctan2(R[2, 1], R[2, 2])
            phi = np.arctan2(-R[2, 0], sy)
            psi = np.arctan2(R[1, 0], R[0, 0])

    # @property
    # def position(self):
    #     return self._position

    # @position.setter
    # def position(self, value):
    #     # print(f"Position updated of {self.name} from {self._position} to {value}")
    #     self._position = np.array(value, dtype="float64")

    # @property
    # def velocity(self):
    #     return self._velocity
    
    # @velocity.setter
    # def velocity(self, value):
    #     # print(f"Velocity updated of {self.name} from {self._velocity} to {value}")
    #     self._velocity = np.array(value, dtype="float64")

    
def check_orthonormal(matrix):
    """
    Check if a matrix is orthonormal
    :param matrix: np.array, matrix to check
    :return: bool, True if the matrix is orthonormal, False otherwise
    """
    identity = np.eye(matrix.shape[0])
    product = np.dot(matrix.T, matrix)
    return np.allclose(product, identity)

def direct_orthonormalisation(transition_matrix):
    """
    Orthonormalisation of a base using the direct method
    :param transition_matrix: np.array, matrix representing the transition from the main basis to the body basis
    :return: np.array, orthonormalised transition matrix
    :note: the first axis will just be orthonormalised, the second will be made orthogonal to the first and then normalised,
           the third will be made orthogonal to the first two and then normalised
    """
    u1 = transition_matrix[:, 0]
    e1 = u1 / np.linalg.norm(u1)

    u2 = transition_matrix[:, 1] - np.dot(transition_matrix[:, 1], e1) * e1
    e2 = u2 / np.linalg.norm(u2)

    u3 = transition_matrix[:, 2] - np.dot(transition_matrix[:, 2], e1) * e1 - np.dot(transition_matrix[:, 2], e2) * e2
    e3 = u3 / np.linalg.norm(u3)

    orthonormal_matrix = np.column_stack((e1, e2, e3))
    return orthonormal_matrix

if __name__ == "__main__":
    import json
    body_dict = json.load(open("scenarii_examples/1_free_fall.json", "r"))
    print(body_dict["objects"]["baloon"])
    body = Body(**body_dict["objects"]["baloon"])
    print(body)