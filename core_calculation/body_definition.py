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

    def __init__(self, mass=None, name=None, init_position=None, init_velocity=None, **args): # init_position and init_velocity are at None bcs limitation of np.array that is run only once
        """
        initiate the physical body
        :param mass: float, mass of the body in kg
        :param name: str, name of the body
        :param init_position: np.array, initial position of the body in meters
        :param init_velocity: np.array, initial velocity of the body in m/s
        :param kwargs: (type: dict) provide a way to pass directly all the parameters of the body
        :note: if same parameters are provided in kwargs and as explicit parameters, the explicit parameters will be used
                         and a warning will be raised
        :return: None
        :note: the kwargs will be very useful later when the bodies will have more parameters
        """
        self.mass = mass
        self.name = name
        # self.position = np.array(init_position) if init_position is not None else None
        # self.velocity = np.array(init_velocity) if init_velocity is not None else None


        # if args:
        #     if len(args) > 1:
        #         raise ValueError("Only one positional argument (dict) is allowed.")
        #     if not isinstance(args[0], dict):
        #         raise TypeError("Positional argument must be a dictionary.")
        #     kwargs = args[0]

        #     for key, value in kwargs.items():
        #         # wow i love python!!!!
        #         if hasattr(self, key):
        #             if locals()[key] is not None:
        #                 print(f"Warning: {key} is provided both in kwargs and as explicit parameter. The explicit parameter will be used.")
        #             else:
        #                 setattr(self, key, value)
        #         else:
        #             raise ValueError(f"Unknown parameter {key} provided in kwargs.")
        
        # setting default values if not provided
        self.position = np.array([0, 0, 0], dtype="float64") if init_position is None else np.array(init_position, dtype="float64")
        self.velocity = np.array([0, 0, 0], dtype="float64") if init_velocity is None else np.array(init_velocity, dtype="float64")
            




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

    

if __name__ == "__main__":
    import json
    body_dict = json.load(open("scenarii_examples/free_fall.json", "r"))
    print(body_dict["objects"]["baloon"])
    body = Body(**body_dict["objects"]["baloon"])
    print(body)