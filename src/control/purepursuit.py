from __future__ import division
import numpy as np

from control.controller import BaseController
from control.controller import compute_position_in_frame


class PurePursuitController(BaseController):
    def __init__(self, **kwargs):
        self.car_length = kwargs.pop("car_length")

        # Get the keyword args that we didn't consume with the above initialization
        super(PurePursuitController, self).__init__(**kwargs)


    def get_error(self, pose, reference_xytv):
        """Compute the Pure Pursuit error.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed

        Returns:
            error: Pure Pursuit error
        """
        return compute_position_in_frame(reference_xytv[:3], pose)

    def get_control(self, pose, reference_xytv, error):
        """Compute the Pure Pursuit control law.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed
            error: error vector from get_error

        Returns:
            control: np.array of velocity and steering angle
        """
        # BEGIN QUESTION 3.1
        L = self.car_length
        squared = error[0]**2 + error[1]**2
        theta =  np.arctan(2.0 * L * error[1] / squared)
        velocity = reference_xytv[3]
        return np.array([velocity, theta])
        # END QUESTION 3.1
