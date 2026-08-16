from dataclasses import dataclass
import numpy as np


@dataclass
class AltitudePD:
    """PD controller for vertical altitude control."""

    hover_thrust: float = 0.26487  # N, measured from this MuJoCo model
    kp: float = 0.10               # N / m
    kd: float = 0.07               # N / (m/s)
    thrust_min: float = 0.0        # N
    thrust_max: float = 0.35       # N

    def compute(self, z: float, vz: float, z_ref: float) -> float:
        """Return collective thrust command in Newtons."""
        position_error = z_ref - z

        thrust = (
            self.hover_thrust
            + self.kp * position_error
            - self.kd * vz
        )

        return float(np.clip(thrust, self.thrust_min, self.thrust_max))