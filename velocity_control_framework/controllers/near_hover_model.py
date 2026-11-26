"""Fixed-yaw, small-angle CrazySim model for applied body-rate commands."""

from dataclasses import dataclass
import math

import numpy as np
from scipy.signal import cont2discrete


@dataclass(frozen=True, slots=True)
class NearHoverModelConfig:
    sample_period_s: float = 0.02
    gravity_m_s2: float = 9.81
    equilibrium_thrust: float = 0.47681863
    thrust_gain_m_s2: float = 14.495
    version: str = "cf2x-t350-hover-9-v1"

    def __post_init__(self) -> None:
        for value in (self.sample_period_s, self.gravity_m_s2, self.thrust_gain_m_s2):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("model period, gravity and thrust gain must be positive")
        if not 0.0 < self.equilibrium_thrust < 1.0:
            raise ValueError("equilibrium thrust must be strictly between zero and one")


def discrete_model(config: NearHoverModelConfig) -> tuple[np.ndarray, np.ndarray]:
    """Return ZOH matrices for [x,y,z,vx,vy,vz,roll,pitch,yaw]."""
    a = np.zeros((9, 9))
    a[:3, 3:6] = np.eye(3)
    a[3, 7] = -config.gravity_m_s2
    a[4, 6] = -config.gravity_m_s2
    b = np.zeros((9, 4))
    b[6:9, :3] = np.diag([1.0, -1.0, -1.0])
    b[5, 3] = config.thrust_gain_m_s2
    ad, bd, _, _, _ = cont2discrete(
        (a, b, np.eye(9), np.zeros((9, 4))), config.sample_period_s
    )
    return ad, bd


def world_to_fixed_yaw(yaw_rad: float) -> np.ndarray:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.asarray([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
