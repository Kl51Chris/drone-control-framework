from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ._state_validation import Vector3, finite_float, vector3


@dataclass(slots=True)
class DroneState:
    """
    Unified best-current drone state used by all controllers.

    This type is source-agnostic. A StateEstimator selects or combines
    source-specific EstimatedState and MeasuredState values to produce it.

    Coordinate and unit convention:
        position:
            World-frame position [x, y, z] in meters.

        velocity:
            World-frame velocity [vx, vy, vz] in meters/second.

        roll:
            Estimated roll angle in radians.

        pitch:
            Estimated pitch angle in radians.

        yaw:
            Estimated world-frame yaw angle in radians.

        p, q, r:
            Body-frame angular rates about the body x, y, and z axes,
            respectively, in radians/second. These are not generally equal
            to the Euler angle derivatives roll_dot, pitch_dot, yaw_dot.

        timestamp:
            Measurement time in seconds.
    """

    position: Vector3
    velocity: Vector3

    roll: float
    pitch: float
    yaw: float

    timestamp: float = 0.0

    # Defaults preserve compatibility with existing call sites that predate
    # body-rate feedback. State acquisition should provide real values.
    p: float = 0.0
    q: float = 0.0
    r: float = 0.0

    def __post_init__(self) -> None:
        self.position = vector3(
            self.position,
            "position",
        )
        self.velocity = vector3(
            self.velocity,
            "velocity",
        )

        self.roll = finite_float(
            self.roll,
            "roll",
        )
        self.pitch = finite_float(
            self.pitch,
            "pitch",
        )
        self.yaw = finite_float(
            self.yaw,
            "yaw",
        )
        self.timestamp = finite_float(
            self.timestamp,
            "timestamp",
        )
        self.p = finite_float(self.p, "p")
        self.q = finite_float(self.q, "q")
        self.r = finite_float(self.r, "r")

    @classmethod
    def zero(
        cls,
        timestamp: float = 0.0,
    ) -> DroneState:
        """Create a zero-valued drone state."""
        return cls(
            position=np.zeros(
                3,
                dtype=np.float64,
            ),
            velocity=np.zeros(
                3,
                dtype=np.float64,
            ),
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            timestamp=timestamp,
            p=0.0,
            q=0.0,
            r=0.0,
        )
