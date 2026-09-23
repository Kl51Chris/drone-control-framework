from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray


Vector3 = NDArray[np.float64]


def vector3(value: ArrayLike, name: str) -> Vector3:
    """Convert an input to a finite float64 vector with shape (3,)."""
    array = np.asarray(value, dtype=np.float64)

    if array.shape != (3,):
        raise ValueError(f"{name} must have shape (3,), got {array.shape}")

    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")

    return array.copy()


def finite_float(value: float, name: str) -> float:
    """Convert an input to a finite float."""
    result = float(value)

    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite")

    return result
