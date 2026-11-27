"""Causal FIR augmentation of the physical near-hover state."""

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from .near_hover_model import discrete_model


@dataclass(frozen=True, slots=True)
class IdentifiedHoverModelConfig:
    variant: str
    evidence_path: str
    coefficients: tuple[tuple[float, ...], ...]
    bias: tuple[float, ...]

    @property
    def history_slots(self) -> tuple[tuple[int, int], ...]:
        """Lag-major (zero-based history index, axis), including only needed taps."""
        return tuple(
            (lag, axis) for lag in range(self.history_steps)
            for axis, row in enumerate(self.coefficients) if lag + 1 < len(row)
        )

    @property
    def history_steps(self) -> int:
        return max(len(row) for row in self.coefficients) - 1

    @classmethod
    def from_report(cls, path: Path, variant: str):
        report = json.loads(path.read_text())
        if report["test_opened"] is not False:
            raise ValueError("model report does not certify an untouched test split")
        names = {"hybrid":"rate_FIR6_thrust_delay4","joint-fir6":"joint_FIR6"}
        if variant not in names:
            fit = report["variants"][variant]["parameters"]
            rows = [np.asarray(fit[str(axis)], dtype=float) for axis in range(4)]
            if any(row.ndim != 1 or len(row) not in range(2,27) or not np.isfinite(row).all() for row in rows):
                raise ValueError("invalid axis-specific FIR coefficient vector")
            return cls(variant,str(path),tuple(tuple(row[:-1]) for row in rows),
                       tuple(float(row[-1]) for row in rows))
        model=report["models"]["25"][names[variant]]["parameters"]
        packed=np.zeros((4,6))
        bias=[]
        for axis in range(4):
            row=model[str(axis)]
            taps,delay,ar=row["structure"]
            coeff=np.asarray(row["coefficients"])
            if ar or taps<1 or delay<0 or taps+delay>6 or len(coeff)!=taps+1 or not np.isfinite(coeff).all():
                raise ValueError("unsupported identification coefficient shape")
            packed[axis,delay:delay+taps]=coeff[:-1]
            bias.append(float(coeff[-1]))
        return cls(variant,str(path),tuple(tuple(row) for row in packed),tuple(bias))


def identified_discrete_model(base, candidate):
    """State: physical 9, only the required issued input taps, constant 1."""
    ad,bd=discrete_model(base)
    derivative_map=bd/np.array([1.,-1.,-1.,base.thrust_gain_m_s2])
    coeff = candidate.coefficients
    slots = {slot: i + 9 for i,slot in enumerate(candidate.history_slots)}
    size = 10 + len(slots)
    a=np.zeros((size,size))
    b=np.zeros((size,4))
    a[:9,:9]=ad
    b[:9]=derivative_map*np.asarray([row[0] for row in coeff])
    for (lag, axis), index in slots.items():
        a[:9,index] = derivative_map[:,axis]*coeff[axis][lag+1]
        if lag:
            a[index,slots[lag-1,axis]] = 1.
        else:
            b[index,axis] = 1.
    a[:9,-1]=derivative_map@np.asarray(candidate.bias)
    a[-1,-1]=1.
    return a,b
