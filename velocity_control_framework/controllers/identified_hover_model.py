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

    @classmethod
    def from_report(cls, path: Path, variant: str):
        names = {"hybrid":"rate_FIR6_thrust_delay4","joint-fir6":"joint_FIR6"}
        report = json.loads(path.read_text())
        if report["test_opened"] is not False:
            raise ValueError("model report does not certify an untouched test split")
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
    """State: physical 9, five issued input vectors (newest first), constant 1."""
    ad,bd=discrete_model(base)
    derivative_map=bd/np.array([1.,-1.,-1.,base.thrust_gain_m_s2])
    coeff=np.asarray(candidate.coefficients)
    a=np.zeros((30,30))
    b=np.zeros((30,4))
    a[:9,:9]=ad
    b[:9]=derivative_map*coeff[:,0]
    for lag in range(1,6):
        a[:9,9+4*(lag-1):9+4*lag]=derivative_map*coeff[:,lag]
    a[:9,-1]=derivative_map@np.asarray(candidate.bias)
    b[9:13]=np.eye(4)
    a[13:29,9:25]=np.eye(16)
    a[-1,-1]=1.
    return a,b
