"""CrazySim host-side observation perturbation, independent of native sensors."""

from dataclasses import dataclass, replace
import math

import numpy as np


@dataclass(frozen=True, slots=True)
class MeasurementNoise:
    profile: str
    amplitude: float
    seed: int

    def __post_init__(self):
        if self.profile not in ("none","iid-small"):
            raise ValueError("unknown host measurement profile")
        if not math.isfinite(self.amplitude) or not 0<=self.amplitude<=3 or self.seed<0:
            raise ValueError("invalid noise amplitude or seed")

    def apply(self,state,step):
        original=np.r_[state.position,state.velocity,state.roll,state.pitch,state.yaw,state.p,state.q,state.r]
        noise=np.zeros(12)
        if self.profile!="none" and self.amplitude:
            rng=np.random.default_rng(np.random.SeedSequence([self.seed,step]))
            scale=np.repeat([.001,.005,.001,.01],3)
            noise=self.amplitude*scale*rng.standard_normal(12)
        if not np.any(noise):
            return state,{"step":step,"raw":original.tolist(),"delta":noise.tolist(),"used":original.tolist()}
        used=original+noise
        used[8]=math.remainder(used[8],2*math.pi)
        observed=replace(state,position=used[:3],velocity=used[3:6],roll=used[6],
                         pitch=used[7],yaw=used[8],p=used[9],q=used[10],r=used[11])
        return observed,{"step":step,"raw":original.tolist(),"delta":noise.tolist(),"used":used.tolist()}
