"""Deterministic bounded multisine for CrazySim hover identification."""

import numpy as np
from scipy.signal import firwin


class HoverExcitation:
    def __init__(self, seed: int, axis: str, max_hz: float, kind: str = "multisine") -> None:
        rng = np.random.default_rng(seed)
        time = np.arange(96) * 0.02
        frequencies = np.linspace(0.65, max_hz, 24)[:, None]
        phase = rng.uniform(0, 2 * np.pi, (24, 4))
        weights = rng.normal(size=(24, 4))
        signal = np.sum(weights * np.sin(2 * np.pi * time[:, None, None] * frequencies + phase), axis=1)
        if kind == "filtered":
            taps = firwin(7, max_hz, fs=50)
            noise = rng.choice((-1., 1.), (len(time) + len(taps) - 1, 4))
            signal = np.column_stack([np.convolve(col, taps, mode="valid") for col in noise.T])
        signal -= np.mean(signal, axis=0)
        signal /= np.max(np.abs(signal), axis=0)
        amplitude = np.array([0.045, 0.045, 0.045, 0.009])
        if axis != "all":
            amplitude *= np.arange(4) == ("roll", "pitch", "yaw", "thrust").index(axis)
        self.values = signal * amplitude
        self.seed = seed
        self.axis = axis
        self.max_hz = max_hz
        self.kind = kind

    def sample(self, elapsed: float) -> np.ndarray:
        index = round((elapsed - 0.20) / 0.02)
        return self.values[index] if 0 <= index < len(self.values) else np.zeros(4)
