"""Source-derived calibration constants for CrazySim's default cf2x_T350."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class HoverThrustEstimate:
    mass_kg: float
    gravity_m_s2: float
    maximum_thrust_per_motor_n: float
    weight_n: float
    thrust_per_motor_n: float
    raw_pwm: float
    normalized_command: float


def estimate_cf2x_t350_hover_thrust(
    *,
    mass_kg: float = 0.0379,
    gravity_m_s2: float = 9.81,
    maximum_thrust_per_motor_n: float = 0.18,
) -> HoverThrustEstimate:
    """Map weight balance through CrazySim PWM and project command scaling.

    Assumes the default model, no ``--mass`` override, level steady flight,
    and no optional ground-effect multiplier.
    """
    if mass_kg <= 0 or gravity_m_s2 <= 0 or maximum_thrust_per_motor_n <= 0:
        raise ValueError("physical calibration inputs must be positive")
    weight = mass_kg * gravity_m_s2
    per_motor = weight / 4.0
    raw_pwm = 65535.0 * per_motor / maximum_thrust_per_motor_n
    normalized = (raw_pwm - 10001.0) / (60000.0 - 10001.0)
    if not 0.0 <= normalized <= 1.0:
        raise ValueError("weight-balance thrust is outside the command range")
    return HoverThrustEstimate(
        mass_kg=mass_kg,
        gravity_m_s2=gravity_m_s2,
        maximum_thrust_per_motor_n=maximum_thrust_per_motor_n,
        weight_n=weight,
        thrust_per_motor_n=per_motor,
        raw_pwm=raw_pwm,
        normalized_command=normalized,
    )


CF2X_T350_HOVER = estimate_cf2x_t350_hover_thrust()
