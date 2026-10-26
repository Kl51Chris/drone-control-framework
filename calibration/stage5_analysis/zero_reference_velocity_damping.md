# Zero-reference vertical-velocity damping investigation

## Scope

The candidate keeps altitude Kp=0.10, Kv=0.04, equilibrium thrust=0.425,
integral disabled, correction limits -0.045/+0.025, absolute thrust limits
0.380/0.450, and every attitude, XY, and safety setting unchanged. Only the
velocity reference used by the existing altitude velocity term changes:

```text
current ramp-tracking PD: Kv * (vz_ref - measured_vz)
candidate zero-reference damping: Kv * (0 - measured_vz)
```

The separate candidate is
`stage5c_altitude_pd_zero_velocity_reference_candidate_config.json`. Existing
configurations were not overwritten.

## Same-state command replay

Commands were recomputed through `CascadedPIDController` on the recorded states.
The reconstructed historical P-only and current ramp-PD commands match their
logged commands exactly. Recorded states remain exogenous, so this comparison
does not predict the resulting trajectory.

### States from the P-only flight

| Phase | P-only thrust range / mean | Ramp-PD range / mean | Zero-reference PD range / mean | Zero minus ramp |
|---|---:|---:|---:|---:|
| TAKEOFF | 0.41652–0.42896 / 0.42476 | 0.40821–0.43117 / 0.42128 | 0.40421–0.42717 / 0.41728 | exactly -0.00400 |
| HOVER | 0.40428–0.42873 / 0.41623 | 0.40206–0.43311 / 0.41623 | 0.40206–0.43311 / 0.41623 | exactly 0 |
| LANDING | 0.41367–0.42008 / 0.41577 | 0.41105–0.42381 / 0.41646 | 0.41425–0.42701 / 0.41966 | exactly +0.00320 |
| SETTLE | 0.41718–0.42170 / 0.42018 | 0.40794–0.42541 / 0.41779 | 0.40794–0.42541 / 0.41779 | exactly 0 |
| LANDING_CONFIRM | 0.40856–0.42215 / 0.41403 | 0.40620–0.42663 / 0.41282 | 0.40620–0.42663 / 0.41282 | exactly 0 |

The fixed differences follow directly from Kv=0.04: removing +0.10 m/s
takeoff tracking subtracts 0.004 thrust, while removing -0.08 m/s landing
tracking adds 0.0032 thrust. HOVER and ground-hold phases already use zero
velocity reference, so the two PD laws are identical there.

### States from the physical PD flight

| Phase | Historical/current command range / mean | Zero-reference range / mean | Difference |
|---|---:|---:|---:|
| TAKEOFF | 0.40763–0.43053 / 0.42047 | 0.40363–0.42653 / 0.41647 | exactly -0.00400 |
| HOVER | 0.40164–0.42764 / 0.41097 | 0.40164–0.42764 / 0.41097 | exactly 0 |

This isolates an important limitation of the hypothesis: zero-reference damping
can reduce the initial takeoff command and may reduce the velocity carried into
HOVER, but it cannot change the feedback law after HOVER begins. The worse
three-second HOVER result in the recorded PD flight cannot be attributed to a
continuing nonzero velocity reference; it can only have been influenced through
the takeoff initial condition or ordinary run-to-run variation.

## Bounds and landing implications

No replayed command saturated. Across the P-only states, zero-reference PD used
0.40206–0.43311 outside LANDING and 0.41425–0.42701 during LANDING. Across the
physical PD states it used 0.40164–0.42764. These remain inside 0.380–0.450 with
substantial authority in both directions.

During LANDING, zero-reference damping treats every descent as a velocity error
and commands 0.0032 more thrust than ramp tracking. This may reduce excessive
descent and impact/rebound energy, but it also opposes the intended -0.08 m/s
descent and may delay touchdown. Position P still commands downward thrust when
above the reference, and the replay shows no saturation, but closed-loop landing
performance is unvalidated. SETTLE and LANDING_CONFIRM behavior is identical to
current PD because their reference velocity is already zero.

## Recommendation

**Reasonable as one focused diagnostic candidate, not demonstrated superior.**
It directly removes the +0.004 takeoff feedforward that coincided with earlier,
faster target crossing in the PD flight, while retaining the same damping law in
HOVER. Its principal risk is slower landing descent. A future explicitly
authorized run should be judged on target-crossing velocity, peak altitude,
common-window HOVER RMSE, landing duration, and touchdown confirmation. The
replay does not justify claiming improved closed-loop performance or changing
Kv, Kp, equilibrium thrust, bounds, or safety limits.
