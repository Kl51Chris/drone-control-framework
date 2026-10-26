# XY physical mapping validation

Both local horizontal attitude mappings are supported by deliberate bipolar
hardware excitation:

| mapping | run | slope | 95% bootstrap interval | sign |
|---|---|---:|---:|---:|
| pitch to body-X acceleration | `20261026T074303_254078Z` | -8.719 m/s²/rad | [-10.405, -6.972] | -1 |
| roll to body-Y acceleration | `20261026T075325_531723Z` | -9.265 m/s²/rad | [-10.148, -8.713] | -1 |

The roll safety abort was at elapsed 4.040054 s during `SETTLE`. Both pulses and
the final recovery completed at 2.500054 s, 1.540 s earlier. The pitch abort was
at 5.480054 s, 2.980 s after its sequence completed at 2.500083 s. Each analysis
selected only its 65 neutral/pulse/recovery rows, and its centered 0.30 s local
velocity regressions were calculated within that selected set. Neither later
safety event affected an identification regression window.

The existing controller mapping is correct:

```text
pitch_reference = -atan2(desired_body_x_acceleration, g)
roll_reference  = -atan2(desired_body_y_acceleration, g)
```

The hardware mapping interlock is explicitly validated in
`stage5d_xy_pd_hover_candidate_config.json`. Remaining uncertainty concerns
closed-loop gain performance, coupling, estimator behavior, and the short local
nature of the tests—not the measured polarity. All original safety limits and
per-run authorization remain required.
