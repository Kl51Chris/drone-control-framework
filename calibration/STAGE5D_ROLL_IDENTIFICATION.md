# Stage 5D roll/body-Y polarity identification

## Evidence review

No existing run provides a controlled roll-polarity identification. During the
65 active samples of pitch run `20261026T074303_254078Z_pitch-response-id`, an
exploratory application of the same noise-aware regression gives roll versus
body-Y acceleration correlation -0.779, slope -13.263 m/s²/rad, and moving-block
bootstrap interval [-17.752, -10.347]. However, roll occupied only
[-0.00736, +0.00312] rad and was neither deliberately commanded nor bipolar in
a controlled sequence. It is coupled observational motion and is insufficient
to validate `body_y_acceleration_to_roll_sign`.

## Minimal roll experiment

`roll-response-id` reuses the pitch experiment's controller, flight runner,
fixed timeline, envelope-margin checks, estimator checks, logger, shutdown
paths, and ±0.020 rad conservative excitation bound. It changes only the
excited attitude axis. XY feedback remains disabled. The controller file keeps
altitude Kp=0.06, Kv=0.06, equilibrium thrust 0.425, and zero-reference velocity
damping.

| phase | time (s) | roll reference |
|---|---:|---:|
| takeoff | 0.00–1.00 | 0 |
| neutral safety assessment | 1.00–1.20 | 0 |
| neutral baseline | 1.20–1.40 | 0 |
| positive roll | 1.40–1.65 | +0.020 rad |
| neutral recovery | 1.65–1.90 | 0 |
| negative roll | 1.90–2.15 | -0.020 rad |
| neutral recovery | 2.15–2.50 | 0 |
| controlled landing | from 2.50 | 0 |

Exact supervised flight command (not executed):

```bash
python3 -m calibration run roll-response-id \
  --config calibration/pitch_identification_flight_config.json \
  --controller-config calibration/stage5c_altitude_pd_pole_candidate_config.json \
  --roll-excitation-rad 0.020 \
  --confirm-flight I_UNDERSTAND_PROPELLERS_MAY_SPIN
```

Analyze the resulting run without hardware access:

```bash
python3 -m calibration analyze-roll-response \
  calibration/runs/<roll-response-id-run>
```

A usable sign requires both pulses and recoveries, command-to-roll changes of at
least ±0.005 rad with the commanded signs, a body-Y acceleration bootstrap
interval excluding zero, no more than 5% roll-rate saturation, and no safety
event. Otherwise the result is `INCONCLUSIVE`; no mapping or interlock changes
are automatic.

## Prepared full XY/Z hover candidate

`stage5d_xy_pd_hover_candidate_config.json` combines the unchanged altitude PD
(Kp=0.06, Kv=0.06), existing bounded XY PD, identified body-X/pitch sign -1,
and the experimentally supported body-Y/roll sign -1. Review of roll run
`20261026T075325_531723Z_roll-response-id` found both pulses and recoveries
complete at 2.500054 s; its horizontal-envelope abort occurred later, at
4.040054 s during `SETTLE`. The 65-sample identification regression and its
centered windows therefore precede the event. Its -9.265 m/s²/rad slope and
[-10.148, -8.713] bootstrap interval support the configured sign.

The candidate now records `horizontal_mapping_hardware_validated=true` for both
reviewed physical polarities. `hardware_flight_authorized` remains false in the
persistent configuration; the exact one-run command still requires the explicit
confirmation literal:

```bash
python3 -m calibration run cascaded-xy \
  --config calibration/stage5b_first_flight_config.json \
  --controller-config calibration/stage5d_xy_pd_hover_candidate_config.json \
  --confirm-flight I_UNDERSTAND_PROPELLERS_MAY_SPIN
```

Every existing hard safety threshold remains in force. The identification
evidence validates local polarity, not XY gain performance, cross-axis coupling,
large-angle dynamics, or successful hover. Exactly one supervised run should be
reviewed before any repeat or parameter change.
