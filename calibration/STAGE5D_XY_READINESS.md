# Stage 5D first XY-containment readiness review

## Decision

**GO for one explicitly authorized, supervised XY/Z containment flight.**
Bipolar physical tests resolved both local mappings: body-X/pitch is negative
and body-Y/roll is negative. Both safety events occurred after their complete
identification sequences and regression windows. The combined PD candidate
therefore records `horizontal_mapping_hardware_validated=true`; this validates
polarity, not closed-loop XY performance.

No altitude, attitude, thrust, body-rate, tilt, displacement, or velocity limit
was changed. No hardware experiment was run.

## Existing controller equations

The implementation remains `CascadedPIDController`; no second controller was
created. With zero XY reference velocity:

```
e_p_world = p_reference - p
e_v_world = v_reference - v
a_world_raw = Kp_xy e_p_world + Kd_xy e_v_world
a_world = magnitude_limit(a_world_raw)
a_body = Rz(-yaw) a_world
pitch_reference = sx atan2(a_body_x, g)
roll_reference  = sy atan2(a_body_y, g)
```

`Rz(-yaw)` expands to `(cos(yaw) ax + sin(yaw) ay,
-sin(yaw) ax + cos(yaw) ay)`, which is the correct passive world-to-body planar
rotation. Positive gains make each position and velocity contribution point in
the direction of its corresponding desired-minus-measured error. Unit tests
exercise nonzero yaw and all four restoring terms.

The attitude references then enter the unchanged Stage 5B attitude P loops and
produce the same `BodyRateThrustCommand` interface. XY has no integral term.

## Candidate configuration

`stage5d_xy_first_flight_config.json` preserves:

- altitude P 0.10; altitude velocity and integral feedback disabled;
- equilibrium/min/max thrust 0.425/0.380/0.450;
- roll/pitch/yaw attitude gains 4.0/4.0/2.0;
- roll/pitch/yaw rate limits 0.35 rad/s;
- flight safety tilt 0.45 rad, displacement 0.50 m, horizontal speed 0.75 m/s,
  altitude 1.5 m, and vertical speed 1.5 m/s.

The conservative XY PD candidate is Kp=0.4 s^-2 and Kd=0.8 s^-1, with
acceleration magnitude limited to 0.4 m/s² and desired tilt magnitude limited to
0.08 rad. Both persistent authorization flags remain false. These are candidate
values for sign/range review, not validated flight gains.

## First-flight replay

All 207 samples from `20261026T053025_672531Z_cascaded-5b` were replayed with the
captured XY origin, zero desired XY velocity, recorded altitude reference, and
captured heading. Recorded states were exogenous; no containment is simulated.

| quantity | maximum absolute value |
|---|---:|
| world X acceleration | 0.23149 m/s² |
| world Y acceleration | 0.30192 m/s² |
| desired roll | 0.03152 rad |
| desired pitch | 0.02376 rad |
| roll-rate command | 0.13872 rad/s |
| pitch-rate command | 0.25539 rad/s |

Position and velocity contributions were restoring on 100% of replay samples.
Acceleration-vector, tilt-reference, roll-rate, and pitch-rate saturation counts
were all zero. `xy_replay.csv` logs position/velocity errors, their individual
acceleration contributions, raw and limited world accelerations, body-frame
accelerations, desired attitudes, raw/limited body-rate commands, and every
saturation flag.

## Identified pitch/body-X polarity

Run `20261026T074303_254078Z_pitch-response-id` completed both pitch pulses and
both recovery windows before its later landing-confirmation safety abort. It
measured command-to-attitude correlation +0.904 and pitch-to-body-X-acceleration
correlation -0.889. The fitted acceleration slope was -8.719 m/s²/rad, with a
95% moving-block-bootstrap interval of [-10.405, -6.972], which excludes zero.

Accordingly, positive desired body-X acceleration requires negative pitch
reference: `sx=-1`. This is independent of the existing
`pitch_command_sign=-1`, which compensates the cflib command-packet convention
inside the attitude loop and is supported by the positive command-to-attitude
correlation. Changing both signs would duplicate the inversion.

The candidate configuration records `sx=-1`.

## Identified roll/body-Y polarity

Run `20261026T075325_531723Z_roll-response-id` completed both roll pulses and
recoveries at 2.500054 s. Its envelope abort occurred at 4.040054 s during
`SETTLE`, 1.540 s later, and did not enter the 65-sample regression. Measured
command-to-roll correlation was +0.916. Roll-to-body-Y acceleration correlation
was -0.983 with slope -9.265 m/s²/rad and 95% bootstrap interval
[-10.148, -8.713]. Thus positive desired body-Y acceleration requires negative
roll reference: `sy=-1`, matching the existing configuration.

## Prepared flight command and safety gates

The prepared one-run command is:

```bash
python3 -m calibration run cascaded-xy \
  --config calibration/stage5b_first_flight_config.json \
  --controller-config calibration/stage5d_xy_pd_hover_candidate_config.json \
  --confirm-flight I_UNDERSTAND_PROPELLERS_MAY_SPIN
```

It retains explicit per-run confirmation, protocol/Flow Deck/emergency-stop checks,
settled fresh-state capture, supervisor checks, loop deadline, full state and
contribution logging, existing flight envelope, hard supervisor stop, stop
setpoints, disarm, and touchdown confirmation. There is no repeat or gain search.

## Acceptance criteria for the first containment flight

All criteria are measurable and must be reviewed after exactly one run:

- no hard-safety event and normal touchdown confirmation;
- complete takeoff and at least 3.0 s HOVER logging;
- maximum horizontal displacement <=0.30 m and final displacement <=0.20 m;
- HOVER mean horizontal speed <=0.10 m/s and final speed <=0.10 m/s;
- no sustained outward displacement trend during the final 1.0 s of HOVER;
- maximum tilt <=0.15 rad;
- acceleration/tilt/body-rate saturation on <=5% of samples per channel;
- state age, loop deadline, thrust, body-rate, altitude, and velocity limits all
  respected;
- measured initial response on both axes has the physically restoring sign.

Failure is diagnostic evidence, not permission to raise limits or repeat
automatically. Altitude metrics should still be reported, but the Stage 5B
altitude and attitude settings must remain unchanged for this containment test.
