# Stage 5B saturation and command-interface review

## Result

**Conditional GO to implement and review a restrained powered-attitude test;
NO-GO to execute it or attempt free flight yet.** Offline restoring polarity and
the command interface are consistent, and saturation is confined to short
takeoff transients in the official-controller recording. The fixed-heading yaw
objective produced one 0.66 s saturated command run and must be treated as a
different objective from the official baseline's zero-yaw-rate request.

No gains or limits were changed by this review.

## Saturation breakdown

`LANDING` below groups raw `LANDING` and `LANDING_CONFIRM` rows.

| axis | TAKEOFF | HOVER | LANDING |
|---|---:|---:|---:|
| roll | 3 | 0 | 0 |
| pitch | 14 | 0 | 0 |
| yaw | 33 | 0 | 0 |

Contiguous events, using `last timestamp - first timestamp + final dt` as the
estimated command duration:

| axis | first / last sample (s) | samples | estimated duration (s) | sign |
|---|---:|---:|---:|---|
| roll | 0.534565 / 0.574657 | 3 | 0.060202 | negative |
| pitch | 0.534565 / 0.614545 | 5 | 0.099918 | positive |
| pitch | 0.794547 / 0.954530 | 9 | 0.180008 | negative |
| yaw | 0.594606 / 1.234507 | 33 | 0.659903 | negative |

Machine-readable timestamps, estimated end times, raw/grouped phases, maximum
errors, and signs are in `saturation_interface_report.json`.

## Yaw interpretation

Replay captured the first sample's heading, -0.001519 rad at elapsed 0.254450 s,
and held it as an absolute reference. With yaw Kp=2.0 s^-1 and the 0.35 rad/s
limit, saturation begins at an absolute heading error of 0.175 rad (10.0 degrees).
The recorded wrapped error reached 0.572512 rad. All 33 saturated rows are one
negative-command run during TAKEOFF.

The controller wraps heading error with `atan2(sin(error), cos(error))`. Every
replayed error stayed in [-pi, pi], and measured yaw crossed no wrap boundary;
therefore wrapping neither caused nor split this event. The recording instead
contains a real departure from the captured initial heading as represented by
the estimator.

The official hover experiment sent `yawrate=0`. That requests zero yaw rate; it
does not impose this custom controller's captured absolute heading objective.
The official flight's yaw motion under a zero-rate request is useful disturbance
data, but replaying an absolute-heading controller against it will naturally
create commands the official controller never issued. The 33 events cannot be
used to claim that fixed-heading control is unstable—or that it is validated.

Initial heading for a future controller must be captured from fresh, settled
pre-arm/pre-command state, recorded in metadata, and held continuously across
the test. Resetting the heading reference during a run would hide drift and
invalidate the result.

## Restoring-direction review

Across the replay, firmware-side command direction opposed the measured
deviation from level/held heading for every nonzero-error sample:

| axis | restoring / evaluated samples |
|---|---:|
| roll | 394 / 394 |
| pitch | 394 / 394 |
| yaw | 393 / 393 |

Roll and yaw packet signs equal their API command signs. Installed cflib packs
the manual pitch field as the negative of the API pitch argument. The controller's
`pitch_command_sign=-1` maps the legacy measured-pitch convention into the API
command convention; cflib then performs its documented packet-coordinate
conversion. The resulting packet-side pitch rate has the same sign as the
desired-minus-measured legacy pitch error and is restoring under the Stage 3
convention.

There is no second pitch inversion inside `CrazyflieCommandAdapter`: it only
converts rad/s to deg/s. Thus there is no duplicated inversion in that adapter.
There is, however, split sign ownership: the controller contains an explicit
legacy pitch sign while cflib performs the packet inversion, even though the
adapter documentation says backend sign conventions belong in the adapter.
This is an integration hazard and must be captured in regression tests; it must
not be "cleaned up" without preserving the verified net polarity.

## Exact command interface

The controller constructs `BodyRateThrustCommand` in this order:
`roll_rate`, `pitch_rate`, `yaw_rate`, `thrust`. Rates are finite rad/s and thrust
is normalized [0,1]. The adapter preserves ordering, converts rates to deg/s,
converts thrust to percent, and calls:

```
send_setpoint_manual(roll, pitch, yawrate, thrust_percent, rate=True)
```

Installed cflib 0.1.34 packs:

```
TYPE_MANUAL, roll, -pitch, yawrate, uint16(10001 + normalized*49999), rate
```

The new regression test exercises the actual controller, existing adapter, and
installed cflib packet encoder together, including field ordering, units, pitch
packing, thrust encoding, and rate mode.

## Gate for the next step

Implementation of a dedicated restrained-test runner may proceed without gain
or limit changes. Execution remains blocked until the runner has been reviewed
for explicit per-run authorization, fresh settled heading capture, single-axis
selection, command-before-send logging, supervisor checks, short time bounds,
hard-stop behavior, and zero automatic repetition. The first powered tests must
evaluate roll and pitch polarity separately; fixed-heading yaw must be a distinct
test and must not be compared as if it were the official zero-rate objective.
