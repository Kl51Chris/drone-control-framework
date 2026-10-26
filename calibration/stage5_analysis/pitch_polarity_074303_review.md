# Pitch-polarity run 074303 review

Run: `20261026T074303_254078Z_pitch-response-id`

## Timing and data validity

The safety event occurred at experiment elapsed time **5.480054 s**, about one
control interval (0.019999 s) after the final telemetry row. It was a horizontal
envelope violation during `LANDING_CONFIRM`, at 0.500412 m displacement and
0.045175 m/s horizontal speed.

| interval | elapsed time (s) |
|---|---:|
| positive pitch | 1.400-1.650 |
| positive recovery | 1.650-1.900 |
| negative pitch | 1.900-2.150 |
| negative recovery | 2.150-2.500 |
| identification complete / landing starts | 2.500083 |
| safety abort | 5.480054 |

Thus the abort was 3.330 s after the negative pulse and 2.980 s after the full
identification sequence. All 65 identification samples preceded it. Even the
centered 0.30 s velocity-regression window around the final identification
samples ended long before the abort. The safety event made the automatic final
status `INCONCLUSIVE`; it did not temporally contaminate the fitted polarity.

## Mapping audit

The controller first computes restoring world acceleration and rotates it into
body coordinates:

`a_body = Rz(-yaw) a_world`

The experiment measured `a_body_x / pitch = -8.719 m/s²/rad` (correlation
-0.889; bootstrap 95% interval [-10.405, -6.972]). Therefore the required map is

`pitch_reference = -atan2(a_body_x, g)`.

The prior XY candidate used the opposite sign, `+atan2(a_body_x, g)`. Only
`body_x_acceleration_to_pitch_sign` must change from +1 to -1. The attitude-loop
`pitch_command_sign=-1` must remain unchanged: it handles the cflib packet
convention, and the measured command-to-attitude correlation of +0.904 confirms
that command and achieved pitch reference have the same sign.

## Verdict

The bipolar response, strong correlations, and bootstrap interval excluding
zero sufficiently support the negative pitch/body-X physical mapping. This is
a **GO for adopting `sx=-1` in the offline XY candidate and continuing the
review toward supervised XY validation**, but a **NO-GO for powered XY flight
today**: `horizontal_mapping_hardware_validated` remains false pending review of
the roll/body-Y direction and the later horizontal-envelope abort.
