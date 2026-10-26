# Phase 1 read-only inspection

The active implementation was inspected before this package was added. No file
under `velocity_control_framework/`, `experiments/`, `tools/`, or `legacy/` was
changed.

## Command boundary

- `BodyRateThrustCommand` uses body roll/pitch/yaw rates in rad/s and normalized
  collective thrust in `[0, 1]`.
- `CrazyflieCommandAdapter` converts rates to deg/s, clamps thrust to `[0, 1]`,
  converts it to `[0, 100]`, and calls
  `send_setpoint_manual(roll, pitch, yaw, thrust, rate=True)`.
- Installed cflib then negates pitch while packing the generic manual packet and
  maps thrust percentage to uint16 range 10001--60000. Thus a true motor stop is
  the separate stop-setpoint, not a manual setpoint containing 0 percent.
- These commands retain the Crazyflie firmware body-rate loops and motor mixer;
  they are not individual motor or world-force commands.

## State boundary and coordinates

- `CrazyflieStateProvider` logs `stateEstimate.{x,y,z,vx,vy,vz}` in world-frame
  metres and m/s. World `z` is treated as positive upward.
- Roll, pitch, and yaw arrive in degrees and are converted to radians. The
  Crazyflie legacy pitch sign is passed through without another inversion.
- Body rates come from `stateEstimateZ.rate*` in milliradians/s and are converted
  to rad/s. A complete state's timestamp is host monotonic time when assembled.
- The provider splits translation, attitude, and rate values across three 10 ms
  log blocks. A complete state can therefore mix the most recently received
  values; it is not an atomic firmware snapshot.

## Altitude PID

- Error is `z_reference - z`; positive error produces a positive correction for
  positive gain. Thrust is `equilibrium_thrust + correction`.
- PID integration is rectangular. The first derivative is zero, then derivative
  is `(error[k] - error[k-1]) / dt`. There is no derivative filter or setpoint
  weighting.
- Each PID output is clamped. Conditional integration accepts a candidate only
  when unsaturated or when error would drive the output back toward its limits.
  Final thrust is separately clamped to configured normalized limits.

## Existing safety behavior

- The generic runner stops commands, stops state acquisition, and disarms in a
  `finally` path. Staged validation additionally requires explicit physical-test
  authorization and applies stage-specific command/state limits.
- Hardware tools repeatedly send stop setpoints during shutdown. Supervisor
  arming/disarming uses `send_arming_request(True/False)`.
- Repository documentation records no durable evidence that stable hover or the
  staged flight sequence has been physically validated. Defaults in this new
  package are therefore examples, not flight-validated values.
