# Independent altitude PID calibration

This package is independent of `UniversalRunner` and the staged experiments. It
reuses the verified Crazyflie command and state adapters, but owns its own
configuration, PID, state machine, telemetry, analysis, and shutdown lifecycle.

## Offline use

```bash
pytest calibration/tests -v
python -m calibration analyze calibration/runs/<run>
python -m calibration export calibration/runs/<run> calibration/validated.json
```

Each run contains `metadata.json` and `telemetry.csv`. Analysis adds
`metrics.json` and a dependency-free `response.svg`. Export refuses incomplete or invalid runs.
An export maps every altitude setting to the corresponding
`PIDConfig`/`OuterLoopPIDConfig` field; it does not modify that controller.
Exports intentionally retain `hardware_validated: false`: software cannot infer
physical safety or validation from a completed log. A human review must make and
record that determination outside this automatic export.

## Hardware procedure

1. Remove propellers. Secure the vehicle, inspect frame/motors/deck, charge the
   battery, clear the area, verify the radio URI, and keep a physical means to
   disconnect power.
2. Run the non-flight connectivity/state/supervisor check:

   ```bash
   python -m calibration run preflight --config calibration/default_config.json
   ```

3. Review the saved metadata, state source, signs, thrust bounds, attitude gains,
   altitude ceiling, and timeout values. Defaults are **not hardware validated**.
4. If using an external measurement, set `external_altitude_udp_port`. Send UDP
   JSON to loopback only, for example `{"altitude_m": 0.42}`. The receive time is
   the measurement timestamp and velocity is a finite difference. Confirm this
   stream with props removed; loss or staleness aborts flight.
5. Install propellers only in an appropriate flight area with an observer and an
   immediately reachable emergency action. Start exactly one selected trial:

   ```bash
   python -m calibration run p-only --config my_config.json \
     --confirm-flight I_UNDERSTAND_PROPELLERS_MAY_SPIN
   ```

Available modes are `preflight`, `hover-baseline`, `p-only`, `pid`, and
`takeoff-hover-land`. The explicit confirmation is required for every flight
process. There is no automatic repeat or gain search. Press Ctrl-C to trigger the
same stop-setpoint and disarm cleanup used for faults.

All flight modes use takeoff, hover, landing, and settle phases with rate-limited
altitude references. `hover-baseline` sends equilibrium thrust with attitude hold
and logs the reference/response without altitude feedback; because it cannot
guarantee reference tracking, perform it only under a test arrangement reviewed
for that open-loop risk. `p-only` forcibly disables I and D. `pid` tests the
configured PID. `takeoff-hover-land` is the final full-profile validation.
Every mode is one short trial, not an unattended calibration routine.

## Interpretation and limitations

An analyzed result is marked valid only if the process completed and recorded a
hover phase with enough telemetry. “Valid” means structurally complete, not that
the gains are safe or performant. Hardware validation can only be claimed from
reviewed real measurements. The attitude-hold signs are configuration assumptions
that must be verified for the specific firmware/vehicle with props removed and a
restrained, low-risk test before free flight.
