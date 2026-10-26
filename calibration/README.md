# Independent altitude PID calibration

This package is independent of `UniversalRunner` and the staged experiments. It
reuses the verified Crazyflie command and state adapters, but owns its own
configuration, PID, state machine, telemetry, analysis, and shutdown lifecycle.

## Offline use

```bash
pytest calibration/tests -v
python -m calibration analyze calibration/runs/<run>
python -m calibration analyze-equilibrium calibration/runs/<official-hover-run>
python -m calibration export calibration/runs/<run> calibration/validated.json
python -m calibration verify-control calibration/runs/<state-monitor-run> --config calibration/default_config.json
python -m calibration replay-pid calibration/runs/<official-hover-run> calibration/stage5_replay \
  --equilibrium-thrust 0.425 --kp 0.10 --ki 0 --kd 0 \
  --correction-min -0.045 --correction-max 0.025 --min-thrust 0.38 --max-thrust 0.45
python -m calibration replay-cascaded calibration/runs/<official-hover-run> \
  calibration/stage5_analysis/cascaded_stage5b \
  --controller-config calibration/stage5b_offline_config.json
```

Each run contains `metadata.json` and `telemetry.csv`. Analysis adds
`metrics.json` and a dependency-free `response.svg`. Export refuses incomplete or invalid runs.
An export maps every altitude setting to the corresponding
`PIDConfig`/`OuterLoopPIDConfig` field; it does not modify that controller.
Exports intentionally retain `hardware_validated: false`: software cannot infer
physical safety or validation from a completed log. A human review must make and
record that determination outside this automatic export.

`replay-pid` writes every P/I/D term, predicted command, and PID/thrust-clamp
saturation flag. Recorded measurements are exogenous: this is controller
arithmetic replay, not a closed-loop plant simulation, and cannot establish
stability or flight safety.

`CascadedPIDController` is the Stage 5 unified controller. Its output is exactly
`BodyRateThrustCommand`, leaving body-rate PID and motor mixing onboard. The
checked-in Stage 5B configuration enables altitude P and level attitude/heading
feedback only; altitude velocity/integral and horizontal position/velocity
stages are configuration-gated for 5C and 5D. Both hardware authorization flags
in that configuration are false, so it is an offline artifact, not a flight file.

The dedicated first-flight path is `cascaded-5b`; it never calls the official
hover API. It requires the standard explicit confirmation and separate flight
and controller configurations. Analyze either a completed or aborted artifact
without reconnecting to hardware:

```bash
python -m calibration run cascaded-5b \
  --config calibration/stage5b_first_flight_config.json \
  --controller-config calibration/stage5b_offline_config.json \
  --confirm-flight I_UNDERSTAND_PROPELLERS_MAY_SPIN
python -m calibration analyze-cascaded-flight calibration/runs/<cascaded-5b-run>
```

The prepared Stage 5D XY mode is `cascaded-xy`. Deliberate bipolar hardware
identification supports negative pitch/body-X and negative roll/body-Y mappings;
the combined altitude-PD/XY-PD candidate records that reviewed mapping. The
earlier first-flight candidate remains interlocked for provenance. Offline replay
remains available:

```bash
python -m calibration replay-cascaded-flight \
  calibration/runs/<cascaded-5b-run> calibration/stage5_analysis/xy-replay \
  --controller-config calibration/stage5d_xy_pd_hover_candidate_config.json
```

The remaining body-X/pitch uncertainty has a dedicated, XY-disabled physical
identification mode. It retains the Stage 5B controller and all flight safety
limits, waits for a bounded hover condition, then applies fixed ±0.020 rad pitch
reference pulses separated by neutral recovery periods. See
`STAGE5D_PITCH_IDENTIFICATION.md`. Each flight still requires the confirmation
literal; preparation never runs it automatically. Analyze its artifact with:

```bash
python -m calibration analyze-pitch-response calibration/runs/<pitch-response-id-run>
```

## Hardware procedure

1. Remove propellers. Secure the vehicle, inspect frame/motors/deck, charge the
   battery, clear the area, verify the radio URI, and keep a physical means to
   disconnect power.
2. Run the non-flight connectivity/state/supervisor check:

   ```bash
   python -m calibration run preflight --config calibration/default_config.json
   ```

   Then record the onboard estimator for 20 seconds at approximately 50 Hz:

   ```bash
   python -m calibration run state-monitor --config calibration/default_config.json
   ```

   State monitor never arms or sends a flight setpoint. It always uses onboard
   `stateEstimate.z`/`vz`, even if the configuration includes an external UDP
   port. Ctrl-C safely finishes the partial artifact. The run produces
   `state_telemetry.csv`, `metrics.json`, and `state_response.svg`.

   For the guided 30-second physical attitude-sign check, keep the propellers
   removed and follow the six synchronized five-second terminal prompts:

   ```bash
   python -m calibration run attitude-sign-monitor --config calibration/default_config.json
   ```

   The guided monitor is also read-only: it never arms or transmits a setpoint.
   Every telemetry row contains its phase marker, physical action, expected axis,
   and expected sign. It writes `attitude_sign_summary.json` automatically.

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

Available modes are `preflight`, `state-monitor`, `attitude-sign-monitor`,
`hover-baseline`, `p-only`, `pid`, and
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

## Official firmware hover baseline

`official-hover-baseline` is separate from the custom/open-loop
`hover-baseline`. It sends the official generic hover setpoint directly at
50 Hz with body-frame `vx=0`, `vy=0`, `yawrate=0`. Firmware interprets
`zDistance` as an absolute world-frame Z setpoint, so the toolkit captures the
stationary preflight median Z and adds the configured height-above-ground
profile. The sequence ramps from 0.05 m to 0.30 m at 0.15 m/s, hovers for
3 seconds, and ramps down at 0.12 m/s to 0.03 m. It sends the stop packet only
after measured relative height is at most 0.06 m and |vz| is at most 0.08 m/s
continuously for 0.5 s. Failure to confirm this within 3 s is a hard abort and
is not reported as a successful landing.

The direct `send_hover_setpoint()` interface was selected instead of
`MotionCommander`: it uses the same official firmware hover controller but
keeps takeoff, telemetry, safety checks, landing, and fault classification in
one explicit testable state machine. Installed cflib 0.1.34 provides both APIs.
At runtime the mode requires protocol version 12 or newer, both to avoid the
legacy hover packet and because this safety state machine depends on the
supervisor interface.

Run only after reviewing this checklist:

- suitable open test area, charged battery, undamaged and correctly installed
  propellers, secure Flow Deck, and a second person/observer when possible;
- correct radio URI and no other client or high-level commander connected;
- flat, textured, well-lit surface within Flow Deck operating conditions;
- vehicle level and stationary, with the battery connector immediately
  reachable;
- configured 0.30 m target, 0.15/0.12 m/s profile, 1.5 m altitude ceiling,
  1.5 m/s vertical-speed limit, 0.50 m horizontal displacement, 0.75 m/s
  horizontal speed, 0.45 rad tilt limit, 150 ms state timeout, and
  100 ms loop deadline reviewed for the test location;
- understand that the command below arms and may spin the motors;
- understand that Ctrl-C sends stop setpoints and disarms, terminating lift
  immediately. It is not a controlled or safe landing.

Explicitly initiate exactly one flight with:

```bash
python -m calibration run official-hover-baseline \
  --config calibration/default_config.json \
  --confirm-flight I_UNDERSTAND_PROPELLERS_MAY_SPIN
```

Before arming, the mode verifies radio connectivity, modern protocol support,
supervisor armability (and `can_fly` confirmation after arming), Flow Deck detection (`deck.bcFlow`/`deck.bcFlow2`), fresh
and settled onboard state, level attitude, horizontal position/velocity against
the flight limits, configured limits, and availability
of the supervisor emergency-stop API. Normal completion always uses the
controlled landing profile. A recoverable software fault attempts that profile;
stale/unsafe state and failed recovery use immediate shutdown.

The stationary preflight component-wise median x/y is the horizontal origin.
Metadata records that origin, the latest and maximum preflight horizontal
velocity estimates, and preflight displacement. If a safety check aborts, a
`safety_event` snapshot is flushed to metadata before shutdown and identifies
the exact condition, measured components/magnitude, origin, and applicable
limit. A zero-row telemetry file therefore no longer hides the rejected sample.

Telemetry is stored in `flight_telemetry.csv`, with synchronized reference and
measured altitude, full position/velocity, attitude, supervisor state, phase,
timing, and optional `stabilizer.thrust`. That firmware value is logged as
`onboard_thrust_raw`: it is a raw collective controller value, not normalized
thrust, force in newtons, or a calibrated equilibrium-thrust result. Analysis
generates `metrics.json` and `flight_response.svg`; `physically_validated`
remains false because the software cannot certify flight performance itself.

## Interpretation and limitations

An analyzed result is marked valid only if the process completed and recorded a
hover phase with enough telemetry. “Valid” means structurally complete, not that
the gains are safe or performant. Hardware validation can only be claimed from
reviewed real measurements. The attitude-hold signs are configuration assumptions
that must be verified for the specific firmware/vehicle with props removed and a
restrained, low-risk test before free flight.
