# CrazySim integration

CrazySim is an external, read-only dependency installed at
`/home/chris/CrazySim`. Project-specific integration remains in this
repository. The default single-agent CFLib endpoint is
`udp://127.0.0.1:19850`.

## Backend selection

Production flight assembly requires an explicit backend:

```bash
python -m experiments.run_flight --backend hardware
python -m experiments.run_flight --backend crazysim
```

Both commands are inspection-only unless `--execute` is supplied. Hardware
accepts only `radio://` URIs and CrazySim accepts only `udp://` URIs. Transport
mismatches fail without discovery or fallback. The hardware default remains
`radio://0/80/2M`; the CrazySim default is `udp://127.0.0.1:19850`.

## Telemetry

`CrazyflieStateProvider` is shared by both backends. It requests firmware
`stateEstimate` position, velocity, and attitude plus compressed
`stateEstimateZ` angular rates at 10 ms periods. Controller state is therefore
the Crazyflie firmware estimate, not direct MuJoCo ground truth. In CrazySim's
default configuration, MuJoCo pose is supplied to the firmware as an external
pose measurement before the firmware publishes `stateEstimate`.

Passive validation, which sends no commander or arming packets, is available as:

```bash
.venv/bin/python -m tools.validate_crazysim_telemetry \
  --uri udp://127.0.0.1:19850 --duration 10
```

## Legacy commander compatibility

The bundled CrazySim firmware decodes legacy RPYT commands but does not decode
CFLib generic commander `TYPE_MANUAL = 11`. `CrazySimCommandAdapter` therefore:

1. sets `flightmode.stabModeRoll`, `flightmode.stabModePitch`, and
   `flightmode.stabModeYaw` to `0` (rate mode),
2. reads all three values back and fails closed if they are not confirmed,
3. immediately after confirmed supervisor arming, sends one legacy
   zero-raw-thrust RPYT packet to unlock the firmware thrust path,
4. sends normal body-rate commands through CFLib `send_setpoint()`, and
5. uses the generic stop packet for normal shutdown.

Angular rates are converted from rad/s to deg/s. Normalized thrust preserves
the existing CFLib manual mapping:

```text
raw = int(10001 + normalized * (60000 - 10001))
```

The unlock packet, ordinary command, generic stop packet, and supervisor
emergency stop are intentionally separate operations.

## Remaining validation

Before any takeoff attempt:

- run the passive telemetry diagnostic and record rate and maximum gap;
- validate position, attitude, and angular-rate axes, especially pitch rate;
- inspect the live firmware setpoint/motor response at a safe non-flying level;
- verify supervisor arming, stop, disarm, and emergency-stop behavior;
- establish a CrazySim-specific controller configuration and hover thrust;
- confirm MuJoCo real-time factor remains sufficiently close to one.

Hardware PID gains and thrust calibration are not assumed valid in simulation.
No MuJoCo ground-truth logger is implemented yet.

## Verified Phase 2A results

Live validation of the shared `CrazyflieStateProvider` against the default
CrazySim endpoint produced:

```text
duration                    11.006 s
updates                     1143
effective update frequency  103.85 Hz
maximum receive gap         12.85 ms
```

The production CLI dry-run also resolved `backend=crazysim` and
`udp://127.0.0.1:19850`, remained in `inspection_only` status, and made no
backend connection.

## Supervisor and commander diagnostics

### CRTP-v7 supervisor compatibility

CrazySim's bundled firmware reports CRTP protocol version 7. CFLib's
`Supervisor` state API requires version 12: for every older version its
`_fetch_bitfield()` returns the literal value zero without sending a state
request. Consequently, a CFLib `read_bitfield()` result of zero is not a
firmware response and must not be interpreted as failed pre-flight checks.

The firmware still publishes its real 16-bit status as `supervisor.info`, with
bits 0 through 7 matching the documented can-arm, armed, auto-arm, can-fly,
flying, tumbled, locked, and crashed layout. `CrazySimSupervisorAdapter` logs
that value at 100 Hz, rejects stale telemetry, and uses it for all safety
checks. Bits introduced with the v12 supervisor port are not present in this
firmware; the CrazySim diagnostics do not use the high-level commander.

CFLib's v7 arming fallback sends platform command `0x01`. In CrazySim's SITL
build, `platformCommandProcess()` returns immediately under
`CONFIG_PLATFORM_SITL`, so that packet cannot arm or disarm the firmware. The
compatibility adapter can use the firmware's deprecated `system.arm` parameter
only for a genuinely disarmed lifecycle. It writes `1` only after a fresh
`supervisor.info` sample says the firmware can be armed and has no tumble,
lock, or crash fault. Parameter readback and subsequent `is_armed` plus
`can_fly` telemetry confirmation remain mandatory.

The default CrazySim build omits `CONFIG_MOTORS_REQUIRE_ARMING`, which makes
firmware `AUTO_ARMING=1`. It reaches the normal `0x000E` state (armed,
auto-armed, can-fly) before CFLib connects. Writing `system.arm=0` cannot clear
the separate internal auto-arm request, and the SITL platform disarm command is
compiled out. The project therefore never claims to disarm this lifecycle:
cleanup sends STOP and verifies firmware thrust, controller targets, and all
four motor outputs are zero. Explicitly armed non-auto lifecycles still receive
the matching parameter disarm. This preserves the firmware state machine
rather than bypassing it.

The default diagnostic is passive. It reads the firmware `supervisor.info` bitfield,
roll/pitch/yaw mode parameters, controller targets, current firmware control
thrust, and `motor.m1..m4`. It does not configure modes, arm, unlock the legacy
commander, send a setpoint, or send STOP:

```bash
.venv/bin/python -m tools.diagnose_crazysim_backend
```

The report prints `supervisor.info` in decimal and hexadecimal, every decoded
flag, and the age/freshness limit of the firmware log sample. An armed or flying
firmware is still reportable in this read-only mode. If either opt-in mutation
is requested, `is_armed` and `is_flying` are checked separately; the diagnostic
prints the complete report and the exact triggering flag, performs no write or
STOP operation, and exits with a refusal.

The diagnostic is restricted to exactly `udp://127.0.0.1:19850` and refuses to
continue if the firmware reports that it is armed or flying.

Writing and reading back all three rate modes is a separate opt-in operation.
It does not arm or send the unlock packet:

```bash
.venv/bin/python -m tools.diagnose_crazysim_backend \
  --configure-rate-modes
```

Sending one generic STOP packet is another explicit opt-in and requires the
rate-mode opt-in:

```bash
.venv/bin/python -m tools.diagnose_crazysim_backend \
  --configure-rate-modes \
  --verify-stop
```

STOP packet transmission only proves that CFLib accepted and transported the
command. Confirming zero simulated motor output requires a subsequent runtime
observation. This diagnostic implements no arming or nonzero-thrust option.

## Initial CrazySim controller configuration

The CrazySim PID default is separately named:

```text
crazysim-cf2x-t350-initial
```

It is deliberately uncalibrated and non-flying: feedback gains, equilibrium
thrust, and thrust limits are zero. It makes backend/configuration selection
explicit without guessing hover thrust or claiming that physical-airframe
gains apply to the MuJoCo model. Selecting `--backend crazysim` chooses this
configuration unless another configuration is explicitly requested.

Before hover, remaining command validation includes observing firmware target
and motor logs for positive and negative axis commands, validating the pitch
sign end to end, distinguishing raw thrust 0 from the normal-command minimum
of 10001, verifying STOP and emergency-stop motor effects, and establishing a
reviewed CrazySim-specific equilibrium thrust and safety envelope.

## Repeated connection lifecycle

CrazySim has no CFLib disconnect message. Its UDP bridge stores the source
address of the most recent client packet in `_cflib_addr` and never clears it.
A packet from a later client replaces that address, so consecutive clients are
intended to work without restarting MuJoCo or SITL.

Two cleanup details can make the behavior look otherwise:

- CFLib's normal `Crazyflie.close_link()` sends a legacy raw-zero RPYT packet
  before closing its UDP socket. Passive diagnostics now use a command-free
  `PassiveCrazyflie` close path so they do not change the thrust-lock state.
- The original `CrazyflieStateProvider.stop()` stopped firmware log blocks but
  did not delete them. CrazySim paths now delete their blocks during shutdown;
  hardware behavior is unchanged. CFLib also sends a logging reset while
  refreshing the TOC on a new connection, so stale blocks alone do not explain
  a failure that reaches normal connection setup.

CrazySim also retains bounded firmware/client forwarding queues after the first
client closes. The reconnect diagnostic allows 250 ms for late replies to
drain to the old UDP address before opening its second session.

Test two complete sessions against one simulator process:

```bash
.venv/bin/python -m tools.validate_crazysim_reconnect
```

The result distinguishes:

- `reconnect_supported`: both CFLib sessions received state;
- `bridge_alive_but_cflib_session_failed`: the second session failed but the
  bridge still answered its documented UDP `0xff` probe;
- `simulator_or_udp_bridge_not_running`: the endpoint did not answer, meaning
  MuJoCo exited, the launch script cleaned it up, or the bridge is unavailable.

Closing the MuJoCo viewer terminates the visualization loop. The launch
script's exit trap then kills the `cf2` firmware process. That is process
termination, not a surviving bridge refusing a second client.

## Body-rate and actuator validation

The firmware defines `RATE=0` and `ANGLE=1`. The legacy RPYT defaults are roll
angle, pitch angle, and yaw rate. The adapter explicitly writes and reads back
all three rate parameters:

```text
flightmode.stabModeRoll  = 0
flightmode.stabModePitch = 0
flightmode.stabModeYaw   = 0
```

CFLib negates pitch while serializing the legacy packet. Firmware then uses
the decoded roll and pitch directly as rate targets. Firmware negates legacy
yaw before storing `attitudeRate.yaw`. Actual response is read through
`DroneState.p/q/r`; `stateEstimateZ.ratePitch` itself contains a firmware-side
gyro-Y negation, so commanded and measured signs must be evaluated end to end.

The powered diagnostic is independent of the production controller. It is
bounded to 0.5 seconds, 0.25 rad/s, and normalized thrust 0.20. It requires two
explicit acknowledgement flags, refuses an unsafe or already-armed supervisor,
checks state freshness and supervisor state on every iteration, and always
sends STOP followed by disarm in cleanup. It logs commands, `p/q/r`, firmware
control thrust, and `motor.m1..m4` under `log/crazysim-diagnostics/`.

The firmware's `is_flying` flag is not a MuJoCo takeoff detector. In SITL,
`isFlyingCheck()` becomes true when any firmware motor ratio exceeds idle and
holds that condition for two seconds after the last above-idle observation. A
normal raw-thrust command can therefore move supervisor status from `0x000E`
to `0x001E` while the vehicle remains on the ground. The diagnostic requires
`is_flying=false` before command authorization, permits either value only
during the explicitly authorized bounded command, and continues to reject
stale telemetry, disarm, loss of can-fly, high-level control, tumble, crash,
and lock conditions.

Generic STOP decodes to a zero-initialized setpoint. Cleanup verifies
`stabilizer.thrust`, all four motor outputs, and all three controller targets
return to zero. It does not require `is_flying` to clear within the 250 ms
cleanup window because the firmware deliberately retains that flag for up to
two seconds; after the hysteresis expires, the Flying state can transition to
Landed. The final supervisor state and post-STOP outputs are recorded
separately.

Example syntax, to be used only after reviewing the chosen thrust:

```bash
.venv/bin/python -m tools.validate_crazysim_actuators \
  --axis roll --rate-rad-s 0.10 \
  --normalized-thrust 0.05 --duration 0.20 \
  --execute-powered-test --i-understand-motors-may-spin
```

Thrust stages are distinct:

1. Raw 0 is the legacy unlock packet and produces a zero setpoint.
2. Framework normalized 0 maps to raw 10001 for an ordinary command.
3. Firmware accepts legacy raw thrust above its `MIN_THRUST=1000`; CrazySim
   independently treats per-motor PWM below 7000 as stopped. Mixer/controller
   output therefore determines the first effective simulated motor value.
4. Hover thrust is model- and controller-dependent and remains unmeasured.

`motor.m1..m4` are the capped 16-bit motor outputs sent to CrazySim;
`motor.m1req..m4req` are also available as uncapped mixer requests, and
`stabilizer.thrust` exposes the firmware controller's collective output.
Emergency stop must be validated in a separate, explicitly reviewed run since
it can latch the supervisor into a locked state; it is not triggered by the
actuator diagnostic automatically.

The separate emergency-stop diagnostic requires two acknowledgement flags and
refuses an armed or flying SITL instance:

```bash
.venv/bin/python -m tools.validate_crazysim_emergency_stop \
  --execute-emergency-stop-test --i-understand-sitl-may-lock
```

## Minimal high-level flight demo

`simulation/scripts/demo_crazysim_flight.py` uses the firmware high-level
planner rather than the uncalibrated project PID. Installed CFLib commands 7
(`TAKEOFF_2`), 8 (`LAND_2`), and 3 (`STOP`) match the bundled CRTP-v7 firmware
layouts. The demo targets 0.35 m above the initial estimate over 3 seconds,
hovers for 2 seconds, lands to 0.03 m above the initial estimate over 3
seconds, and is bounded to 12 seconds total.

It requires the verified safe auto-armed lifecycle, stationary and level state,
fresh telemetry, zero startup motors/thrust/controller targets, and bounded
position, speed, and tilt throughout. Normal and exceptional cleanup both send
high-level STOP followed by generic STOP, then verify all logged outputs return
to zero. The default invocation is inspection-only:

```bash
.venv/bin/python -m simulation.scripts.demo_crazysim_flight
```

The explicitly powered invocation is:

```bash
.venv/bin/python -m simulation.scripts.demo_crazysim_flight --execute
```
