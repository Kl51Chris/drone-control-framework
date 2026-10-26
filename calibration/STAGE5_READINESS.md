# Stage 5A readiness and Stage 5B preparation

## Decision

This document records the preflight readiness decision. The first supervised
custom-controller experiment has since been completed: powered attitude response
was bounded, altitude control was active, and horizontal displacement triggered
the configured hard abort at 0.50326 m before the profile completed. See
`STAGE5B_FIRST_FLIGHT_RESULT.md`. The default validation flag remains false and
no gain or thrust limit in `default_config.json` was changed automatically.

## PID inspection

The controller error is `reference altitude - measured altitude`, with world Z
positive upward. Positive proportional gain therefore raises collective thrust
when the vehicle is below the reference. Thrust is `equilibrium + PID output`.

Gain units, because output is normalized command, are:

- Kp: normalized thrust per metre;
- Ki: normalized thrust per metre-second;
- Kd: normalized thrust-seconds per metre.

Integration is forward rectangular. The derivative is backward error difference
over the measured loop interval, with zero derivative on the first update. It is
unfiltered and differentiates the setpoint, so nonzero Kd can kick on reference
changes and amplify estimator noise. Stage 5's first flight must use Kd=0.

The PID correction is clamped before it is added to equilibrium thrust. Final
thrust is clamped separately. Conditional integration rejects an integral update
that would drive an already saturated correction farther into saturation and
accepts one that drives it back toward range. With Ki=0 the stored integral is
intentionally irrelevant. Nonfinite reference and altitude inputs are now
rejected. Tests cover sign, terms, derivative, saturation recovery, invalid time,
and nonfinite inputs.

## Thrust authority

The Stage 4B stable mean candidate is 0.426080 and median is 0.424944. Against
the unchanged 0.45 ceiling, available upward authority is only 0.023920 from the
mean (5.61% of the equilibrium command) or 0.025056 from the median. Against the
current zero lower bound, nominal downward authority is about 0.425. Thus the
current bounds are severely asymmetric and the old equilibrium 0.30 is
inconsistent with the one recorded hover: it is 0.125 below the median candidate.
The candidate remains conditional because only one flight was measured and
battery, mass, propellers, ground effect, firmware build, and motor mixing were
not characterized.

For a first P-only trial, the proposed *review-only* command envelope is
equilibrium 0.425, minimum 0.380, maximum 0.450, PID correction [-0.045, +0.025],
and Kp=0.10 m^-1 with Ki=Kd=0. This preserves the existing ceiling rather than
raising it. It gives 0.045 downward and 0.025 upward authority. Proportional
correction reaches the upper limit at +0.25 m error and the lower limit at
-0.45 m error. These are command-scale choices, not proof that either bound is
physically safe or sufficient for takeoff.

## Offline replay

`pid_replay.py` replays the 394 official-hover rows and writes P/I/D terms,
unsaturated correction, final predicted thrust, and separate saturation flags.
Artifacts are under `stage5_analysis/`.

| replay | predicted thrust min / mean / max | PID saturation | thrust saturation |
|---|---:|---:|---:|
| unchanged defaults (0.30, Kp=.25, Kd=.08) | .272291 / .296963 / .341805 | 0 / 394 | 0 / 394 |
| proposed P-only candidate | .417746 / .424091 / .435105 | 0 / 394 | 0 / 394 |

The measured error range was -0.07254 to +0.10105 m and RMSE was 0.04944 m over
the whole profile. These are errors generated while the official firmware
controller was flying. Predicted commands did not affect the recorded vehicle.
This is an arithmetic/range replay, **not closed-loop simulation**, and says
nothing about stability, lift-off, tracking, or landing under the custom loop.

## Transition and state-machine review

Normal custom operation uses ramped TAKEOFF, HOVER, LANDING, and SETTLE references,
then sends repeated stop packets and disarms. The stop packet terminates lift; it
is not a controlled landing. The custom path retains firmware body-rate loops and
the mixer, but replaces the firmware attitude/altitude command path with custom
attitude-rate and collective commands.

Preparation changes add protocol >=12, Flow Deck, emergency-stop availability,
fresh-state, supervisor, tilt, altitude, vertical speed, horizontal displacement,
horizontal speed, loop-deadline, and explicit powered-attitude interlocks.
Safety evidence is flushed to metadata before abort. Hard violations still use
immediate stop/disarm. The generic custom runner does not yet have the official
hover runner's independently tested fault-classification and recoverable landing
machinery. That is another reason the present recommendation is NO-GO rather
than authorization to fly.

## Required first-flight procedure

1. Keep the default interlock false. With props removed, rerun preflight, state
   monitor, and guided attitude-sign checks; review freshness and supervisor data.
2. Perform a separately designed, restrained powered attitude-stabilization test
   with an observer and immediate power removal. This step needs explicit human
   authorization and is not initiated by this software review.
3. Only after that evidence passes, copy the default to a per-run file, record
   `custom_attitude_hardware_validated: true`, and enter the reviewed P-only
   candidate. Do not alter the default or raise 0.45.
4. For the first free trial, propose a 0.20 m height, 0.10 m/s takeoff, 3 s hover,
   and 0.08 m/s landing. Review tighter trial limits: 0.50 m altitude, 0.60 m/s
   vertical speed, 0.25 rad tilt, 0.25 m horizontal displacement, 0.40 m/s
   horizontal speed, 0.10 s state age, and 0.06 s loop interval. These values are
   proposals requiring human approval, not validated settings.
5. Use a charged vehicle over a textured, level surface in a cleared area, with
   an observer, reachable battery disconnect, and no competing commander. Start
   exactly one `p-only` process with the explicit confirmation. No repeat or gain
   search is allowed.
6. Abort immediately for tilt, displacement, velocity, supervisor, stale-state,
   or deadline violations. Attempt the normal landing profile only while state,
   attitude, link, and thrust control remain trustworthy; use emergency stop for
   a hard violation. Preserve the run directory before any next decision.

## Acceptance criteria for that one flight

All must pass: normal controlled landing; no safety event or missed deadline;
no stale/nonfinite state; at least 100 hover samples; hover altitude RMSE <=0.08 m;
overshoot <=0.08 m; absolute tilt <=0.25 rad; horizontal displacement <=0.25 m;
horizontal speed <=0.40 m/s; absolute vertical speed <=0.60 m/s; and no more than
5% saturated commands. The trace must show no sustained or growing altitude or
attitude oscillation on human review. Passing permits analysis and a decision on
the next single run; it does not automatically authorize gain changes or repeats.
