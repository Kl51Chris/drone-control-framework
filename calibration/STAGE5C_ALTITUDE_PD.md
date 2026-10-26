# Stage 5C altitude PD calibration readiness

## Recorded response

Source: `runs/20261026T064847_690179Z_pitch-response-id` (488 samples,
9.7403 s). This was a P-only flight; pitch excitation was skipped.

The 0.10 m takeoff reference ramp lasted 1.0 s. Actual altitude reached
0.1830 m at the end of TAKEOFF while rising at 0.2948 m/s, then peaked at
0.30734 m at 1.9402 s. During constant-reference HOVER, a seven-sample moving
average identifies a 0.30660 m peak at 1.9202 s and a 0.06436 m trough at
3.8202 s. Their 0.24224 m peak-to-trough span gives a 0.12112 m half-amplitude
and an approximate 3.80 s period. This estimate is low-confidence because the
four-second HOVER contains only one clear peak-to-trough half-cycle.

Maximum measured vertical speed was 0.39938 m/s. The P-only thrust range was
0.40428--0.42896 with no altitude saturation. The command followed the recorded
position error as implemented; continued motion after the thrust correction
changed sign is evidence of insufficient velocity damping, not a sign error in
the P term.

## Velocity feedback and reference semantics

The unified controller computes

`u_z = Kp * (z_ref - z) + Kv * (vz_ref - vz) + Ki * integral(error)`

before applying the existing correction and absolute-thrust bounds. `Kv` has
units of normalized thrust per (m/s). A positive gain reduces thrust during an
upward velocity error and increases it during a downward velocity error. It uses
the firmware's measured `stateEstimate.vz`; it does not differentiate altitude
and applies no additional host-side filtering. Consequently, firmware estimator
filtering and delay are assumed adequate for the initial low gain.

The flight runner previously omitted the derivative of its altitude reference,
so `vz_ref` was zero even during ramps. It now supplies +0.10 m/s in TAKEOFF,
0 in HOVER, -0.08 m/s in LANDING, and 0 in SETTLE/LANDING_CONFIRM. This has no
effect on Stage 5B because velocity feedback is disabled.

## Exogenous-state replay

Reproduce the replay with:

```bash
python -m calibration analyze-altitude-pd \
  calibration/runs/20261026T064847_690179Z_pitch-response-id \
  calibration/stage5_analysis/altitude_pd_latest
```

| Kv (thrust/(m/s)) | thrust min | thrust max | velocity term min/max | saturated samples |
|---:|---:|---:|---:|---:|
| 0.00 | 0.40428 | 0.42896 | 0 / 0 | 0 |
| 0.02 | 0.40347 | 0.43042 | -0.00599 / +0.00410 | 0 |
| 0.04 | 0.40206 | 0.43311 | -0.01198 / +0.00819 | 0 |
| 0.06 | 0.39813 | 0.43645 | -0.01796 / +0.01229 | 0 |
| 0.08 | 0.39254 | 0.43994 | -0.02395 / +0.01639 | 0 |

The unchanged bounds are 0.380--0.450, with correction bounds -0.045--+0.025
around equilibrium thrust 0.425. The recommended 0.04 candidate retains
0.01689 upward and 0.02206 downward absolute-thrust headroom on the recorded
states. No tested candidate saturated. Recorded states are exogenous, so these
results validate equations, signs, command ranges, and saturation only; they do
not predict a closed-loop PD trajectory.

## Candidate configuration

`stage5c_altitude_pd_candidate_config.json` changes only:

- stage: `5C`;
- altitude velocity feedback: enabled;
- altitude `velocity_kp`: `0.04`.

Altitude Kp remains 0.10, equilibrium thrust remains 0.425, integral action
remains disabled, and all thrust, attitude, heading, XY, and safety settings are
unchanged. Kv=0.04 is preferred over 0.02 because its maximum recorded damping
term is material relative to the P term, and over 0.06/0.08 because it retains
more authority and reduces exposure to estimator delay/noise. That choice is an
engineering assumption pending a physical PD response, not an identified
optimal gain.

## Landing review

The P-only flight crossed the descending reference but bounced. During
LANDING_CONFIRM it briefly met the touchdown altitude/velocity criteria for 12
samples (about 0.22 s), below the required continuous 0.5 s, then rose again.
It ended at about 0.1545 m with +0.0339 m/s and correctly triggered
`touchdown_not_confirmed` after the unchanged three-second timeout.

PD supplies -0.08 m/s as the descent-rate reference during LANDING and zero in
LANDING_CONFIRM. It should oppose both excessive descent and a rebound, but the
replay cannot establish that touchdown will occur. The existing rule remains:
motors are not normally stopped until relative altitude is at most 0.06 m and
absolute vertical speed at most 0.08 m/s continuously for 0.5 s. Timeout remains
a hard-safety shutdown.

## First supervised PD experiment

After reviewing the candidate and granting authorization for exactly one run:

```bash
python -m calibration run cascaded-5c \
  --config calibration/pitch_identification_flight_config.json \
  --controller-config calibration/stage5c_altitude_pd_candidate_config.json \
  --confirm-flight I_UNDERSTAND_PROPELLERS_MAY_SPIN
```

This uses the existing 0.10 m target, 1.0 s takeoff ramp, 4.0 s hover, controlled
landing, estimator gates, flight envelope, persistent logging, and shutdown
paths. XY remains disabled and no pitch excitation occurs.

Acceptance criteria for this diagnostic run are: no hard-safety event; no
altitude-command saturation; maximum altitude below 0.25 m; maximum absolute
vertical velocity below 0.35 m/s; at least 0.2 s continuously within 0.08 m of
the hover target with absolute vertical speed below 0.30 m/s; and normal
touchdown confirmation. Failure of a performance criterion is diagnostic and
does not authorize a gain increase.

**Recommendation: software GO for one explicitly authorized, supervised
Stage 5C diagnostic flight, but hardware performance remains unvalidated.** A
reviewer should confirm the separate candidate configuration, corrected
reference-velocity log, unchanged limits, clear area, and emergency-stop path.
No unattended repetition or automatic gain adjustment is permitted.
