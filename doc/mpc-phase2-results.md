# MPC Phase 2 evaluation

## Experimental conditions

CrazySim CF2X T350, 1 ms physics steps, 50 Hz host runner, official HLC
takeoff and landing, original nine-state controller model and OSQP. Every
episode starts new firmware and simulator processes. Initial pose and
physical options are held fixed. The batch artifacts include the exact
commands, matrix, controller/reference configuration, per-run status,
firmware and simulator output, flight CSV, prediction sidecar, and
independent MuJoCo truth CSV. The results refer to local, ignored log files.

Truth and estimated state use different streams. Scores interpolate truth
at the logged control observations, while lag uses the strictly increasing
reference clock to differentiate the circle. Attitude validation confirmed
the legacy pitch sign convention. The prediction error measurement
initializes from truth and replays the host-issued commands. Actual firmware
acceptance timestamps are unavailable. The phase lag statistic is the
median signed angular error divided by reference angular speed over the
high-speed part of the circle.

## Reference mode comparison

Three paired runs per mode, 0.10 m radius, 15 s, one revolution, N=25.
Each mode passed all three runs. Means per independent run:

| Mode | X/Y truth RMSE | Phase lag | QP P99 |
| --- | --- | --- | --- |
| Constant | 16.0 / 14.2 mm | 0.437 s | 1.03 ms |
| Preview | 4.6 / 4.1 mm | 0.092 s | 1.03 ms |

There was no predicted input or state-bound activation in these cases.
Preview lowered roll and pitch increment RMS, with a small increase in
thrust increment RMS and collective effort. Use preview for known circles
and retain constant as a selectable baseline. Unexpected steps remain
unannounced in both modes.

## Nominal evaluation

All 30 cases passed the safety and timing screen. Each signed axis step was
repeated three times, with 0.03 m XYZ and 5 degree yaw amplitudes.
XYZ holds used the same 2-second evaluation window as yaw. Settling is
the first in-band observation after the last out-of-band sample with at
least 0.2 s subsequently observed in-band. The tolerance is the larger
of 5 percent amplitude and 0.002 m or rad, respectively.

| Scenario | Result |
| --- | --- |
| X/Y signed steps | Approximately 15.4 mm tracking-phase RMSE. End-window error approximately 3.0 mm. All runs censored for settling. |
| Z positive / negative steps | 3/3 settled each. Mean settling 1.19 / 1.26 s. |
| Yaw signed steps | About 1.94 degrees tracking-phase RMSE and 0.32 degrees end-window error. All runs censored for settling. |
| Faster circle, 0.10 m / 10 s | X/Y RMSE 7.2 / 6.4 mm, lag 0.11 s. |
| Scaled circle, 0.08 m / 12 s | X/Y RMSE 4.9 / 4.3 mm, lag 0.09 s. |

Tracking-phase RMSE includes the pre-step portion. End-window means are
reported separately and do not establish steady state on censored runs.
The 9.5 s fixed-hover safety timeout allows at most a 2 s hold under the
current supervised lifecycle. Establishing longer observations requires
an explicit future protocol decision.

Reproduction commands are in the [CrazySim testing guide](crazysim-hlc-testing.md).

## Horizon sweep

The same preview policy, weights, model, and constraints were kept fixed
for three paired repeats of the baseline circle and unexpected X step.
The reported QP numbers are means of each circle run's P99.

| Horizon | Circle X/Y truth RMSE | Circle lag | Circle QP P99 | X step |
| --- | --- | --- | --- | --- |
| 15 / 0.30 s | 6.8 / 4.9 mm | 0.102 s | 0.69 ms | 0/3 settled, X RMSE 17.6 mm |
| 25 / 0.50 s | 4.9 / 4.2 mm | 0.095 s | 1.20 ms | 0/3 settled, X RMSE 15.4 mm |
| 40 / 0.80 s | 3.8 / 3.5 mm | 0.076 s | 2.05 ms | 3/3 settled, 1.47 s mean, X RMSE 14.0 mm |

All 18 sweep runs passed the feasibility screen. Across all 54 scored runs
there were no observed deadline misses or unexpected fallbacks. The largest
per-run P99 truth interpolation bracket was 12.0 ms. The largest per-run
P99 incremental observer callback time was 0.0163 ms.

Carry 40 steps forward as the performance candidate and 25 as the cheaper
control. The tracking optimum is at the tested boundary. If the next weight
tuning leaves comparable timing headroom, include one longer horizon in
validation. Start by separating XY/yaw tracking weights from rate effort,
with slew weights fixed. Tune slew only after checking overshoot and the
first body-rate command after handover. Keep 25 and 40 available for
confirmation runs. No final horizon is frozen.

## Local artifacts

- Reference comparison: `log/mpc-phase2-compare`
- Nominal: `log/mpc-phase2-nominal`
- Completed horizon rerun: `log/mpc-phase2-horizon-repeat`
- Prior interrupted horizon attempt: `log/mpc-phase2-horizon`
- Isolated pilot: `log/mpc-phase2-pilot-isolated`

The initial pilot failed to bind the occupied default UDP port. The initial
horizon attempt was interrupted by SIGTERM. Both records were preserved and
excluded from the completed paired results.
