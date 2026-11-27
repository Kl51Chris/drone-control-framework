# CrazySim MPC testing

Run these tests in CrazySim only. The commands below assume the local project,
the CrazySim checkout and a built SITL firmware. Run each flight with a fresh
SITL instance. Install the controller and plotting dependencies in the project
environment before the first run.

```bash
cd /home/chris/drone-project
.venv/bin/python -m pip install -r requirements-ubuntu-lock.txt
.venv/bin/python -m pip install matplotlib==3.10.8
```

In terminal 1, start the firmware in a fresh working directory.

```bash
cd "$(mktemp -d)"
/home/chris/CrazySim/crazyflie-firmware/sitl_make/build/cf2 19950
```

In terminal 2, start the headless MuJoCo simulation.

```bash
sim=/home/chris/CrazySim
"$sim/.venv/bin/python" \
  "$sim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py" \
  --model-type cf2x_T350 --port 19950 --dt 0.001 -- 0,0
```

In terminal 3, run the bidirectional handover diagnostic.

```bash
cd /home/chris/drone-project
.venv/bin/python -m simulation.scripts.demo_crazysim_flight --execute --verify-handover
```

The diagnostic prints a JSON log path. For identification, add
`--axis roll|pitch|yaw|thrust` and `--direction 1|-1`, selecting one axis and
one direction per run. Stop and restart both SITL processes before each run.
The JSON keeps the measured state, command, and host timestamp for each
body-rate step.

To check the production CLI with PID as the tracking controller, start a
fresh SITL instance and run:

```bash
.venv/bin/python -m experiments.run_flight --backend crazysim \
  --configuration crazysim-cf2x-t350-calibrated --flight-lifecycle hlc --execute
```

Use the directory printed as `[LOG]` as the input to the existing analyzer.

```bash
.venv/bin/python -m tools.analyze_flight_log /absolute/path/to/run-directory
```

HLC phases have blank body-rate command fields. The default
`--flight-lifecycle controller` retains the existing outer-loop flight path.

## MPC scenarios

In the project directory, run one line per fresh simulator instance. Omitting
`--execute` prints the resolved configuration and exits without connecting.
All four commands use official takeoff and landing.

```bash
export OPENBLAS_NUM_THREADS=1
.venv/bin/python -m experiments.run_flight --backend crazysim --controller outer-loop-mpc --execute
.venv/bin/python -m experiments.run_flight --backend crazysim --controller outer-loop-mpc --step-axis x --execute
.venv/bin/python -m experiments.run_flight --backend crazysim --controller outer-loop-mpc --reference circular --execute
.venv/bin/python -m experiments.run_flight --backend crazysim --controller outer-loop-mpc --force-solver-failure --execute
```

The step option accepts `x`, `y`, or `z`. The failure scenario raises a
controlled solver error after body-rate tracking is established. A passing
failure scenario enters `FALLBACK_LANDING` and satisfies the same estimated
landing criteria. The failure injection tests the recovery path, not the
solver's native status for infeasibility.

Use `--horizon 25` to select the number of 20 ms steps (25 is the default).
`--mpc-config path.json` accepts overrides. For example:

```json
{
  "horizon": 25,
  "q": [120, 120, 200, 12, 12, 24, 6, 6, 10],
  "r": [2, 2, 1, 12],
  "model": {"equilibrium_thrust": 0.47681863, "thrust_gain_m_s2": 14.495}
}
```

State order is position, velocity, roll, pitch and yaw. Input order is
roll-rate, pitch-rate, yaw-rate and normalized thrust deviation from hover.
The model is local to fixed-yaw near-hover flight. JSON model overrides adjust
the parameters of the nine-state model. Keep the model period at 20 ms for
this runner. Set `OPENBLAS_NUM_THREADS=1` for comparable CPU timing.

Analyzing one or more printed log directories generates a JSON summary and
PNG per run.

```bash
.venv/bin/python -m tools.analyze_predictive_flight /absolute/path/to/run-directory
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m tools.analyze_crazysim_identification --benchmark
.venv/bin/python -m tools.analyze_crazysim_identification --predict path/to/independent-pulse-run.json
```

`metadata.json` contains the resolved controller, model, scenario and
reference configuration. `flight.csv` and `events.csv` hold applied-command
provenance, compact solver diagnostics, deadlines and phase transitions.
`predictions.jsonl` holds background-written snapshots keyed by control
`step_index`. Inputs in snapshots use the MPC model convention, while
`flight.csv` uses normalized absolute collective thrust. Online prediction
errors compare a previously planned trajectory with later observations.
The separate identification analyzer rolls out the model using recorded
future commands. Neither analysis uses independent MuJoCo ground truth.

At the HLC handover the controller uses a hover seed for the first slew
penalty. This seed has no associated prior body-rate packet. Subsequent
updates penalize change from the previously issued command. The repository
retains the original PID-controlled flight path as an explicit alternative.

## Recorded baseline

The local validation used CrazySim checkout `3ec8b55`, firmware checkout
`aa6571dc`, CF2X T350 and 1 ms MuJoCo physics steps. Raw local artifacts are
under the git-ignored `log/` directory.

| Scenario | Local run directory prefix | Result |
| --- | --- | --- |
| Hover lifecycle | `20261126_161258_fixed-hover` | 100 tracking steps, Z RMSE 7.9 mm |
| X step | `20261126_161314_fixed-hover` | 100 tracking steps, X RMSE 16.8 mm |
| Multi-axis circle | `20261126_161330_circular` | 750 tracking steps, X/Y RMSE 15.6/14.3 mm |
| Injected solver failure | `20261126_161400_fixed-hover` | 20 tracking steps followed by official fallback landing |

These runs had zero recorded deadline misses and zero dropped prediction
snapshots. HLC landing is qualified by estimator criteria and post-STOP
actuator inactivity. Data are local to this simulator and test configuration.

## Phase 2 evaluation

Run from the repository root. The matrix tool defaults to three repeats.
Each matrix uses its own output directory, and each case starts new firmware
and MuJoCo processes. Stop any process using the selected firmware port or
choose a different port. CFLib connects on the firmware port minus 100.

```bash
.venv/bin/python -m tools.mpc_phase2_matrix compare > /tmp/mpc-compare.json
.venv/bin/python -m tools.run_crazysim_batch /tmp/mpc-compare.json log/my-mpc-compare
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m tools.analyze_mpc_batch log/my-mpc-compare
```

Add `--gui` to the batch command to show the MuJoCo viewer. The default is
headless. After reviewing the paired comparison, generate stage `nominal`
and then `horizon` using the same commands with fresh output directories and
pass `--mode preview` or `--mode constant` to the matrix generator.

`--reference-mode constant` is the original MPC objective. In preview mode,
the circular reference is sampled at the future control ticks, with samples
after the circle clamped to its endpoint. Both modes use the same solver
structure. Steps are unannounced in both modes. For example, add
`--step-axis yaw --step-amplitude -0.0872664626 --step-hold 2` for a -5° step.
XYZ step amplitudes are in meters and yaw amplitudes are in radians.
Step hold is limited to 2 s by the existing fixed-hover flight timeout.
An unsettled response is reported as censored at the end of the observation.

For circles, use `--circle-radius`, `--circle-duration` and
`--circle-speed-profile smoothstep|smootherstep`. For one revolution, mean
speed is 2*pi*radius/duration and peak speed is respectively 1.5 or 1.875
times the mean. The factory profile remains cubic smoothstep.

The truth observer records simulator time, host monotonic time, world
position, XYZW quaternion, world linear velocity, and body angular velocity
from MuJoCo. It returns the original pose unchanged to the simulator.
The analysis aligns truth to the host-timestamped observations, reports
interpolation gaps, and reports estimation error separately from
reference-to-truth tracking error. The model validation rolls out the
existing model from truth with recorded host-issued commands. It is distinct
from comparing an online MPC plan with later measurements.
Each run saves commands and status in `experiment.json`, including failed
runs. The analysis generates per-run plots, `summary.json`, `aggregate.json`
and grouped comparison plots.

See [Phase 2 evaluation results](mpc-phase2-results.md) for the completed
comparison, nominal runs, horizon sweep, and local artifact locations.
