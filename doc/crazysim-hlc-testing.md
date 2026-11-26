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
