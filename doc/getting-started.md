# Getting started

This guide is for a new student working on the active Crazyflie framework.
Begin with software-only inspection and tests. Hardware commands below are
experimental and require a supervised, explicitly planned run.

## Prerequisites

Software-only work requires Linux or Ubuntu/WSL, Git, Python 3, and Python's
`venv` module. The lock files include the Crazyflie library (`cflib==0.1.32`),
NumPy, pytest, and the repository's ROS/MuJoCo dependencies.

Hardware work additionally requires a Crazyflie 2.x, Crazyradio access, USB
permissions for the radio, a charged battery, and a Flow Deck V2 (the current
state provider uses onboard estimator position, velocity, attitude, and body
rates). Motive/OptiTrack is not required for the current fixed-hover path.

## Installation

```bash
git clone git@github.com:Kl51Chris/drone-control-framework.git
cd drone-control-framework
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-ubuntu-lock.txt
python -c "import cflib, numpy, pytest; print('environment OK')"
```

For WSL, install from `requirements-wsl-lock.txt` instead. The repository has
no separate `setup.py` or `pyproject.toml`; the lock file is the supported
dependency installation source.

## First demo: inspection mode

From the repository root:

```bash
python -m experiments.run_flight \
    --controller outer-loop-pid \
    --configuration stage5-hardware-candidate \
    --reference fixed-hover
```

The command prints `[CONTROLLER]`, `[CONFIGURATION]`, `[REFERENCE]`, resolved
`[PARAMETERS]`, `[SAFETY]`, and `[STATUS] inspection_only`. Without `--execute`,
it does not open a Crazyflie radio connection. This is configuration inspection,
not a physics simulation or closed-loop plant simulation.

## Experimental hardware demo

Only run this after a supervisor has reviewed the procedure and the flight area:

```bash
python -m experiments.run_flight \
    --controller outer-loop-pid \
    --configuration stage5-hardware-candidate \
    --reference fixed-hover \
    --execute \
    --uri radio://0/80/2M
```

This is an experimental supervised demonstration, not a validated autonomous
flight procedure. The current fixed-hover reference is 0.30 m above the
initialized ground estimate and uses:

- armed idle: 0.5 s;
- takeoff: 3.0 s;
- hover: 3.0 s;
- descent reference: 3.0 s;
- `LANDING_CHECK` through completion or the 15.0 s maximum duration.

The runner checks estimator freshness and finite values, position/velocity,
attitude, supervisor state, control-loop deadlines, maximum duration, and
flight-envelope limits. It requests an emergency stop for an armed hard
exception and always stops command output, requests disarm, and stops the state
provider in cleanup. Ctrl-C follows the runner's normal stop/disarm cleanup;
the code does not claim that this guarantees shutdown during radio loss.

Before a run, verify the battery, Crazyradio link, Flow Deck/estimator
readiness, clear isolated flight area, operator supervision, and a known
emergency-stop procedure. Do not improvise undocumented keyboard controls.

Landing success is estimator-qualified, not physical-contact sensing: low
estimated height and low vertical speed must both pass. `LANDING_FAILED` may
therefore trigger emergency motor shutdown while the vehicle is still airborne.
The production path currently prints console telemetry and result data; no
persistent production flight-file logger has been verified here.

## Controller and reference selection

`experiments/run_flight.py` accepts:

- `--controller`: currently `outer-loop-pid`;
- `--configuration`: currently `stage5-hardware-candidate`;
- `--reference`: `fixed-hover` (default) or `circular`;
- `--execute`: explicit switch from inspection to hardware execution;
- `--uri`: Crazyflie radio URI, default `radio://0/80/2M`.

The circular provider is reusable for software inspection, but powered circular
tracking is interlocked until fixed-hover production validation is further
reviewed. New students should start at this production entry point, not with
calibration hardware scripts.

## Running tests

Run the active production and calibration suites with:

```bash
python -m pytest velocity_control_framework/tests calibration/tests
```

Controller unit tests check equations, limits, mappings, and state handling.
Calibration equivalence tests compare the production and calibration cascaded
controllers on identical histories. Hardware experiments are separate runs;
passing software tests is not flight validation.

Running pytest at repository root currently has a collection blocker: the
MuJoCo hardware test imports `models/mujoco_menagerie/bitcraze_crazyflie_2/scene.xml`,
which is not present in this checkout. This does not affect the two suites above.

## Troubleshooting

- **Import or dependency errors:** activate `.venv` and install the matching
  Ubuntu or WSL lock file; confirm with the environment command above.
- **Crazyradio connection failure:** check USB permissions, URI, radio power,
  and that no other client owns the link. Do not retry an armed run blindly.
- **Supervisor arming refusal:** inspect the printed supervisor state. The
  runner requires `can_be_armed`, rejects locked/crashed/tumbled states, sends
  the arm request, then waits for `is_armed` and `can_fly`; a refusal or timeout
  requests disarm and cleans up.
- **Estimator readiness or safety violation:** stop and inspect the state age,
  continuity, position, velocity, attitude, and supervisor diagnostics. Do not
  raise limits as a first response.
- **`LANDING_FAILED`:** this means the estimator landing criteria were not met
  before the maximum duration. It is not proof that the vehicle touched down;
  preserve the result and review telemetry before another run.
