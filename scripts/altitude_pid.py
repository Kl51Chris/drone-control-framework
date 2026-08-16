import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


MODEL_PATH = Path(
    "models/mujoco_menagerie/bitcraze_crazyflie_2/scene.xml"
)

# Desired altitude
Z_REF = 0.5

# Hover operating point found from the previous test
T_HOVER = 0.26487

# Start conservative. We tune after verifying sign/direction.
KP_Z = 0.8
KD_Z = 0.25
KI_Z = 0.0

# From XML actuator range
T_MIN = 0.0
T_MAX = 0.35


model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

dt = model.opt.timestep
integral_error = 0.0

print(f"dt = {dt}")
print(f"z_ref = {Z_REF}")
print(f"T_hover = {T_HOVER}")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # qpos = [x, y, z, qw, qx, qy, qz]
        z = data.qpos[2]

        # qvel = [vx, vy, vz, wx, wy, wz]
        vz = data.qvel[2]

        error_z = Z_REF - z
        integral_error += error_z * dt

        # Prevent integral windup later if KI is enabled
        integral_error = np.clip(integral_error, -0.2, 0.2)

        thrust = (
            T_HOVER
            + KP_Z * error_z
            - KD_Z * vz
            + KI_Z * integral_error
        )

        thrust = np.clip(thrust, T_MIN, T_MAX)

        # [total thrust, roll moment, pitch moment, yaw moment]
        data.ctrl[:] = [thrust, 0.0, 0.0, 0.0]

        mujoco.mj_step(model, data)
        viewer.sync()

        time.sleep(dt)
