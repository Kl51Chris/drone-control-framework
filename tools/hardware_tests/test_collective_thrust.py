import time
from pathlib import Path

import mujoco
import mujoco.viewer

MODEL_PATH = Path(
    "models/mujoco_menagerie/bitcraze_crazyflie_2/scene.xml"
)

# 先从这个开始；之后按现象调。
THRUST =  0.26487

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

print("body mass =", model.body_mass[1], "kg")
print("gravity =", model.opt.gravity)
print("estimated hover thrust =", model.body_mass[1] * abs(model.opt.gravity[2]), "N")
print("commanded thrust =", THRUST, "N")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # [total thrust, roll moment, pitch moment, yaw moment]
        data.ctrl[:] = [THRUST, 0.0, 0.0, 0.0]

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(model.opt.timestep)
