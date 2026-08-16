import time
from pathlib import Path

import mujoco
import mujoco.viewer

MODEL_PATH = Path(
    "models/mujoco_menagerie/bitcraze_crazyflie_2/scene.xml"
)

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

print(f"Loaded: {MODEL_PATH}")
print(f"nq={model.nq}, nv={model.nv}, nu={model.nu}")
print("Close the viewer window to exit.")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(model.opt.timestep)
