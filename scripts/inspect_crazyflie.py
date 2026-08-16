from pathlib import Path
import mujoco

MODEL_PATH = Path(
    "models/mujoco_menagerie/bitcraze_crazyflie_2/scene.xml"
)

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

print("\n=== Basic model dimensions ===")
print(f"nq       = {model.nq}")          # generalized coordinates
print(f"nv       = {model.nv}")          # generalized velocities
print(f"nu       = {model.nu}")          # actuator input dimension
print(f"njnt     = {model.njnt}")
print(f"nbody    = {model.nbody}")
print(f"nsensor  = {model.nsensor}")
print(f"timestep = {model.opt.timestep}")

print("\n=== Joints ===")
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    qpos_adr = model.jnt_qposadr[i]
    dof_adr = model.jnt_dofadr[i]

    print(
        f"[{i}] name={name}, "
        f"type={model.jnt_type[i]}, "
        f"qpos_adr={qpos_adr}, "
        f"dof_adr={dof_adr}"
    )

print("\n=== Actuators ===")
for i in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)

    if model.actuator_ctrllimited[i]:
        ctrl_range = model.actuator_ctrlrange[i]
    else:
        ctrl_range = "unlimited"

    print(
        f"[{i}] name={name}, "
        f"ctrl_range={ctrl_range}, "
        f"gear={model.actuator_gear[i]}, "
        f"trntype={model.actuator_trntype[i]}, "
        f"trnid={model.actuator_trnid[i]}"
    )

print("\n=== Sensors ===")
for i in range(model.nsensor):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
    print(
        f"[{i}] name={name}, "
        f"type={model.sensor_type[i]}, "
        f"dim={model.sensor_dim[i]}, "
        f"adr={model.sensor_adr[i]}"
    )

print("\n=== Initial state ===")
print("qpos =", data.qpos)
print("qvel =", data.qvel)
print("ctrl =", data.ctrl)
