import csv
import time
from datetime import datetime
from pathlib import Path

import mujoco
import mujoco.viewer

from controllers.altitude_pd import AltitudePD


PROJECT_ROOT = Path(__file__).resolve().parents[1]

MODEL_PATH = (
    PROJECT_ROOT
    / "models"
    / "mujoco_menagerie"
    / "bitcraze_crazyflie_2"
    / "scene.xml"
)

LOG_DIR = PROJECT_ROOT / "logs"

# Experiment settings
Z_REF = 1.0
SIM_DURATION = 12.0  # simulated seconds


def main() -> None:
    LOG_DIR.mkdir(exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    controller = AltitudePD(
        hover_thrust=0.26487,
        kp=0.8,
        kd=0.25,
        thrust_min=0.0,
        thrust_max=0.35,
    )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = LOG_DIR / f"altitude_hover_{timestamp}.csv"

    print(f"Model: {MODEL_PATH.name}")
    print(f"Reference altitude: {Z_REF:.2f} m")
    print(f"Simulation duration: {SIM_DURATION:.1f} s")
    print(f"Logging to: {log_path}")

    with open(log_path, "w", newline="") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "time",
                "z",
                "vz",
                "z_ref",
                "thrust",
                "roll_cmd",
                "pitch_cmd",
                "yaw_cmd",
            ],
        )
        writer.writeheader()

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time < SIM_DURATION:
                # MuJoCo free-joint state convention:
                # qpos = [x, y, z, qw, qx, qy, qz]
                # qvel = [vx, vy, vz, wx, wy, wz]
                z = float(data.qpos[2])
                vz = float(data.qvel[2])

                thrust = controller.compute(
                    z=z,
                    vz=vz,
                    z_ref=Z_REF,
                )

                # [total thrust, roll moment, pitch moment, yaw moment]
                data.ctrl[:] = [thrust, 0.0, 0.0, 0.0]

                writer.writerow(
                    {
                        "time": data.time,
                        "z": z,
                        "vz": vz,
                        "z_ref": Z_REF,
                        "thrust": thrust,
                        "roll_cmd": 0.0,
                        "pitch_cmd": 0.0,
                        "yaw_cmd": 0.0,
                    }
                )

                mujoco.mj_step(model, data)
                viewer.sync()

                # Viewer pacing only. Physics timestep remains model.opt.timestep.
                time.sleep(model.opt.timestep)

    print(f"Finished. Log saved to: {log_path}")


if __name__ == "__main__":
    main()