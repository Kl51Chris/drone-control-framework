import time

import mujoco
import mujoco.viewer


XML = """
<mujoco model="falling_box">
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <worldbody>
    <light pos="0 0 3"/>

    <geom
      type="plane"
      size="5 5 0.1"
      rgba="0.2 0.2 0.2 1"
    />

    <body pos="0 0 1">
      <freejoint/>

      <geom
        type="box"
        size="0.1 0.1 0.1"
        rgba="0.2 0.6 0.9 1"
      />
    </body>
  </worldbody>
</mujoco>
"""


def main():
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            mujoco.mj_step(model, data)
            viewer.sync()

            remaining = model.opt.timestep - (time.time() - step_start)
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()
