import mujoco
import mujoco.viewer
import obotx_sim.utils.transformations as T
from pathlib import Path

import numpy as np

SCENE_PATH = Path(__file__).parent.parent.parent / "models" / "scene.xml"
TIMESTEP = 1e-3


def run_simulation() -> None:
    try:
        mj_model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        # simulation options
        mj_model.opt.timestep = TIMESTEP
        mj_model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST

        mj_data = mujoco.MjData(mj_model)
        mujoco.mj_step(mj_model, mj_data)

        with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
            # adjust viewer view
            viewer.cam.azimuth = -38.5
            viewer.cam.elevation = -36.75
            viewer.cam.distance = 10
            viewer.cam.lookat =np.array([0.10027655, 0.1260649 , 2.86440666])

            steps=0
            while viewer.is_running():
                mujoco.mj_step(mj_model, mj_data)
                steps+=1
                if steps%10==0:
                    viewer.sync()

    except KeyboardInterrupt:
        print("\nExiting simulation...")
    except Exception as e:
        print(f"Simulation error: {e}")
        raise e
