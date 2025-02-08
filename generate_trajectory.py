import time
import mujoco
import mujoco.viewer
import numpy as np
import itertools
import json

# Load model
model_name = "3_robot_model"
model_path = f"C:/Users/smart/Downloads/desc/model/{model_name}.xml" 

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Generate all possible 6-value combinations
values = range(0, 360, 60)
static_trajectories = list(itertools.product(values, repeat=6))
valid_trajectories = []
iteration = 0

# File to store valid trajectories
file_path = "C:/Users/smart/Downloads/desc/valid_trajectories.json"

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < 400  and iteration < len(static_trajectories) - 1:
        step_start = time.time()

        # Set joint positions
        data.qpos = np.deg2rad(
            static_trajectories[iteration]
        )  # Convert degrees to radians
        data.qacc = [0] * 6

        # Check for collision condition
        if data.contact.geom.size == 2:  
            valid_trajectories.append(static_trajectories[iteration])
            print("Valid trajectory:", static_trajectories[iteration])
        else:
            print("Collision detected for trajectory:", static_trajectories[iteration])

        mujoco.mj_step(model, data)
        iteration += 1

        viewer.sync()

        # Control the simulation step time
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

# Save valid trajectories to file
with open(file_path, "w") as f:
    json.dump(valid_trajectories, f)
    print(f"Valid trajectories saved to {file_path}")
