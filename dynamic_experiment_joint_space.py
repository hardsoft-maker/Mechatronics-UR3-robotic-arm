import csv, json
import mujoco
import mujoco.viewer
import time
import numpy as np

def _store_torque_values_dynamic(model_id: int, torques_list: list[float]):
    torques_data_file = f"C:/Users/smart/Downloads/desc/data/{model_id}_robot_model/dynamic_torques_data.csv"

    # Write the torques data to a CSV file
    with open(torques_data_file, "w", newline='') as f:
        writer = csv.writer(f)
        writer.writerow([f"Torque_{i}" for i in range(len(torques_list[0]))])  # Assuming each torque is a list
        writer.writerows(torques_list)

    print(f"Valid trajectories saved to {torques_data_file}")




def _read_trajectories():
    file_path = "C:/Users/smart/Downloads/desc/data/valid_trajectories.json"

    # Load valid trajectories from file
    with open(file_path, "r") as f:
        loaded_trajectories = json.load(f)
        print(f"Loaded {len(loaded_trajectories)} valid trajectories from {file_path}")

    return loaded_trajectories



def run_dynamic_experiment(model_id: int):

    # Load model
    model_path = f"C:/Users/smart/Downloads/desc/model/{model_id}_robot_model.xml" 

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    loaded_trajectories = _read_trajectories()

    iteration = 0
    torques = []

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start = time.time()
        while viewer.is_running() and time.time() - start < 300 and iteration < len(loaded_trajectories) - 1:
            step_start = time.time()
            mujoco.mj_forward(model, data)
            # Set joint positions
            data.qpos = np.deg2rad(
                loaded_trajectories[iteration]
            )  # Convert degrees to radians
            data.qacc = [0] * 6

            mujoco.mj_inverse(model, data)
            torques.append(data.qfrc_inverse)
            iteration += 1

            viewer.sync()

            # Control the simulation step time
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    _store_torque_values_dynamic(model_id, torques)


