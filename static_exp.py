import csv
import mujoco
import mujoco.viewer
import time
import numpy as np


def _store_torque_values(model_id: int, torques_list: list[float]):
  torques_data_file = f"C:/Users/smart/Downloads/desc/data/{model_id}_robot_model/static_torques_data.csv"

  # Write the torques data to a CSV file
  with open(torques_data_file, "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow([f"Torque_{i}" for i in range(len(torques_list[0]))])  # Assuming each torque is a list
      
    # Write each set of torques
    writer.writerows(torques_list)

  print(f"Valid trajectories saved to {torques_data_file}")


def run_static_experiment(model_id: int):
  model_path = f"C:/Users/smart/Downloads/desc/model/{model_id}_robot_model.xml" 

  model = mujoco.MjModel.from_xml_path(model_path)
  data = mujoco.MjData(model)

  # Define joint positions (in radians)
  positions = [
      np.deg2rad([0, 0, 0, 0, 0, 0]),
      np.deg2rad([0, 10, 10, 10, 10, 10]),
      np.deg2rad([0, 15, 15, 15, 15, 15]),
      np.deg2rad([0, 20, 20, 20, 20, 20]),
      np.deg2rad([5, 20, 20, 20, 20, 20]),
      np.deg2rad([50, -60, -50, 20, 20, 10])
  ]

  torques = []


  with mujoco.viewer.launch_passive(model, data) as viewer:
      start = time.time()

      while viewer.is_running() and time.time() - start < 2:
        step_start = time.time()

        mujoco.mj_forward(model, data)

        data.qpos = positions[-1]
        data.qacc[:] = 0

        mujoco.mj_inverse(model, data)

        if data.contact.geom.size == 2:  # Assuming valid if two geoms in contact
          print("data.qfrc_inverse", data.qfrc_inverse)  
          torques.append(data.qfrc_inverse.copy())

        viewer.sync()

        # Control the simulation step time
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


  _store_torque_values(model_id, torques_list=torques)