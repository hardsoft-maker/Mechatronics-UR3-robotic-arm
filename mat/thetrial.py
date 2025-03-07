import time
import itertools
import mujoco
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Load MuJoCo model and data
model_path = "C:/Users/smart/Downloads/mat/new_armature.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Get joint names and limits
joints = []
joint_limits = []
num_samples = 10
for i in range(model.njnt):
    joints.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i))
    joint_limits.append(model.jnt_range[i])

# Simulate a step to initialize
mujoco.mj_step(model, data)

# Generate all possible configurations within the joint limits
joint_ranges = [np.linspace(limit[0], limit[1], num=num_samples) for limit in joint_limits]
all_configurations = itertools.product(*joint_ranges)

# Initialize a list to store results
experiment_data = []

# Function to calculate inverse dynamics
def calculate_inverse_dynamics(model, data, qpos):
    mujoco.mj_resetData(model, data)
    data.qpos[:len(qpos)] = qpos
    mujoco.mj_forward(model, data)  # To compute velocity and acceleration
    mujoco.mj_inverse(model, data)  # To compute torques
    return data.qfrc_inverse[:len(qpos)]

# Iterate through all configurations and calculate torques
for config in all_configurations:
    torques = calculate_inverse_dynamics(model, data, config)
    lst = list(config) + list(torques)
    experiment_data.append(lst)

# Create a DataFrame to store the results
columns = joints + [f'torque_{joint}' for joint in joints]
df = pd.DataFrame(experiment_data, columns=columns)

# Save results to a CSV file
df.to_csv('robot_dynamics_results.csv', index=False)

# Read the CSV file for plotting
data = pd.read_csv('robot_dynamics_results.csv')

# Preparing the DataFrame for plotting
torque_columns = [f'torque_{joint}' for joint in joints]
df_melted = data.melt(value_vars=torque_columns, var_name='Joint', value_name='Torque')

# Plotting the torque distributions using a violin plot
plt.figure(figsize=(12, 8))
sns.violinplot(x='Joint', y='Torque', data=df_melted)
plt.title('Torque Distribution Across Joints')
plt.xlabel('Joint')
plt.ylabel('Torque')
plt.xticks(rotation=45)
plt.tight_layout()
plt.savefig('torques_new_armature.png')
plt.show()