import numpy as np
# Define initial and final joint configurations
q_start = np.array([0, 0, 0, 0, 0, 0])  # Example initial configuration
q_end = np.array([1.0, 0.5, -0.5, 0.2, 0.3, 0.1])  # Example final configuration
# Define initial and final end-effector positions
X_start = np.array([0.5, 0.2, 0.3])  # Example initial position
X_end = np.array([0.7, 0.3, 0.4])  # Example final position
def polynomial_trajectory(t, T):
    a3 = 10 / T**3
    a4 = -15 / T**4
    a5 = 6 / T**5
    s = a3 * t**3 + a4 * t**4 + a5 * t**5
    s_dot = 3 * a3 * t**2 + 4 * a4 * t**3 + 5 * a5 * t**4
    s_ddot = 6 * a3 * t + 12 * a4 * t**2 + 20 * a5 * t**3
    return s, s_dot, s_ddot

def generate_trajectory(T, dt, q_start, q_end):
    t = np.arange(0, T, dt)
    s, s_dot, s_ddot = polynomial_trajectory(t, T)
    q_traj = q_start + s[:, np.newaxis] * (q_end - q_start)
    q_dot_traj = s_dot[:, np.newaxis] * (q_end - q_start)
    q_ddot_traj = s_ddot[:, np.newaxis] * (q_end - q_start)
    return t, q_traj, q_dot_traj, q_ddot_traj

def generate_cartesian_trajectory(T, dt, X_start, X_end):
    t = np.arange(0, T, dt)
    s, s_dot, s_ddot = polynomial_trajectory(t, T)
    X_traj = X_start + s[:, np.newaxis] * (X_end - X_start)
    X_dot_traj = s_dot[:, np.newaxis] * (X_end - X_start)
    X_ddot_traj = s_ddot[:, np.newaxis] * (X_end - X_start)
    return t, X_traj, X_dot_traj, X_ddot_traj
import mujoco
import matplotlib.pyplot as plt

# Load the model
model = mujoco.MjModel.from_xml_path('C:/Users/smart/Downloads/mat/new_armature.xml')
data = mujoco.MjData(model)

# Define simulation parameters
T = 5.0  # Total time for the trajectory
dt = 0.01  # Time step
time_steps = int(T / dt)

# Generate joint space trajectory
t, q_traj, q_dot_traj, q_ddot_traj = generate_trajectory(T, dt, q_start, q_end)

# Generate Cartesian space trajectory
t_cart, X_traj, X_dot_traj, X_ddot_traj = generate_cartesian_trajectory(T, dt, X_start, X_end)

# Simulate the joint space trajectory
joint_positions = []
for i in range(time_steps):
    data.ctrl[:] = q_traj[i]  # Set the joint positions
    mujoco.mj_step(model, data)
    joint_positions.append(data.qpos.copy())

# Simulate the Cartesian space trajectory
# (This requires inverse kinematics to convert Cartesian positions to joint positions)
# For simplicity, we assume a direct mapping here
cartesian_positions = []
for i in range(time_steps):
    # Perform inverse kinematics to get joint positions from Cartesian positions
    # For now, we just use the joint space trajectory as a placeholder
    data.ctrl[:] = q_traj[i]  # Set the joint positions
    mujoco.mj_step(model, data)
    cartesian_positions.append(data.qpos.copy())
# Plot joint space trajectory
plt.figure(figsize=(12, 6))
for i in range(6):
    plt.plot(t, q_traj[:, i], label=f'Desired Joint {i+1}')
    plt.plot(t, np.array(joint_positions)[:, i], '--', label=f'Actual Joint {i+1}')
plt.xlabel('Time (s)')
plt.ylabel('Joint Position (rad)')
plt.title('Joint Space Trajectory')
plt.legend()
plt.grid(True)
plt.show()
plt.savefig('Joint Space Trajectory.png')

# Plot Cartesian space trajectory
plt.figure(figsize=(12, 6))
for i in range(3):
    plt.plot(t_cart, X_traj[:, i], label=f'Desired Cartesian {i+1}')
    plt.plot(t_cart, np.array(cartesian_positions)[:, i], '--', label=f'Actual Cartesian {i+1}')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Cartesian Space Trajectory')
plt.legend()
plt.grid(True)
plt.show()
plt.savefig('Cartesian Space Trajectory.png')
