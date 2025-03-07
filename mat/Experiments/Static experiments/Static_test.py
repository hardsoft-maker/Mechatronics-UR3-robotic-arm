import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

MODEL_PATH = "C:/Users/smart/Downloads/mat/actuator_sizing/UR3_CRA-RI80-110-PRO-101.xml"

RATED_TORQUES = {
    "shoulder_pan_joint": 330,
    "shoulder_lift_joint": 330,
    "elbow_joint": 150,
    "wrist_1_joint": 54,
    "wrist_2_joint": 54,
    "wrist_3_joint": 54,
}

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

test_angles = np.linspace(-np.pi / 2, np.pi / 2, 10)

results = {joint: [] for joint in RATED_TORQUES.keys()}

for angle in test_angles:
    for i, joint in enumerate(RATED_TORQUES.keys()):
        data.qpos[i] = angle
    mujoco.mj_forward(model, data)
    
    for i, joint in enumerate(RATED_TORQUES.keys()):
        results[joint].append(data.qfrc_actuator[i])

plt.figure(figsize=(12, 6))
#sns.set_theme(style="whitegrid")
'''
plt.plot(1, 1)
sns.barplot(data=np.array(list(results.values())).T)
plt.xticks(ticks=range(len(RATED_TORQUES)), labels=RATED_TORQUES.keys(), rotation=30)
plt.ylabel("Torque (Nm)")
plt.title("Barplot of Torques")
'''
plt.plot(1, 1)
sns.violinplot(data=np.array(list(results.values())).T)
plt.xticks(ticks=range(len(RATED_TORQUES)), labels=RATED_TORQUES.keys(), rotation=30)
plt.ylabel("Torque (Nm)")
plt.title("Violinplot of Torques")

plt.tight_layout()
plt.show()