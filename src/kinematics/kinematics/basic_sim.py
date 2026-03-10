import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
from forword_k import forward   # your FK (must use radians)

# ================= READ CSV =================
csv_file = "delta_joint_log.csv"

joint_traj = []

with open(csv_file, newline='') as file:
    reader = csv.DictReader(file)
    for row in reader:
        joint_traj.append([
            float(row["joint_1"]),
            float(row["joint_2"]),
            float(row["joint_3"])
        ])

joint_traj = np.array(joint_traj)
num_frames = len(joint_traj)

print("Frames:", num_frames)

# ================= FIGURE =================
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim(-0.2, 0.2)
ax.set_ylim(-0.2, 0.2)
ax.set_zlim(-0.4, 0.05)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Forward Kinematics Path")

# Moving point
point, = ax.plot([], [], [], 'ro')

# Path trail
path_x, path_y, path_z = [], [], []
trail, = ax.plot([], [], [], 'b-')

# ================= UPDATE =================
def update(frame):

    theta = joint_traj[frame]  # radians

    ee = forward(*theta)
    if ee == -1:
        return

    x, y, z = ee

    # update moving point
    point.set_data([x], [y])
    point.set_3d_properties([z])

    # store trajectory
    path_x.append(x)
    path_y.append(y)
    path_z.append(z)

    trail.set_data(path_x, path_y)
    trail.set_3d_properties(path_z)

    return point, trail

# ================= ANIMATION =================
ani = FuncAnimation(fig, update, frames=num_frames, interval=30)
plt.show()