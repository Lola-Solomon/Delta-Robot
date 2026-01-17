import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import Forword_K
from Forword_K import forward

# ================= CONSTANTS =================
sqrt3 = np.sqrt(3)
pi = np.pi
sin30 = 0.5
tan30 = 1 / sqrt3
tan60 = sqrt3

# ================= GEOMETRY =================
f = 0.439075 / 2     # base triangle side (m)
e = 0.200918 / 2     # end effector triangle side (m)
rf = 0.100           # bicep length (m)
re = 0.200           # forearm length (m)



# ================= FIGURE =================
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim(-0.3, 0.3)
ax.set_ylim(-0.3, 0.3)
ax.set_zlim(-0.6, 0.1)

ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("Delta Robot Simulator")

# ================= BASE TRIANGLE =================
L=f*sqrt3/2
R = f / (sqrt3*2)
A = np.array([-f/2, -R, 0])
B = np.array([ f/2, -R, 0])
C = np.array([ 0,  L-R, 0])

print (A)

base = np.array([A, B, C, A])
ax.plot(base[:,0], base[:,1], base[:,2], 'r', lw=2)

base_pts = np.array([(A+B)/2, (B+C)/2, (C+A)/2])
edges = [B-A, C-B, A-C]

# ================= DRAW OBJECTS =================
upper = [ax.plot([], [], [], 'k', lw=3)[0] for _ in range(3)]
lower = [ax.plot([], [], [], 'b', lw=2)[0] for _ in range(3)]
ee_triangle, = ax.plot([], [], [], 'g', lw=2)
ee_point = ax.scatter([], [], [], c='r', s=40)

printed = False  # print once

# ================= ROTATION ABOUT AXIS =================
def rotate(v, u, theta):
    return (
        v*np.cos(theta)
        + np.cross(u, v)*np.sin(theta)
        + u*np.dot(u, v)*(1 - np.cos(theta))
    )

# ================= EE TRAJECTORY =================
ee_path_x = []
ee_path_y = []
ee_path_z = []
ee_path_line, = ax.plot([], [], [], 'm--', lw=2)  # trajectory line

# ================= ANIMATION =================
def update(frame):
    global printed

    # If theta is constant, the point will not move (trajectory will be a dot).
    # Use a varying theta to SEE the line:
    t = frame * 0.05
    theta = [
        0*np.sin(t),
        0*np.sin(t + 2*pi/3),
        20*np.sin(t + 4*pi/3)
    ]

    ee = forward(*theta)
    if ee is None:
        return

    # store trajectory AFTER ee is valid
    ee_path_x.append(ee[0])
    ee_path_y.append(ee[1])
    ee_path_z.append(ee[2])

    # update trajectory line
    ee_path_line.set_data(ee_path_x, ee_path_y)
    ee_path_line.set_3d_properties(ee_path_z)

    # ---- End-effector triangle (visual scaling trick) ----
    scale = e / f
    center = (A + B + C) / 3

    Aee = ee + scale*(A - center)
    Bee = ee + scale*(B - center)
    Cee = ee + scale*(C - center)

    ee_triangle.set_data(
        [Aee[0], Bee[0], Cee[0], Aee[0]],
        [Aee[1], Bee[1], Cee[1], Aee[1]]
    )
    ee_triangle.set_3d_properties(
        [Aee[2], Bee[2], Cee[2], Aee[2]]
    )

    # ---- EE side midpoints ----
    ee_pts = [
        (Aee + Bee) / 2,
        (Bee + Cee) / 2,
        (Cee + Aee) / 2
    ]

    # ---- Arms ----
    for i in range(3):
        Bp = base_pts[i]
        u = edges[i] / np.linalg.norm(edges[i])

        v0 = Bp / np.linalg.norm(Bp) * rf
        E = Bp + rotate(v0, u, theta[i]*pi/180)

        upper[i].set_data([Bp[0], E[0]], [Bp[1], E[1]])
        upper[i].set_3d_properties([Bp[2], E[2]])

        P = ee_pts[i]
        lower[i].set_data([E[0], P[0]], [E[1], P[1]])
        lower[i].set_3d_properties([E[2], P[2]])

    # ---- Red dot (FK EE) ----
    ee_point._offsets3d = ([ee[0]], [ee[1]], [ee[2]])

    if not printed:
        print("FK End Effector (first frame):", ee)
        printed = True

ani = FuncAnimation(fig, update, frames=400, interval=30)
plt.show()
