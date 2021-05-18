from modules.Trajectory import Trajectory
import modules.modern_robotics as mr

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.axes.set_xlim3d(left=-2, right=2)
ax.axes.set_ylim3d(bottom=-2, top=2)
ax.axes.set_zlim3d(bottom=-2, top=2)

traj = Trajectory(0,[1.5,0.5,0])
traj.append(0.1, [1,0.5019, 0.1 ])
traj.append(0.2, [0.5,0.30706, 0.2 ])

possible, intersection_point, time_until_intersection = traj.checkSphereIntersection([0,0,0], 0.4087037)
if possible:
    print(f"point: {intersection_point} time: {time_until_intersection}")
    intersection_transf = mr.RpToTrans(np.identity(3), intersection_point)
    ax.scatter(0,0.180212,0)
    ax.scatter(intersection_point[0], intersection_point[1], intersection_point[2])
    u,v = np.mgrid[0:2*np.pi:40j, 0:np.pi:20j]
    x = np.cos(u)*np.sin(v)*0.4087037
    y = np.sin(u)*np.sin(v)*0.4087037
    z = np.cos(v)*0.4087037
    ax.plot_wireframe(x, y, z, color="y")
    t = np.arange(0,2,0.01)
    x_traj = traj.betas["x"][0] + traj.betas["x"][1]*t + traj.betas["x"][2]*(t**2)
    y_traj = traj.betas["y"][0] + traj.betas["y"][1]*t + traj.betas["y"][2]*(t**2)
    z_traj = traj.betas["z"][0] + traj.betas["z"][1]*t + traj.betas["z"][2]*(t**2)
    ax.plot3D(x_traj, y_traj, z_traj, color="r")
    plt.show()

test_transf = np.array([[1,0,0,0],\
                        [0,1,0,0.5],\
                        [0,0,1,1],\
                        [0,0,0,1]])

test_point = np.array([0,0.1,0.3,1])

rot_camera_to_90 = mr.RollPitchYawToRot(0,0,np.pi/4)
transf_camera_to_90 = mr.RpToTrans(rot_camera_to_90, [0,0,0])

transf_90_to_base = np.array([[0, 0,1, 0.07274],\
                                  [0,-1,0, 0.06474],\
                                  [1, 0,0, -0.0175],\
                                  [0, 0,0,       1]])

transf_camera_to_base = transf_90_to_base @ transf_camera_to_90

print(transf_90_to_base @ (transf_camera_to_90 @ test_point))

print(transf_camera_to_base @ test_point)