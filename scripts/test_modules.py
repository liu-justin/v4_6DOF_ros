from modules.Trajectory import Trajectory
import modules.modern_robotics as mr

import numpy as np

a = Trajectory(1,[4,5,6])

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