from modules.Trajectory import Trajectory
from modules.MotorController import MotorController
import modules.modern_robotics as mr

import numpy as np

a = Trajectory(1,[4,5,6])

rot_camera_to_90 = mr.RollPitchYawToRot(0,0,np.pi/4)
transf_camera_to_90 = mr.RpToTrans(rot_camera_to_90, [0,0,0])

test_transf = np.array([[1,0,0,0],\
                        [0,1,0,0.5],\
                        [0,0,1,1],\
                        [0,0,0,1]])

print(transf_camera_to_90 @ test_transf)

transf_camera_to_base = np.array([[0, 0,1,  0.0175],\
                                  [0,-1,0, 0.06474],\
                                  [1, 0,0,-0.07274],\
                                  [0, 0,0,       1]])

print(np.dot(transf_camera_to_base, np.dot(transf_camera_to_90, test_transf)))
print(transf_camera_to_base @ test_transf)