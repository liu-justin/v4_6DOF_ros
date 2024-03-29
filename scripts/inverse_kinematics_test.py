import rospy
import numpy as np
from modules.MotorController import MotorController
from modules import modern_robotics as mr

# import v4_6dof.msg as msg

mc = MotorController()
np.set_printoptions(precision=7, suppress=True)
# these_angles = [1*np.pi/2, -1*np.pi/2, 1*np.pi/2, 0*np.pi/2, -1*np.pi/2, 0*np.pi/2]
# bodyM = mr.FKinBody(mc.M_rest, mc.body_list, these_angles)
# spaceM = mr.FKinSpace(mc.M_rest, mc.space_list, these_angles)
# print(spaceM)
# print(bodyM)

rospy.init_node('talker', anonymous=True)

mc.anglePublish([0, -1*np.pi/2, 1*np.pi/2, 0*np.pi/2, -1*np.pi/2, 0*np.pi/2], 0.3, True)
print(mr.FKinSpace(mc.M_rest, mc.space_list, [0, -1*np.pi/2, 1*np.pi/2, 0*np.pi/2, -1*np.pi/2, 0*np.pi/2]))

xyz = [0.129, 0.418, 0.322]
# R = mr.RollPitchYawToRot(np.pi/8, -1*np.pi/7, np.pi/9)
R = mr.RollPitchYawToRot(0,0,np.pi/4)
transf = mr.RpToTrans(R, xyz)
time = 1

print(f"this is goal: {transf}")
mc.transfMatrixAnalyticalPublish(transf, time)
mc.anglePublish([0, -1*np.pi/2, 1*np.pi/2, 0*np.pi/2, -1*np.pi/2, 0*np.pi/2], 0.3, True)
mc.transfMatrixCartesianPublish(transf, time)


#09-08-2021 max speed is 1.1rad/sec for all motors