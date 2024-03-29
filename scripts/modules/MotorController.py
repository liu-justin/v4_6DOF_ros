#!/usr/bin/env python3
import rospy
import v4_6dof.msg as msg
import numpy as np
import untangle
from . import modern_robotics as mr
from . import unpack as unp
import time
import math

def pi_wrap(x):
    return (x-np.pi) % (2*np.pi) - np.pi

class MotorController():
    def __init__(self):

        self.vel_six = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.pos_six = [0.0,0.0,0.0,0.0,0.0,0.0]

        self.unpack_XML("/home/justin/catkin_ws/src/v4_6dof/scripts/constants/6DoF_URDF.xml")

        self.M_current = mr.FKinSpace(self.M_rest, self.space_list, self.pos_six)

        self.pub = rospy.Publisher('vel_six_chatter',msg.VelGap,queue_size=10)
        self.sub = rospy.Subscriber('pos_six_chatter',msg.VelGap,self.updatePos)

    def updatePos(self, new_pos_six):
        self.pos_six = new_pos_six
        self.M_current = mr.FKinSpace(self.M_rest, self.space_list, self.pos_six)

    def addPos(self, new_pos_six):
        self.pos_six = [a + b for a, b in zip(new_pos_six, self.pos_six)]
        self.M_current = mr.FKinSpace(self.M_rest, self.space_list, self.pos_six)

    def updateVelGap(self, new_vel_six, time):       
        self.vel_six = new_vel_six
        arg = msg.VelGap()
        arg.vel = list(self.vel_six)
        arg.gap = time
        self.pub.publish(arg)

    # sending the specified list of angle_six at the specified timing
    def trajectoryPublish(self, angle_six_list, time_gap):
        # publishing to Teensy, need time in micros
        time_gap_micros = int(1000000*time_gap)
        previousTime = time.perf_counter()

        # sending two right here to fill the buffer at the Arduino
        self.updateVelGap(angle_six_list[1]-angle_six_list[0], time_gap_micros)
        self.updateVelGap(angle_six_list[2]-angle_six_list[1], time_gap_micros)
        i = 3
        while i < len(angle_six_list):
            currentTime = time.perf_counter()
            if currentTime - previousTime > time_gap:
                previousTime = currentTime
                angular_velocity_six = (angle_six_list[i] - angle_six_list[i-1])/time_gap
                self.updateVelGap(angular_velocity_six, time_gap_micros)
                i += 1
        self.updateVelGap([0,0,0,0,0,0], time_gap_micros)

    def transfMatrixAnalyticalPublish(self, new_transf, total_time):
        """
        Uses an analytical approach to get first 3 angles to line up center of differential gears to point, 
        then use IK to finish up the rest of angles
        """
        np.set_printoptions(precision=7, suppress=True)
        R_0n,p = mr.TransToRp(new_transf)
        R1_angle = -1*math.atan(p[2]/p[0])

        # analytical inverse kinematics https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
        a1 = 0.25
        a2 = 0.1909 #math.sqrt(0.06825**2 +0.1783**2)
        x = math.sqrt(p[2]**2 +p[0]**2)
        y = p[1] - 0.02463 - 0.155582
        q2 = -1*math.acos((x**2 +y**2 -a1**2 -a2**2)/(2*a1*a2))
        q1 = math.atan(y/x) - math.atan(a2*math.sin(q2)/(a1+a2*math.cos(q2)))

        T1_angle = -1*math.pi + q1
        T2_angle = math.pi - math.atan(0.06825/0.1783) + q2
        
        current_guess = [R1_angle, T1_angle, T2_angle,0,0,0]
        R_0ee, p_0ee = mr.TransToRp(mr.FKinSpace(self.M_rest, self.space_list, current_guess))
        R_ee_n = mr.RotInv(R_0ee) @ R_0n

        # converting R_ee_n into XZX angles
        if (R_ee_n[0][0] < 1):
            if (R_ee_n[0][0] > -1):
                thetaZ = -1*math.acos(R_ee_n[0][0])
                thetaX0 = -1*math.atan2(-1*R_ee_n[2][0],-1*R_ee_n[1][0])
                thetaX1 = -1*math.atan2(-1*R_ee_n[0][2],1*R_ee_n[0][1])
            else:
                print("irregular")
                thetaZ = np.pi
                thetaX0 = -1*math.atan2(R_ee_n[2][1],R_ee_n[2][2])
                thetaX1 = 0
        else:
            print("double irregular")
            thetaZ = 0
            thetaX0 = math.atan2(R_ee_n[2][1],R_ee_n[2][2])
            thetaX1 = 0
        
        current_guess = np.array([R1_angle, T1_angle, T2_angle, thetaX0, thetaZ, thetaX1])

        print(f"before IK, then angle are: {current_guess}")
        print(mr.FKinSpace(self.M_rest, self.space_list, current_guess))

        # trying to align xaxis of end effector with inverse of velocity vector
        ending_pos_six, success = mr.IKinSpace(self.space_list, self.M_rest, new_transf, current_guess, 0.01, 0.001)

        ending_pos_six[3] = pi_wrap(ending_pos_six[3])
        ending_pos_six[5] = pi_wrap(ending_pos_six[5])
        print(f"after IK and wrapping, the angles are: {ending_pos_six}")
        print(mr.FKinSpace(self.M_rest, self.space_list, ending_pos_six))
        
        if not success:
            print(f"failed IK")    
        # checking joint limits
        elif not all([current_angle > lower and current_angle < upper for current_angle, lower, upper in \
            zip(ending_pos_six, self.limit_list_lower, self.limit_list_upper)]):
            print(f"one of the angles is past the angle limits, which are displayed here: \n")
            print(f"{self.limit_list_lower}\n{ending_pos_six}\n{self.limit_list_upper}")
            return
        else:
            # checking speeds against max speeds
            speeds = [abs((start - end)/total_time) for start, end in zip(self.pos_six, ending_pos_six)]
            if not all([speed < max_speed for speed,max_speed in zip(speeds, self.vel_limits)]):
                print(f"this move is too fast!: the speeds are {speeds}")
                return
            else:
                print(f"this move is all good! speeds are {speeds}")
                print(f"moving to the intersection point")
                self.anglePublish(ending_pos_six, total_time, True)
            
    # publish a move to a new transformation matrix
    def transfMatrixCartesianPublish(self, new_transf, total_time):
        if total_time < 0.5: total_points = 20
        elif total_time < 1: total_points = int(30*total_time)
        else: total_points = 30
        gap_btwn_points = total_time/(total_points-1)
        point_transf_list = mr.CartesianTrajectory(self.M_current, new_transf, total_time, total_points, 3)

        # transforming set of transformations matrices to set of pos 6vectors
        point_pos_list = []
        previous_point_pos = self.pos_six
        print(f"first anglelist: {previous_point_pos}")
        for point_transf in point_transf_list[1:]:
            current_point_pos, success = mr.IKinBody(self.body_list, self.M_rest, point_transf, previous_point_pos, 0.01, 0.001)
            print(f"this is the current anglelist: {current_point_pos}")
            if not success:
                print(f"IK failed, returning")
                return
            elif not all([current_angle > lower and current_angle < upper for current_angle, lower, upper in \
            zip(current_point_pos, self.limit_list_lower, self.limit_list_upper)]):
                print(f"one of the angles is past the angle limits")
                print(current_point_pos)
                print(f"{self.limit_list_lower}{self.limit_list_upper}")
                return
            else:
                current_point_pos[0] *= -1
                point_pos_list.append(current_point_pos)
                current_point_pos[0] *= -1
                previous_point_pos = current_point_pos
        
        a = input("Move slowly to the point")
        self.trajectoryPublish(point_pos_list, 5*gap_btwn_points)
        self.updatePos(point_pos_list[-1])
        
    def transfMatrixJointPublish(self, new_transf, total_time):
        ending_pos_six, success = mr.IKinBody(self.body_list, self.M_rest, new_transf,self.pos_six, 0.01, 0.001) 
        print(f"starting angles: {self.pos_six}, ending angles: {ending_pos_six}")

        if not all([current_angle > lower and current_angle < upper for current_angle, lower, upper in \
            zip(ending_pos_six, self.limit_list_lower, self.limit_list_upper)]):
            print(f"one of the angles is past the angle limits")
            print(ending_pos_six)
            print(f"{self.limit_list_lower}\n{self.limit_list_upper}")
            return
        
        elif success:
            speeds = [abs((start - end)/total_time) for start, end in zip(self.pos_six, ending_pos_six)]
            if max(speeds) > 1.1:
                print(f"this move is too fast!: the speeds are {speeds}")
                return
            else:
                print(f"this move is all good! speeds are {speeds}")
                print(f"moving slowly to the intersection point")
                # safety, change time to 5
                self.anglePublish(ending_pos_six, total_time*3, True)
        else:
            print("failed to find IK")
        
    # publish a move to a new set of angles
    def anglePublish(self, final_pos_six, total_time, use_absolute):
        sample_rate = 120
        total_points = sample_rate*total_time
        gap_btwn_points = total_time/(total_points-1)

        starting_pos_six = self.pos_six if use_absolute else [0,0,0,0,0,0]

        # COULDNT FIGURE OUT HOW TO REVERSE MOTOR DIRECTION USING URDF
        final_pos_six[0] *= -1

        points_pos_list = mr.JointTrajectory(starting_pos_six, final_pos_six, total_time, total_points, 3)
        
        self.trajectoryPublish(points_pos_list, gap_btwn_points)
        final_pos_six[0] *= -1
        self.updatePos(final_pos_six)

    def unpack_XML(self, xml):
        obj = untangle.parse(xml)
        np.set_printoptions(precision=7, suppress=True)

        # initialize T_list, body_list
        self.T_list = []
        self.body_list = np.array([0,0,0,0,0,0])
        self.space_list = np.array([0,0,0,0,0,0])
        self.limit_list_lower = []
        self.limit_list_upper = []
        self.vel_limits = [0.0,0.0,0.0,0.0,0.0,0.0]
        # skips all joints that are type fixed (like the base link and ee_link)
        joint_list = [joint for joint in obj.robot.joint if joint["type"]!="fixed"]

        # getting the space axes
        T_added = mr.rpyxyzToTrans([0,0,0,0,0,0])
        for joint in joint_list:
            rpy = [float(n) for n in joint.origin["rpy"].split()]
            R = mr.RollPitchYawToRot(rpy[0], rpy[1], rpy[2])
            p = np.array([float(n) for n in joint.origin["xyz"].split()])
            T = mr.RpToTrans(R,p)
            T_added = T_added @ T
            R_added, p_added = mr.TransToRp(T_added)

            # find which axis the motor at this joint turns about
            omega_n = [float(n) for n in joint.axis["xyz"].split()]
            omega_0 = np.dot(R_added, omega_n)
            omega_0_skewed = mr.VecToso3(omega_0)

            # negative 1 to invert p_added(p_0n -> p_n0)
            v_0 = -1*np.dot(omega_0_skewed, p_added)

            # combine w,v into body_axis, then insert into body_list
            space_axis = np.r_[omega_0, v_0]
            self.space_list = np.c_[self.space_list, space_axis]

        self.space_list = np.delete(self.space_list, 0,1)
        

        # grabbing the last joint (ee_joint) and its xyz
        rpy_ee = [float(n) for n in obj.robot.joint[-1].origin["rpy"].split()]
        R_ee = mr.RollPitchYawToRot(rpy_ee[0],rpy_ee[1],rpy_ee[2])
        p_ee = [float(n) for n in obj.robot.joint[-1].origin["xyz"].split()]
        T_ee = mr.RpToTrans(R_ee, p_ee)
        self.M_rest = T_added @ T_ee
        T_subtracted = T_ee

        for joint in reversed(joint_list):
            lower_limit = float(joint.limit["lower"])
            upper_limit = float(joint.limit["upper"])
            self.limit_list_lower.insert(0,lower_limit)
            self.limit_list_upper.insert(0,upper_limit)
            self.vel_limits.insert(0,float(joint.limit["velocity"]))

            R_subtracted, p_subtracted = mr.TransToRp(mr.TransInv(T_subtracted))

            omega_n = [float(n) for n in joint.axis["xyz"].split()]
            omega_ee = np.dot(mr.RotInv(R_subtracted), omega_n)
            omega_ee_skewed = mr.VecToso3(omega_ee)
            v = -1 * np.dot(omega_ee_skewed, p_subtracted)

            body_axis = np.r_[omega_ee, v]
            self.body_list = np.c_[body_axis, self.body_list]

            # find the roll-pitch-yaw, about the z-y-x axes of the previous joint
            rpy = [float(n) for n in joint.origin["rpy"].split()]
            R = mr.RollPitchYawToRot(rpy[0], rpy[1], rpy[2])
            p = np.array([float(n) for n in joint.origin["xyz"].split()])
            T = mr.RpToTrans(R,p)
            self.T_list.insert(0,T)

            T_subtracted = T @ T_subtracted

        # remove the filler column needed to start appending
        self.body_list = np.delete(self.body_list, len(self.body_list[0])-1,1)

        ##### inverse dynamics #####
        # need G_list, or spatial inertai matrix list
        #   6x6 matrix, top left corner is 3x3 rotational inertia matrix, bottom right is mass of link * identity matrix
        self.G_list = []

        # urdf file has world link and ee_link, so that all joints have a parent and child
        # also we dont need the base link and link from joint 6 to ee_link called link6 (remember 6 joints should only have 5 links)
        # so for this for loop, skip the first two and last two
        for link in obj.robot.link[2:-2]: 
            mass = float(link.inertial.mass["value"])

            # translate from parent joint to CoM 
            # negative one b/c this is from parent link origin to CoM, but I need CoM to parent link origin
            xyz_CoM= -1*np.array([float(n) for n in link.inertial.origin["xyz"].split()])

            # grab Ixx, Ixy, Ixz, Iyy, Iyz, Izz about the CoM, with the parent link coordinate systems
            inertia_values_CoM = [float(n) for n in (vars(link.inertial.inertia_CoM)["_attributes"].values())]
            
            # putting those values into a rotational inertia matrix, centered at CoM, using parent link coords
            I_CoM = np.array([[inertia_values_CoM[0], inertia_values_CoM[1], inertia_values_CoM[2]],
                            [inertia_values_CoM[1], inertia_values_CoM[3], inertia_values_CoM[4]],
                            [inertia_values_CoM[2], inertia_values_CoM[4], inertia_values_CoM[5]]])

            # grabbing the eigenvectors of the rotational inertia matrix, to find the principle axes of inertia
            w,v = np.linalg.eig(I_CoM) 
            # rotational inertia matrix, centered at CoM, aligned w/ principle axes of inertia
            rotated_I_CoM = np.transpose(v) @ I_CoM @ v

            # rotational inertia matrix, centered at parent link origin, aligned w/ parent link origin coords
            translated_T_CoM = I_CoM + mass*(np.inner(xyz_CoM, xyz_CoM)*np.identity(3) - np.outer(xyz_CoM, xyz_CoM))

            mI = mass*np.identity(3)
            zeros = np.zeros((3,3))
            Gi = np.c_[np.r_[rotated_I_CoM, zeros], np.r_[zeros,mI]]
            self.G_list.append(Gi)

        