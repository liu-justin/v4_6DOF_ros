#!/usr/bin/env python3
import rospy
import v4_6dof.msg as msg
from . import modern_robotics as mr
from . import unpack as unp
import time

class MotorController():
    def __init__(self):

        self.vel_six = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.pos_six = [0.0,0.0,0.0,0.0,0.0,0.0]

        self.M_rest, self.T_list, self.body_list, self.G_list = unp.unpack_XML("/home/justin/catkin_ws/src/v4_6dof/scripts/constants/6DoF_URDF.xml")

        self.M_current = mr.FKinBody(self.M_rest, self.body_list, self.pos_six)

        self.pub = rospy.Publisher('vel_six_chatter',msg.VelGap,queue_size=10)
        self.sub = rospy.Subscriber('pos_six_chatter',msg.VelGap,self.updatePos)

    def updatePos(self, new_pos_six):
        self.pos_six = new_pos_six
        self.M_current = mr.FKinBody(self.M_rest, self.body_list, self.pos_six)

    def addPos(self, new_pos_six):
        self.pos_six = [a + b for a, b in zip(new_pos_six, self.pos_six)]
        self.M_current = mr.FKinBody(self.M_rest, self.body_list, self.pos_six)

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

    # publish a move to a new transformation matrix
    def transfMatrixCartesianPublish(self, new_transf, total_time):
        if total_time < 0.5: total_points = 10
        elif total_time < 1: total_points = int(20*total_time)
        else: total_points = 20
        gap_btwn_points = total_time/(total_points-1)
        point_transf_list = mr.CartesianTrajectory(self.M_current, new_transf, total_time, total_points, 3)

        # transforming set of transformations matrices to set of pos 6vectors
        point_pos_list = []
        previous_point_pos = self.pos_six
        for point_transf in point_transf_list[1:]:
            current_point_pos, success = mr.IKinBody(self.body_list, self.M_current, point_transf, previous_point_pos, 0.01, 0.001)
            if not success:
                print("IK failed, returning")
                return
            point_pos_list.append(current_point_pos)
            previous_point_pos = current_point_pos
        
        print(self.pos_six)
        print(point_pos_list)
        self.trajectoryPublish(point_pos_list, 10*gap_btwn_points)
        self.updatePos(point_pos_list[-1])
        
    def transfMatrixJointPublish(self, new_transf, total_time):
        ending_pos_six, success = mr.IKinBody(self.body_list, self.M_current, new_transf,self.pos_six, 0.01, 0.001) 
        print(f"starting angles: {self.pos_six}, ending angles: {ending_pos_six}")
        
        if success:
            speeds = [abs((start - end)/total_time) for start, end in zip(self.pos_six, ending_pos_six)]
            if max(speeds) > 0.8:
                print(f"this move is too fast!: the fastest move is {max(speeds)}")
                return
            else:
                print(f"this move is all good! speeds are {speeds}")
                print(f"moving slowly to the intersection point")
                # safety, change time to 5
                self.anglePublish(ending_pos_six, 5, True)
        else:
            print("failed to find IK")
        


    # publish a move to a new set of angles
    def anglePublish(self, final_pos_six, total_time, use_absolute):
        sample_rate = 10
        total_points = sample_rate*total_time
        gap_btwn_points = total_time/(total_points-1)

        starting_pos_six = self.pos_six if use_absolute else [0,0,0,0,0,0]

        points_pos_list = mr.JointTrajectory(starting_pos_six, final_pos_six, total_time, total_points, 3)
        
        self.trajectoryPublish(points_pos_list, gap_btwn_points)
        self.updatePos(final_pos_six)