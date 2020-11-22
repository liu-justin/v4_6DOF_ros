#!/usr/bin/env python

import rospy
import std_msgs.msg as m

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + f"I heard {data.data}")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter_motorR1", m.Float32, callback)
    rospy.Subscriber("chatter_motorT1", m.Float32, callback)
    rospy.Subscriber("chatter_motorT2", m.Float32, callback)
    rospy.Subscriber("chatter_motorR2", m.Float32, callback)
    rospy.Subscriber("chatter_motorT3", m.Float32, callback)
    rospy.Subscriber("chatter_motorR3", m.Float32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()