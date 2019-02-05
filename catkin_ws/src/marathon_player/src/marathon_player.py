#! /usr/bin/env python
import numpy as np
import math
import rospy
import roslib
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Float64, Int32, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
global line_angle
line_angle = -888
global marker
marker = "blank"

def line_angle_callback(angle_msg):
    global line_angle
    line_angle = angle_msg
    print("Line angle : "+str(line_angle))

def marker_result_callback(marker_msg):
    global marker
    marker = marker_msg
    print("Marker : "+ marker)

def main():
    rospy.loginfo("Marathon Player - Running")
    rospy.init_node("marathon_player")
    motion_vel_pub = rospy.Publisher("/motion/cmd_vel", Twist, queue_size=1)
    motion_state_pub = rospy.Publisher("/motion/state", String, queue_size=1)
    gripper_state_pub = rospy.Publisher("/gripper/state", Bool, queue_size=1)
    head_pan_pub = rospy.Publisher("/head/pan", Float64, queue_size=1)
    head_tilt_pub = rospy.Publisher("/head/tilt", Float64, queue_size=1)
    line_angle_sub = rospy.Subscriber("/marathon/line/angle", Float32, line_angle_callback)
    marker_result_sub = rospy.Subscriber("/marathon/marker/result", String, marker_result_callback)
    rate = rospy.spin()
    rospy.loginfo("Marathon Player - Shut Down")

if __name__ == "__main__":
    main()
