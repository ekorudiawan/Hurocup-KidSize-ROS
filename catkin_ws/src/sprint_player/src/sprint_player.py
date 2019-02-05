#! /usr/bin/env python
import numpy as np
import math
import rospy
import roslib
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64, Int32, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
global marker_x
marker_x = -1
global marker_y
marker_y = -1
def marker_posx_callback(posx):
    global marker_x, marker_y
    marker_x = posx
    print("Position "+str(marker_x)+" "+str(marker_y))

def marker_posy_callback(posy):
    global marker_x, marker_y
    marker_y = posy
    print("Position "+str(marker_x)+" "+str(marker_y))

def main():
    rospy.loginfo("Sprint Player - Running")
    rospy.init_node("sprint_player")
    motion_vel_pub = rospy.Publisher("/motion/cmd_vel", Twist, queue_size=1)
    motion_state_pub = rospy.Publisher("/motion/state", String, queue_size=1)
    gripper_state_pub = rospy.Publisher("/gripper/state", Bool, queue_size=1)
    head_pan_pub = rospy.Publisher("/head/pan", Float64, queue_size=1)
    head_tilt_pub = rospy.Publisher("/head/tilt", Float64, queue_size=1)
    marker_posx_sub = rospy.Subscriber("/sprint/qrcode/pos_x", Int32, marker_posx_callback)
    marker_posy_sub = rospy.Subscriber("/sprint/qrcode/pos_y", Int32, marker_posy_callback)
    #rospy.spin()
    rate = rospy.spin()
    while not rospy.is_shutdown():
	print("Position "+str(marker_x)+" "+str(marker_y))
        #rate.sleep()
    rospy.loginfo("Sprint Player - Shut Down")

if __name__ == "__main__":
    main()
