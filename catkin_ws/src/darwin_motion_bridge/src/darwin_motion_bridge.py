import rospy
import numpy
import socket
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64, Bool

MOTION_IP = "127.0.0.1"
MOTION_PORT = 8080
motion_state = "init"

head_pan = 0.0
head_tilt = 0.0
gripper_state = False

def velocity_callback(msg):
	rospy.loginfo("Message Received")
	rospy.loginfo(msg)
	global sock
	MESSAGE = "walk "+str(msg.linear.x)+" "+str(msg.linear.y)+" "+str(msg.linear.z)
	rospy.loginfo("Message to Darwin Motion Controller")
	rospy.loginfo(MESSAGE)
	sock.sendto(MESSAGE, (MOTION_IP, MOTION_PORT))

def motion_state_callback(msg):
	rospy.loginfo("Motion State Received")
	rospy.loginfo(msg)
	global sock
	MESSAGE = str(msg.data)
	rospy.loginfo("Message to Darwin Motion Controller")
	rospy.loginfo(MESSAGE)
	sock.sendto(MESSAGE, (MOTION_IP, MOTION_PORT))

def gripper_state_callback(msg):
	rospy.loginfo("Gripper State Received")
	rospy.loginfo(msg)
	global sock, gripper_state
	gripper_state = msg.data
	if gripper_state == True:
		MESSAGE = "grip 1"
	else:
		MESSAGE = "grip 0"
	rospy.loginfo("Message to Darwin Motion Controller")
	rospy.loginfo(MESSAGE)
	sock.sendto(MESSAGE, (MOTION_IP, MOTION_PORT))

def head_pan_callback(msg):
	rospy.loginfo("Head Pan Received")
	rospy.loginfo(msg)
	global sock, head_pan, head_tilt
	head_pan = msg.data
	MESSAGE = "head "+str(head_pan)+" "+str(head_tilt)
	rospy.loginfo("Message to Darwin Motion Controller")
	rospy.loginfo(MESSAGE)
	sock.sendto(MESSAGE, (MOTION_IP, MOTION_PORT))

def head_tilt_callback(msg):
	rospy.loginfo("Head Pan Received")
	rospy.loginfo(msg)
	global sock, head_pan, head_tilt
	head_tilt = msg.data
	MESSAGE = "head "+str(head_pan)+" "+str(head_tilt)
	rospy.loginfo("Message to Darwin Motion Controller")
	rospy.loginfo(MESSAGE)
	sock.sendto(MESSAGE, (MOTION_IP, MOTION_PORT))

def main():
	global sock
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	rospy.init_node("darwin_motion_bridge")
	velocity_subscriber = rospy.Subscriber("/cmd_vel", Twist, velocity_callback)
	motion_state_subscriber = rospy.Subscriber("/motion_state", String, motion_state_callback)
	gripper_state_subscriber = rospy.Subscriber("/gripper_state", Bool, gripper_state_callback)
	head_pan_subscriber = rospy.Subscriber("/head_pan", Float64, head_pan_callback)
	head_tilt_subscriber = rospy.Subscriber("/head_tilt", Float64, head_tilt_callback)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shut Down")

if __name__ == "__main__":
	main()
