#! /usr/bin/env python
import os
import rospy
import numpy
import socket
import serial
import subprocess

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Float32, Bool, Int32MultiArray, Float32MultiArray
from std_srvs.srv import Trigger, TriggerResponse

UDP_IP = "127.0.0.1"
MOTION_PORT = 8080
HEAD_PORT = 8081
BUTTON_PORT = 8082
motion_state = "init"

kill = "killall screen;"
# cmd = "cd ~/Guai-Guai/Player; screen -S dcm lua run_dcm.lua; screen -S player lua walk_server.lua;"
cmd = "cd ~/UPennalizers-master/Player; screen -S dcm lua run_dcm.lua; screen -S player lua walk_server.lua;"

gripperState = False

def velocity_callback(msg):
	rospy.loginfo("Message Received")
	rospy.loginfo(msg)
	global sock_motion
	MESSAGE = "walk " + str(msg.linear.x) + " " + str(msg.linear.y) + " " + str(msg.linear.z)
	rospy.loginfo("Message to Darwin Motion Controller")
	rospy.loginfo(MESSAGE)
	sock_motion.sendto(MESSAGE, (UDP_IP, MOTION_PORT))

def motion_state_callback(msg):
	rospy.loginfo("Motion State Received")
	rospy.loginfo(msg)
	global sock_motion
	MESSAGE = str(msg.data)
	rospy.loginfo("Message to Darwin Motion Controller")
	rospy.loginfo(MESSAGE)
	sock_motion.sendto(MESSAGE, (UDP_IP, MOTION_PORT))

def gripper_state_callback(msg):
	rospy.loginfo("Gripper State Received")
	rospy.loginfo(msg)
	global sock_motion
	gripperState = msg.data
	if gripperState == True:
		MESSAGE = "grip 1"
	else:
		MESSAGE = "grip 0"
	rospy.loginfo("Message to Darwin Motion Controller")
	rospy.loginfo(MESSAGE)
	sock_motion.sendto(MESSAGE, (UDP_IP, MOTION_PORT))

def head_callback(pos_msg):
	rospy.loginfo("Head Received")
	rospy.loginfo(pos_msg)
	global sock_motion
	headPan = pos_msg.data[0]
	headTilt = pos_msg.data[1]
	MESSAGE = "head " + str(headPan)+" " + str(headTilt)
	rospy.loginfo("Message to Darwin Motion Controller")
	rospy.loginfo(MESSAGE)
	sock_motion.sendto(MESSAGE, (UDP_IP, HEAD_PORT))

def trigger_response(request):
    return TriggerResponse(success=True, message="controller run")

def sock_motion_Button():
	global sock_button
	rawData = sock_button.recv(1024)
	state = rawData.split(" ")
	buttonState = Int32MultiArray()
	buttonState.data = [int(state[0]), int(state[1])]
	button_pub.publish(buttonState)		

def kill_node():
	global sock_motion, sock_button
	sock_motion.close()
	sock_button.close()
	rospy.signal_shutdown("shutdown time.") 

def main():
	os.system(kill)
	os.system(cmd)
	
	global sock_motion, sock_button
	sock_motion = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock_button = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock_button.bind((UDP_IP, BUTTON_PORT))
	
	rospy.init_node("darwin_motion_bridge")	
	velocity_subscriber = rospy.Subscriber("/motion/cmd_vel", Twist, velocity_callback)
	motion_state_subscriber = rospy.Subscriber("/motion/state", String, motion_state_callback)
	gripper_state_subscriber = rospy.Subscriber("/gripper/state", Bool, gripper_state_callback)
	head_subscriber = rospy.Subscriber("/head/pos", Float32MultiArray, head_callback)
	my_service = rospy.Service('/srv_controller', Trigger, trigger_response)
	global button_pub
	button_pub = rospy.Publisher("/button/state", Int32MultiArray, queue_size=1)

	while not rospy.is_shutdown():
		sock_motion_Button()		
	# rospy.loginfo("Spinning")
	# rospy.spin()
	rospy.on_shutdown(kill_node)

if __name__ == "__main__":
	main()
