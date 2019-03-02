#! /usr/bin/env python
import numpy as np
import math
import time
import rospy
import roslib
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32, Bool, Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

frame_w = rospy.get_param("/usb_cam/image_width")
frame_h = rospy.get_param("/usb_cam/image_height")

ball_x = -1
ball_y = -1
ball_size = -1

def ball_pos_callback(pos_msg):
	global ball_x, ball_y
	ball_x = pos_msg.data[0]
	ball_y = pos_msg.data[1]

button = [0, 0]

def button_callback(pos_msg):
	global button
	button[0] = pos_msg.data[0]
	button[1] = pos_msg.data[1]

magneto = 0

def compass_callback(cmps_msg):
	global magneto
	magneto = cmps_msg.data


count_ball_loss  = 0

def ball_lost(threshold):
	global count_ball_loss
	if ball_x == -1 and ball_y == -1:		
		count_ball_loss += 1
		if count_ball_loss >= threshold :
			return True
	else :
		count_ball_loss = 0
		return False

def head_move(head_pan, head_tilt):
	global pos_pan, pos_tilt
	pos_pan = head_pan
	pos_tilt = head_tilt
	head_pos = Float32MultiArray()
	head_pos.data = [pos_pan, pos_tilt]
	head_pub.publish(head_pos)

def walk(x, y, a):
	velocity = Twist()
	velocity.linear.x = x
	velocity.linear.y = y
	velocity.linear.z = a
	motion_vel_pub.publish(velocity)

pos_pan = 0.0
pos_tilt = 0.0
pan_min = -1.5
pan_max = 1.5
tilt_min = -1.3
tilt_max = 0.0

def head_limit(pos_pan, pos_tilt):
	if pos_pan <= pan_min :		
		pos_pan = pan_min
	elif pos_pan >= pan_max :		
		pos_pan = pan_max
	if pos_tilt <= tilt_min :		
		pos_tilt = tilt_min
	elif pos_tilt >= tilt_max :	
		pos_tilt = tilt_max
	head_pos = Float32MultiArray()
	head_pos.data = [pos_pan, pos_tilt]
	return head_pos

pan_step = 0.0
tilt_step = 0.0
move_pan = True

def scan_ball(mode):
	global pos_pan, pos_tilt, pan_step, tilt_step, move_pan

	if pan_step > 0:		
		pan_step = rospy.get_param("/united_soccer_params/Pan_Step")
	else:				
		pan_step = rospy.get_param("/united_soccer_params/Pan_Step") * -1
	if tilt_step > 0:	
		tilt_step = rospy.get_param("/united_soccer_params/Tilt_Step") 
	else:				
		tilt_step = rospy.get_param("/united_soccer_params/Tilt_Step") * -1
	
	if mode == 0: # normal
		pos_pan += pan_step
		if pos_pan >= pan_max or pos_pan <= pan_min:
			pan_step *= -1
			pos_tilt += tilt_step
			if pos_tilt >= tilt_max or pos_tilt <= tilt_min:
				tilt_step *= -1 

	elif mode == 1: # only tilt
		pos_pan = 0.0
		pos_tilt += tilt_step
		if pos_tilt >= tilt_max or pos_tilt <= tilt_min:
			tilt_step *= -1 

	elif mode == 2: # rectangle
		if move_pan: 
			pos_pan += pan_step
			if pos_pan >= pan_max or pos_pan <= pan_min:
				pan_step *= -1
				move_pan = False
		else:
			pos_tilt += tilt_step
			if pos_tilt >= tilt_max or pos_tilt <= tilt_min:
				tilt_step *= -1
				move_pan = True	

	head_pos = head_limit(pos_pan, round(pos_tilt, 3))
	pos_pan, pos_tilt = head_pos.data
	head_pub.publish(head_pos)

sum_err_pan = 0
sum_err_tilt = 0
last_error_x = 0
last_error_y = 0

def head_track_ball():
	global pos_pan, pos_tilt, sum_err_pan, sum_err_tilt, last_error_x, last_error_y
	global freq
	dt = 1.0 / float(freq)
	KP_pan = rospy.get_param("/united_soccer_params/Pan_KP")
	KI_pan = rospy.get_param("/united_soccer_params/Pan_KI")
	KD_pan = rospy.get_param("/united_soccer_params/Pan_KD")
	KP_tilt = rospy.get_param("/united_soccer_params/Tilt_KP")
	KI_tilt = rospy.get_param("/united_soccer_params/Tilt_KI")
	KD_tilt = rospy.get_param("/united_soccer_params/Tilt_KD")
	if ball_x != -1 and ball_y != -1:	
		error_x = (frame_w/2) - ball_x
		error_x *= 77.32 / frame_w
		error_x = (error_x * math.pi)/ 180
		error_x_diff = error_x - last_error_x

		P_pan  = last_error_x * KP_pan
		sum_err_pan += error_x * dt
		I_pan = sum_err_pan * KI_pan
		deriv_err_pan = error_x_diff / dt
		D_pan = deriv_err_pan * KD_pan
		last_error_x = error_x
		pos_pan += (P_pan + I_pan + D_pan)

		error_y = (frame_h/2) - ball_y
		error_y *= -1
		error_y *= 61.93 / frame_h
		error_y = (error_y * math.pi) /180
		error_y_diff = error_y - last_error_y

		P_tilt  = last_error_y * KP_tilt
		sum_err_tilt += error_y * dt
		I_tilt = sum_err_tilt * KI_tilt
		deriv_err_tilt = sum_err_tilt / dt
		D_tilt = deriv_err_tilt * KD_tilt
		last_error_y = error_y
		pos_tilt += (P_tilt + I_tilt + D_tilt)

		head_pos = head_limit(pos_pan, round(pos_tilt, 2))
		pos_pan, pos_tilt = head_pos.data		
		head_pub.publish(head_pos)

def body_track_ball():
	global pos_pan, pos_tilt
	KP_body = rospy.get_param("/united_soccer_params/Body_KP")
	if ball_x != -1 and ball_y != -1:	 
		error_body_a = pos_pan - 0
	else:
		error_body_a = 0

	max_walk_a = 0.4	
	body_a = error_body_a * KP_body	

	if body_a >= max_walk_a:
		body_a = max_walk_a
	elif body_a <= -max_walk_a:
		body_a = -max_walk_a
	body_a = round(body_a, 2)
	return body_a

ball_pos = False
px_ball_pos = 0.00
py_ball_pos = 0.00
def ball_positioning(setPoint_X, setPoint_Y, speed=0.10):
	global pos_pan, pos_tilt, ball_pos, px_ball_pos, py_ball_pos
	errorPos_X = pos_pan - setPoint_X
	errorPos_Y = pos_tilt - setPoint_Y

	KP_ball_positioning_y = rospy.get_param("/united_soccer_params/KP_Ball_Pos_Y")
	# print("KP_BALL_POS", KP_ball_positioning_y)

	if (errorPos_X > -0.10 and errorPos_X < 0.10) and (errorPos_Y > -0.10):
		px_ball_pos = 0.00
		py_ball_pos = 0.00
		ball_pos = True
	else:
		ball_pos = False
		if (pos_pan >= 1.0 and pos_tilt >= -1.2) or (pos_pan <= -1.0 and pos_tilt >= -1.2): # bola disamping | pan tilt kircok (polar)
			px_ball_pos = -0.03
			py_ball_pos = errorPos_X * KP_ball_positioning_y
		else:
			# X Move
			if pos_tilt > setPoint_Y:
				px_ball_pos = -0.03
			elif pos_tilt >= (setPoint_Y - 0.1) and pos_tilt <= setPoint_Y:
				px_ball_pos = 0.00
			elif pos_tilt >= (setPoint_Y - 0.3) and pos_tilt < (setPoint_Y - 0.1):
				px_ball_pos = errorPos_Y * -speed
				if px_ball_pos >= 0.02:
					px_ball_pos = 0.02
				elif px_ball_pos <= 0.00:
					px_ball_pos = 0.00
			else: #bola masih jauh
				px_ball_pos = pos_tilt * (0.08 / -1.6)

			# Y Move
			if pos_pan >= (setPoint_X - 0.1) and pos_pan <= (setPoint_X + 0.1):
				py_ball_pos = 0.00
			else: # belum dalam range
				py_ball_pos = errorPos_X * KP_ball_positioning_y

	walk(round(px_ball_pos, 3), round(py_ball_pos,3), 0.0)
	# walk(0.00,0.00,0.00)

def ball_positioning2(setPoint_X, setPoint_Y, speed=0.10):
	global pos_pan, pos_tilt, ball_pos, px_ball_pos, py_ball_pos
	errorPos_X = pos_pan - setPoint_X
	errorPos_Y = pos_tilt - setPoint_Y
	# print("error", errorPos_X, errorPos_Y)
	KP_ball_positioning_x = rospy.get_param("/united_soccer_params/KP_Ball_Pos_X")
	KP_ball_positioning_y = rospy.get_param("/united_soccer_params/KP_Ball_Pos_Y")
	if (errorPos_X > -0.08 and errorPos_X < 0.08 and errorPos_Y > -0.10):
		ball_pos = True
	elif (pos_pan > 1.35 and pos_pan < 1.35):
		py_ball_pos = errorPos_X * KP_ball_positioning_x
		px_ball_pos = -errorPos_Y * KP_ball_positioning_y
		if py_ball_pos >= 0.03:
			py_ball_pos = 0.03
		if py_ball_pos <= -0.03:
			py_ball_pos = -0.03
		walk(-0.015, round(py_ball_pos,3), 0.0)
		ball_pos = False
	else:
		py_ball_pos = errorPos_X * KP_ball_positioning_x
		px_ball_pos = -errorPos_Y * KP_ball_positioning_y
		if px_ball_pos >= 0.04:
			px_ball_pos = 0.04
		if py_ball_pos >= 0.03:
			py_ball_pos = 0.03
		if py_ball_pos <= -0.03:
			py_ball_pos = -0.03
		walk(round(px_ball_pos, 3), round(py_ball_pos,3), 0.0)
		ball_pos = False


def kick(): 
	global pos_pan, pos_tilt, ball_pos

	pPan_kick = rospy.get_param("/united_soccer_params/Pan_Kick")
	pTilt_kick = rospy.get_param("/united_soccer_params/Tilt_Kick")

	if pos_pan >= 0 :#and right_kick == False and left_kick == False: # left_kick
		left_kick = True
		right_kick = False
	elif pos_pan <= 0 :#and right_kick == False and left_kick == False: # right_kick
		right_kick = True
		left_kick = False

	if left_kick: 
		if ball_pos:
			motion_state_pub.publish("stop")		
			time.sleep(1)
			motion_state_pub.publish("action 1")
			# return True
		else:
			ball_positioning(-pPan_kick, pTilt_kick, 0.10)
	if right_kick:
		if ball_pos:
			motion_state_pub.publish("stop")		
			time.sleep(1)
			motion_state_pub.publish("action 2")
			# return True
		else:
			ball_positioning(pPan_kick, pTilt_kick, 0.10)

count_ready_kick = 0

def followBall(mode): #0 normal, 1 sambil belok
	head_track_ball()
	global pos_pan, pos_tilt, count_ready_kick
	set_point_pan = 0.0
	set_point_tilt = 0.0

	if	pos_tilt >= set_point_tilt:
		pos_tilt = set_point_tilt
	elif pos_tilt < -2.0:
		pos_tilt = -2.0

	error_fPan  = pos_pan - set_point_pan
	error_fTilt = pos_tilt - set_point_tilt

	if pos_tilt >= set_point_tilt and pos_pan < 0.4 and pos_pan > -0.4 and ball_x != -1 and ball_y != -1: # Stop(bola sudah dekat)
		count_ready_kick += 1
	else: # Kejar Bola(bola masih jauh)
		count_ready_kick = 0

	if count_ready_kick >= 5:
		px_move = 0.0 # jalan ditempat
		py_move = error_fPan * 0.040 # 0.045
		pa_move = error_fPan * 0.20 # 0.30 0.045
	else:
		if pos_tilt < -1.5:
			px_move = 0.05
		elif pos_tilt >= -1.5 and pos_tilt < -1.3:
			px_move = 0.04
		elif pos_tilt > -1.0:
			px_move = 0.03
		else:
			px_move = 0.02
		py_move = error_fPan * 0.045 # 0.045
		pa_move = error_fPan * 0.25 # 0.35 #0.045

	if mode == 0: # Mode differential walking
		if error_fPan > -0.4 and error_fPan < 0.4:
			# print("AA\n")
			walk(round(px_move, 3), 0.0, round(pa_move,3))
		else:					
			# print("BB\n")
			walk(0.0, 0.0, round(pa_move, 3))
	elif mode == 1: # Mode omnidirectional walking
		if error_fPan > -0.4 and error_fPan < 0.4:
			# print("CC\n")
			walk(round(px_move, 3), round(py_move,3), round(pa_move,3))
		else:					
			#printf("DD\n");
			walk(0.0, 0.0, round(pa_move,3))

def compass_goal_found(compass_goal, compass_minmax=40):
	compass_min = compass_goal - compass_minmax
	if compass_min < 0:
		compass_min = 360 - compass_min
	if compass_min > 360:
		compass_min = compass_min - 360

	compass_max = compass_goal + compass_minmax
	if compass_max < 0:
		compass_max = 360 - compass_max
	if compass_max > 360:
		compass_max = compass_max - 360

	if magneto > compass_min and magneto < compass_max:
		print("True")
		return True
	else:
		print("False")
		return False


def kill_node():
	rospy.signal_shutdown("shutdown time.") 

def main():
	print("United Soccer Player - Running")
	rospy.init_node("united_soccer_player")
	rospy.wait_for_service("/srv_controller")
	
	global head_pub, motion_vel_pub, motion_state_pub
	motion_vel_pub = rospy.Publisher("/motion/cmd_vel", Twist, queue_size=1)
	motion_state_pub = rospy.Publisher("/motion/state", String, queue_size=1)
	head_pub = rospy.Publisher("/head/pos", Float32MultiArray, queue_size=1)
	ball_pos_sub = rospy.Subscriber("/united_soccer/ball/position", Int32MultiArray, ball_pos_callback)
	button_sub = rospy.Subscriber("/button/state", Int32MultiArray, button_callback)
	compass_sub = rospy.Subscriber("/compass/value", Int32, compass_callback)

	print("United Soccer Player - Running")
	time.sleep(0.3)
	motion_state_pub.publish("stand")
	global freq
	freq = 50
	rate = rospy.Rate(freq)
	state = "initial"
	play = False
	button_pressed = [0, 0]
	conf_stop = 0
	foot = "left"
	found_ball = 0
	compass_goal = 175
	count_goal_found = 0
	

	while not rospy.is_shutdown():
		if button[0] == 1:
			button_pressed[0] = 1
		else:
			if button_pressed[0] == 1:
				if play:
					motion_state_pub.publish("sit")
					print("Sit")
					play = False
				else:
					motion_state_pub.publish("stand")
					print("Stand")
					play = True
					state = "initial"
				button_pressed[0] = 0

		#///////////////////////////////////////////////////////////////////////
		#//////////////.............Role of execution............///////////////
		#///////////////////////////////////////////////////////////////////////
		if play :
			# print("pospan %f postilt %f", pos_pan, pos_tilt)
			print(state)
			if state == "initial":
				if ball_lost(20):
					scan_ball(0)
					motion_state_pub.publish("stop")
				else:
					if ball_x != -1 and ball_y != -1 :
						found_ball += 1
					if found_ball >= 50:
						state = "follow_ball"
						found_ball = 0
			elif state == "follow_ball":
				if ball_lost(20):
					scan_ball(0)
					walk(0.0,0.0,0.0)
					# motion_state_pub.publish("stop")
					# head_move(0.0, -1.3)
				else:
					head_track_ball()
					followBall(1)
					if pos_tilt >= -0.6 and ball_x != -1 and ball_y != -1:
						state = "positioning"
					# if button[1] == 1:
						# state = "kick"
					# 	motion_state_pub.publish("start")
					# 	state = "forward"
			
			elif state == "positioning":
				if ball_lost(20):
					state = "follow_ball"
					# scan_ball(0)
					# walk(0.0,0.0,0.0)
				else:
					head_track_ball()
					set_point_x = rospy.get_param("/united_soccer_params/Pan_Kick")
					set_point_y = rospy.get_param("/united_soccer_params/Tilt_Kick")
					# if pos_pan > 0:
					ball_positioning2(-set_point_x, set_point_y)
					# if ball_pos == True and compass_goal_found(compass_goal) == True:
					# 	count_goal_found += 1
					# else:
					# 	count_goal_found = 0 
					if ball_pos == True :
						count_goal_found += 1
					else:
						count_goal_found = 0 
					
					if count_goal_found > 5:
						state = "kick_left"
						count_goal_found = 0
					# else:
					# 	state = "goto_goal_heading"
					# else:
					# 	ball_positioning2(-set_point_x, set_point_y)
					# 	if ball_pos == True:
					# 		state = "kick_left"

			elif state == "goto_goal_heading":
				if ball_lost(20):
					# scan_ball(0)
					# walk(0.0,0.0,0.0)
					state = "follow_ball"
				else:
					head_track_ball()
					if compass_goal_found(compass_goal) == True:
						count_goal_found += 1
					else:
						count_goal_found = 0

					if count_goal_found > 5:
						state = "positioning"
						count_goal_found = 0
					else:
						rotate_alpha = pos_pan * rospy.get_param("/united_soccer_params/KP_Compass_A")
						rotate_y = 0.3
						if pos_tilt > -0.3:
							walk(0.00, round(rotate_y, 3), round(rotate_alpha,3))
						else:
							error_tilt = pos_tilt - rospy.get_param("/united_soccer_params/Tilt_Kick")
							rotate_x = -error_tilt * rospy.get_param("/united_soccer_params/KP_Compass_X")
							walk(round(rotate_x, 3), round(rotate_y, 3), round(rotate_alpha,3))
					
			elif state == "kick_right":
				motion_state_pub.publish("stop")		
				time.sleep(1)
				motion_state_pub.publish("action 2")
				state = "initial"
			elif state == "kick_left":
				motion_state_pub.publish("stop")		
				time.sleep(1)
				motion_state_pub.publish("action 1")
				state = "initial"
			
			elif state == "tune_head":
				if ball_lost(20):
					scan_ball(0)
				else:
					head_track_ball()
				# print("%d, %d", ball_x, ball_y)
			
			elif state == "tune_body":
				if ball_lost(20):
					motion_state_pub.publish("stop")
					scan_ball(0)
				else:
					head_track_ball()
					shift = body_track_ball()
					walk(0.0, 0.0, shift)
				# print("%d, %d", ball_x, ball_y)

			elif state == "test_kick":
				if button[1] == 1:
					if foot == "left":
						motion_state_pub.publish("action 1") # left_kick
						foot = "right"
					elif foot == "right":
						motion_state_pub.publish("action 2") # right_kick
						foot = "left"
						
		rate.sleep()
	print("United Soccer Player - Shut Down")
	rospy.on_shutdown(kill_node)

if __name__ == "__main__":
	main()