#! /usr/bin/env python
import numpy as np
import math
import rospy
import roslib
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32, Bool, Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

frame_w = rospy.get_param("/usb_cam/image_width")
frame_h = rospy.get_param("/usb_cam/image_height")

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
tilt_min = -1.5
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

def scan_line(mode):
    global pos_pan, pos_tilt, pan_step, tilt_step, move_pan

    if pan_step > 0:        
        pan_step = rospy.get_param("/marathon_params/Pan_Step")
    else:               
        pan_step = rospy.get_param("/marathon_params/Pan_Step") * -1
    if tilt_step > 0:   
        tilt_step = rospy.get_param("/marathon_params/Tilt_Step") 
    else:               
        tilt_step = rospy.get_param("/marathon_params/Tilt_Step") * -1
    
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

    elif mode == 3: # only pan
        pos_pan += pan_step
        pos_tilt = rospy.get_param("/marathon_params/Tilt_Angle")
        if pos_pan >= pan_max or pos_pan <= pan_min:
            pan_step *= -1  

    head_pos = head_limit(pos_pan, round(pos_tilt, 3))
    pos_pan, pos_tilt = head_pos.data
    head_pub.publish(head_pos)

button = [0, 0]

def button_callback(pos_msg):
    global button
    button[0] = pos_msg.data[0]
    button[1] = pos_msg.data[1]

magneto = 0

def compass_callback(cmps_msg):
    global magneto
    magneto = cmps_msg.data

line_angle = -888

def line_angle_callback(angle_msg):
    global line_angle
    line_angle = angle_msg.data
    # rospy.loginfo("Line angle : " + str(line_angle))

marker = "blank"

def marker_result_callback(marker_msg):
    global marker
    marker = marker_msg
    # rospy.loginfo("Marker : "+ marker.data)

line_centre_x = -1
line_centre_y = -1

def line_pos_callback(line_pos_msg):
    global line_centre_x, line_centre_y
    line_centre_x = line_pos_msg.data[0]
    line_centre_y = line_pos_msg.data[1]

count_line_loss = 0

def line_lost(threshold):
    global count_line_loss
    if line_centre_x == -1 and line_centre_y == -1:      
        count_line_loss += 1
        if count_line_loss >= threshold :
            return True
    else :
        count_line_loss = 0
        return False

sum_err_pan = 0
sum_err_tilt = 0
last_error_x = 0
last_error_y = 0

def head_track_line():
    global pos_pan, pos_tilt, sum_err_pan, sum_err_tilt, last_error_x, last_error_y
    global freq
    dt = 1.0 / float(freq)
    KP_pan = rospy.get_param("/marathon_params/Pan_KP")
    KI_pan = rospy.get_param("/marathon_params/Pan_KI")
    KD_pan = rospy.get_param("/marathon_params/Pan_KD")
    # KP_tilt = rospy.get_param("/marathon_params/Tilt_KP")
    # KI_tilt = rospy.get_param("/marathon_params/Tilt_KI")
    # KD_tilt = rospy.get_param("/marathon_params/Tilt_KD")
    global line_centre_x
    if line_centre_x != -1 and line_centre_y != -1:
        error_x = (float(frame_w)/2.00) - line_centre_x
        error_x *= 77.32 / float(frame_w)
        error_x = (error_x * math.pi)/ 180
        error_x_diff = error_x - last_error_x

        P_pan  = last_error_x * KP_pan
        sum_err_pan += error_x * dt
        I_pan = sum_err_pan * KI_pan
        deriv_err_pan = error_x_diff / dt
        D_pan = deriv_err_pan * KD_pan
        last_error_x = error_x
        pos_pan += (P_pan + I_pan + D_pan)

        # error_y = (frame_h/2) - marker_y
        # error_y *= -1
        # error_y *= 61.93 / frame_h
        # error_y = (error_y * math.pi) /180
        # error_y_diff = error_y - last_error_y

        # P_tilt  = last_error_y * KP_tilt
        # sum_err_tilt += error_y * dt
        # I_tilt = sum_err_tilt * KI_tilt
        # deriv_err_tilt = sum_err_tilt / dt
        # D_tilt = deriv_err_tilt * KD_tilt
        # last_error_y = error_y
        # pos_tilt += (P_tilt + I_tilt + D_tilt)

        pos_tilt = rospy.get_param("/marathon_params/Tilt_Angle")

        head_pos = head_limit(pos_pan, round(pos_tilt, 2))
        # rospy.loginfo("Marker : %s Line Centre : %d,%d Line Angle: %d", marker, line_centre_x, line_centre_y, line_angle)
        pos_pan, pos_tilt = head_pos.data
        head_pub.publish(head_pos)

def body_track_line():
    global pos_pan, pos_tilt
    KP_body = rospy.get_param("/marathon_params/Body_KP")
    if line_centre_x != -1:   
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

def kill_node():
    rospy.signal_shutdown("shutdown time.") 

def main():
    # rospy.loginfo("Marathon Player - Running")
    rospy.init_node("marathon_player")
    rospy.wait_for_service("/srv_controller")

    global head_pub, motion_vel_pub, motion_state_pub
    motion_vel_pub = rospy.Publisher("/motion/cmd_vel", Twist, queue_size=1)
    motion_state_pub = rospy.Publisher("/motion/state", String, queue_size=1)
    gripper_state_pub = rospy.Publisher("/gripper/state", Bool, queue_size=1)
    head_pub = rospy.Publisher("/head/pos", Float32MultiArray, queue_size=1)

    button_sub = rospy.Subscriber("/button/state", Int32MultiArray, button_callback)
    compass_sub = rospy.Subscriber("/compass/value", Int32, compass_callback)
    line_angle_sub = rospy.Subscriber("/marathon/line/angle", Float32, line_angle_callback)
    line_pos_sub = rospy.Subscriber("/marathon/line/position", Int32MultiArray, line_pos_callback)
    marker_result_sub = rospy.Subscriber("/marathon/marker/result", String, marker_result_callback)

    rospy.loginfo("Marathon Player - Running")
    time.sleep(0.3)
    motion_state_pub.publish("stand")
    global freq
    freq = 50
    rate = rospy.Rate(freq)
    state = "initial"
    play = True
    button_pressed = [0, 0]
    conf_stop = 0
    conf_start = 0

    while not rospy.is_shutdown():
        if button[0] == 1:
            button_pressed[0] = 1
        else:
            if button_pressed[0] == 1:
                if play:
                    motion_state_pub.publish("sit")
                    rospy.loginfo("Sit")
                    play = False
                else:
                    motion_state_pub.publish("stand")
                    rospy.loginfo("Stand")
                    # Set default head position
                    # pos_pan = 0.0
                    # pos_tilt = rospy.get_param("/marathon_params/Tilt_Angle")
                    play = True
                    state = "initial"
                button_pressed[0] = 0

        #///////////////////////////////////////////////////////////////////////
        #//////////////.............Role of execution............///////////////
        #///////////////////////////////////////////////////////////////////////
        if play :
            # rospy.loginfo(state)
            if state == "initial":
                if line_lost(20):
                    scan_line(3)
                    # head_move(0.0, -1.3)
                else:
                    head_track_line()       
                    # rospy.loginfo(pos_tilt)
                    if button[1] == 1:
                        motion_state_pub.publish("start")
                        state = "follow_line"
                            
            elif state == "follow_line":   
                if line_lost(20):
                    scan_line(3)
                    walk(0.04, 0.0, 0.00)
                else: 
                    head_track_line()
                    shift = body_track_line()
                    walk(0.04, 0.0, shift)
                # # PID body
                # if marker == "straight":
                #     count_straight += 1
                # else:
                #     count_straight = 0
                # if count_straight > marker_threshold:
                #     count_straight = 0
                #     state = "go_ahead"

                # if marker == "right":
                #     count_right += 1
                # else:
                #     count_right = 0
                # if count_right > marker_threshold:
                #     count_right = 0
                #     state = "turn_right"

                # if marker == "left":
                #     count_left += 1
                # else:
                #     count_left = 0
                # if count_left > marker_threshold:
                #     count_left = 0
                #     state = "turn_left"

            elif state == "go_ahead":
                # wait until line found
                if marker == "line":
                    count_line += 1
                else:
                    count_line = 0
                if count_line > marker_threshold:
                    count_line = 0
                    state = "follow_line"
            elif state == "turn_left":
                # rotate compass
                walk(0.00, 0.00, 0.00)
                if marker == "line":
                    count_line += 1
                else:
                    count_line = 0
                if count_line > marker_threshold:
                    count_line = 0
                    state = "follow_line"
            elif state == "turn_right":
                # rotate compass
                walk(0.00, 0.00, 0.00)
                if marker == "line":
                    count_line += 1
                else:
                    count_line = 0
                if count_line > marker_threshold:
                    count_line = 0
                    state = "follow_line"        
        rate.sleep()

    # rate = rospy.spin()
    rospy.loginfo("Marathon Player - Shut Down")
    rospy.on_shutdown(kill_node)

if __name__ == "__main__":
    main()