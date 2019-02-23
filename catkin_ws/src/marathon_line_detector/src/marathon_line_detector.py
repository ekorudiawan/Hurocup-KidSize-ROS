#!/usr/bin/env python
import cv2 as cv
import numpy as np
import math
import rospy
import roslib
import time
from std_msgs.msg import String, Float32, Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

total_roi = 1
global rgb_img
rgb_img = np.zeros((rospy.get_param("/usb_cam/image_height"), rospy.get_param("/usb_cam/image_width"), 3), np.uint8)

def nothing(x):
    pass

def image_callback(img_msg):
    global bridge
    try:
        global rgb_img
        rgb_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        
    except CvBridgeError as e:
        rospy.loginfo(e)

def kill_node():
	rospy.signal_shutdown("shutdown time.") 

def main():
    rospy.loginfo("Marathon Line Detector - Running")
    rospy.init_node("marathon_line_detector")
    # time.sleep(.5)
    global line_angle_pub, line_pos_pub, line_bin_img_pub, line_rslt_img_pub
    line_image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, image_callback)
    line_angle_pub = rospy.Publisher("/marathon/line/angle", Float32, queue_size=1)
    line_pos_pub = rospy.Publisher("/marathon/line/position", Int32MultiArray, queue_size=1)
    line_bin_img_pub = rospy.Publisher("/marathon/line/binary_img", Image, queue_size=1)
    line_rslt_img_pub = rospy.Publisher("/marathon/line/result_img", Image, queue_size=1)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        global rgb_img
        result_img = rgb_img.copy()
        img_h, img_w, _ = rgb_img.shape

        red_h_max = rospy.get_param("/marathon_params/H_Max")
        red_h_min = rospy.get_param("/marathon_params/H_Min")
        red_s_max = rospy.get_param("/marathon_params/S_Max")
        red_s_min = rospy.get_param("/marathon_params/S_Min")
        red_v_max = rospy.get_param("/marathon_params/V_Max")
        red_v_min = rospy.get_param("/marathon_params/V_Min")
        blob_size = rospy.get_param("/marathon_params/Min_Size")

        hsv_img = cv.cvtColor(rgb_img, cv.COLOR_BGR2HSV)
        lower_red = np.array([red_h_min, red_s_min, red_v_min])
        upper_red = np.array([red_h_max, red_s_max, red_v_max])
        red_bin_img = cv.inRange(hsv_img, lower_red, upper_red)
        list_centre_dot =[]
        roi_height = img_h / total_roi + 1
        contour_color = (0, 255, 0)
        arrow_color = (0, 255, 255)

        for i in range(0, total_roi):
            roi_image = red_bin_img[i*roi_height:(i+1)*roi_height, :]
            _, line_contours, _ = cv.findContours(roi_image.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            sorted_line_contours = []
            if len(line_contours) > 0:
                sorted_line_contours = sorted(line_contours, key=cv.contourArea, reverse=True)[:3]

            if len(sorted_line_contours) > 0:
                line_cntr = sorted_line_contours[0]
                cntr_area = cv.contourArea(line_cntr)
                if cntr_area > blob_size:
                    box_x, box_y, box_w, box_h = cv.boundingRect(line_cntr)
                    # Update ukuran ke image sebenarnya
                    box_y = i * roi_height + box_y
                    boline_centre_x_x = box_x + box_w / 2
                    boline_centre_x_y = box_y + box_h / 2
                    cv.rectangle(result_img, (box_x, box_y), (box_x + box_w, box_y + box_h), contour_color, 1)
                    centre_dot = (boline_centre_x_x, boline_centre_x_y)
                    cv.circle(result_img, centre_dot, 5, (255, 0, 0), -1)
                    list_centre_dot.append(centre_dot)

        x1 = 0
        x2 = 0
        angle = -888
        line_centre_x = -1
        line_centre_y = -1
        if len(list_centre_dot) > 1:
            cv.arrowedLine(result_img, list_centre_dot[-1], list_centre_dot[0], arrow_color, 2)
            x1, y1 = list_centre_dot[-1]
            x2, y2 = list_centre_dot[0]
            c_dot = (x2, y1)
            cv.circle(result_img, c_dot, 5, (255, 0, 0), -1)
            h_length = np.linalg.norm(np.array(list_centre_dot[-1])-np.array(list_centre_dot[0]))
            a_length = np.linalg.norm(np.array(list_centre_dot[0])-np.array(c_dot))

            line_centre_x = x1
            line_centre_y = y1

            angle_sign = 1

            if x2 >= x1:
                angle_sign = 1
            elif x2 < x1:
                angle_sign = -1

            angle = (90 - (math.asin(a_length / h_length) * 180 / math.pi)) * angle_sign
        
        elif len(list_centre_dot) == 1:
            line_centre_x, line_centre_y = list_centre_dot[0]
            
        angle_msg = Float32
        angle_msg = angle

        line_pos = Int32MultiArray()
        line_pos.data = [line_centre_x, line_centre_y]

        rospy.loginfo("Angle = " + str(angle))
        rospy.loginfo("Line Pos = " + str(line_pos))

        list_centre_dot = None
        line_angle_pub.publish(angle_msg)
        line_pos_pub.publish(line_pos)
        line_bin_img_pub.publish(bridge.cv2_to_imgmsg(red_bin_img, "mono8"))
        line_rslt_img_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
        rate.sleep()
    # cv.destroyAllWindows()
    rospy.loginfo("Marathon Line Detector - Shutdown")
    rospy.on_shutdown(kill_node)

if __name__ == "__main__":
    main()