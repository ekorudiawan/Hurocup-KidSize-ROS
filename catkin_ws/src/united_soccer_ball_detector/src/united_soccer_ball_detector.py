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
    rospy.loginfo("United Soccer Ball Detector - Running")
    rospy.init_node("united_soccer_ball_detector")
    time.sleep(.5)
    global ball_pos_pub, ball_bin_img_pub, ball_rslt_img_pub
    ball_image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, image_callback)    
    ball_pos_pub = rospy.Publisher("/united_soccer/ball/position", Int32MultiArray, queue_size=1)    
    ball_bin_img_pub = rospy.Publisher("/united_soccer/ball/binary_img", Image, queue_size=1)
    ball_rslt_img_pub = rospy.Publisher("/united_soccer/ball/result_img", Image, queue_size=1)
    field_bin_img_pub = rospy.Publisher("/united_soccer/field/binary_img", Image, queue_size=1)
    field_img_pub = rospy.Publisher("/united_soccer/field/field_img", Image, queue_size=1)
    rate = rospy.Rate(50)
    kernel = cv.getStructuringElement(cv.MORPH_RECT,(2,2))
    ball_centre_x = -1; ball_centre_y = -1
    im_area = rospy.get_param("/usb_cam/image_height") * rospy.get_param("/usb_cam/image_width")

    while not rospy.is_shutdown():
        global rgb_img
        result_img = rgb_img.copy()
        img_h, img_w, _ = rgb_img.shape

        ball_h_max = rospy.get_param("/united_soccer_vision_params/H_Max")
        ball_h_min = rospy.get_param("/united_soccer_vision_params/H_Min")
        ball_s_max = rospy.get_param("/united_soccer_vision_params/S_Max")
        ball_s_min = rospy.get_param("/united_soccer_vision_params/S_Min")
        ball_v_max = rospy.get_param("/united_soccer_vision_params/V_Max")
        ball_v_min = rospy.get_param("/united_soccer_vision_params/V_Min")
        ball_erode = rospy.get_param("/united_soccer_vision_params/Erode")
        ball_dilate = rospy.get_param("/united_soccer_vision_params/Dilate")

        field_h_max = rospy.get_param("/united_soccer_vision_params/FH_Max")
        field_h_min = rospy.get_param("/united_soccer_vision_params/FH_Min")
        field_s_max = rospy.get_param("/united_soccer_vision_params/FS_Max")
        field_s_min = rospy.get_param("/united_soccer_vision_params/FS_Min")
        field_v_max = rospy.get_param("/united_soccer_vision_params/FV_Max")
        field_v_min = rospy.get_param("/united_soccer_vision_params/FV_Min")
        field_erode = rospy.get_param("/united_soccer_vision_params/FErode")
        field_dilate = rospy.get_param("/united_soccer_vision_params/FDilate")

        hsv_img = cv.cvtColor(rgb_img, cv.COLOR_BGR2LAB)
        grayscale_image = cv.cvtColor(rgb_img, cv.COLOR_BGR2GRAY)
        # Field
        lower_field = np.array([field_h_min, field_s_min, field_v_min])
        upper_field = np.array([field_h_max, field_s_max, field_v_max])
        field_bin_img = cv.inRange(hsv_img, lower_field, upper_field)
        field_bin_img = cv.erode(field_bin_img, kernel, iterations = field_erode)
        field_bin_img = cv.dilate(field_bin_img, kernel, iterations = field_dilate)
        field_mask = np.zeros(rgb_img.shape[:2], np.uint8)
        _, field_contours, _ = cv.findContours(field_bin_img.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        field_img = rgb_img.copy()
        if len(field_contours) > 0:
            field_cntr = max(field_contours, key=cv.contourArea)
            hull = cv.convexHull(field_cntr)
            cv.drawContours(field_mask, [hull], 0, 255, cv.FILLED, offset=(0,0))
            field_img = cv.bitwise_and(rgb_img, rgb_img, mask=field_mask)

        hsv_field_img = cv.cvtColor(field_img, cv.COLOR_BGR2HSV)
        lower_ball = np.array([ball_h_min, ball_s_min, ball_v_min])
        upper_ball = np.array([ball_h_max, ball_s_max, ball_v_max])
        ball_bin_img = cv.inRange(hsv_field_img, lower_ball, upper_ball)
        # ball_bin_img = cv.bitwise_not(ball_bin_img)
        ball_bin_img = cv.erode(ball_bin_img, kernel, iterations = ball_erode)
        ball_bin_img = cv.dilate(ball_bin_img, kernel, iterations = ball_dilate)

        _, ball_contours, _ = cv.findContours(ball_bin_img.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        ball_found =  False
        if ball_found == False:
            if len(ball_contours) > 0:
                sorted_ball_contours = sorted(ball_contours, key=cv.contourArea, reverse=True)[:5]
                # marker_ball = sorted_ball_contours[0]
                # print(marker_ball)
                # marker_area = cv.contourArea(marker_ball)
                # print("Marker Area:", marker_area)
                #if marker_area > blob_size:
                ball_iteration = 0
                for balls in sorted_ball_contours:
                    ball_centre_x = ball_centre_y = -1
                    ball_width = ball_height = 0
                    ball_area = ball_rect_area = 0
                    ball_area_ratio = 0
                    ball_wh_ratio = 0
                    percent_white = 0

                    ball_topleft_x, ball_topleft_y, ball_width, ball_height = cv.boundingRect(balls)
                    ball_centre_x = ball_topleft_x + (ball_width/2)
                    ball_centre_y = ball_topleft_y + (ball_height/2)
                    ball_area = (float(cv.contourArea(balls)) / float(im_area)) * 100.0
                    ball_rect_area = (float (ball_width * ball_height) / float(im_area)) * 100.0
                    
                    # Handle exception devide by zero
                    if ball_rect_area != 0:
                        ball_area_ratio = float(ball_area)/float(ball_rect_area)
                    if ball_height != 0:
                        ball_wh_ratio = float(ball_width) / float(ball_height)
                    predicted_ball = grayscale_image[ball_topleft_y:ball_topleft_y + ball_height, ball_topleft_x:ball_topleft_x + ball_width]
                    hist = cv.calcHist([predicted_ball], [0], None, [5], [0, 256])
                    hist_val_0, hist_val_1, hist_val_2, hist_val_3, hist_val_4 = hist
                    sum_hist = hist_val_0 + hist_val_1 + hist_val_2 + hist_val_3 + hist_val_4

                    if sum_hist > 0:
					    percent_white = (float(hist_val_4) / float(sum_hist)) * 100.0

                    # debug ball
                    if rospy.get_param("/united_soccer_vision_params/debug_ball"):
                        selected_ball = rospy.get_param("/united_soccer_vision_params/ball_no")
                        # print(selected_ball, ball_iteration)
                        if selected_ball == ball_iteration:
                            # print('Ball Number %d ==> X = %d Y = %d W = %d H = %d Area = %.2f R_Area = %.2f Area_Rat = %.2f WH_Rat = %.2f Percent_Wh = %.2f'\
                            #         %(ball_iteration,ball_centre_x,ball_centre_y,ball_width,ball_height,ball_area,ball_rect_area,ball_area_ratio,ball_wh_ratio, percent_white))
                            print('Val = %d Ball Number %d ==> X = %d Y = %d W = %d H = %d Area = %.2f Area_Rat = %.2f WH_Rat = %.2f Percent_Wh = %.2f'\
                                    %(field_bin_img[ball_centre_y, ball_centre_x], ball_iteration,ball_centre_x,ball_centre_y,ball_width,ball_height,ball_area,ball_area_ratio,ball_wh_ratio, percent_white))
                            ball_color = (0, 255, 0)
                        else:
                            ball_color = (0, 0, 255)
                        cv.rectangle(result_img, (ball_topleft_x,ball_topleft_y), (ball_topleft_x + ball_width, ball_topleft_y+ball_height), ball_color, 3)
                    # Decision Tree Ball Detection
                    if not ball_found:
                        # ukuran bola dekat dan ukuran bola jauh
                        ball_area_min = rospy.get_param("/united_soccer_vision_params/ball_area_min")
                        ball_area_max = rospy.get_param("/united_soccer_vision_params/ball_area_max")
                        ball_area_ratio_thresh = rospy.get_param("/united_soccer_vision_params/ball_area_ratio")
                        ball_wh_ratio_min = rospy.get_param("/united_soccer_vision_params/ball_wh_ratio_min")                        
                        ball_wh_ratio_max = rospy.get_param("/united_soccer_vision_params/ball_wh_ratio_max")
                        ball_percent_white_thresh = rospy.get_param("/united_soccer_vision_params/ball_percent_white")
                        # if field_bin_img[ball_centre_y, ball_centre_x] == 0:
                        if ball_area >= ball_area_min and ball_area <= ball_area_max: # Ball ball_area maximal for Detection
                            if ball_area_ratio >= ball_area_ratio_thresh:
                                if ball_wh_ratio >= ball_wh_ratio_min and ball_wh_ratio <= ball_wh_ratio_max: # 0.5 2.7
                                    if percent_white >= ball_percent_white_thresh: # Ball must have minimal 50% white pixel
                                        ball_found = True
                                        cv.circle(result_img, (ball_centre_x, ball_centre_y), (ball_width / 2), (0, 255, 255), 3)
                                        # cv.rectangle(result_img, (ball_centre_x, ball_centre_y), (ball_centre_x + ball_w, ball_centre_y + ball_h), (255, 255, 0), 2)
                                        break
                    ball_iteration += 1
                    # print(ball_iteration)

        if not ball_found:
            ball_centre_x = ball_centre_y = -1
            ball_width = ball_height = 0
            ball_area = 0
            ball_rect_area = 0
            ball_area_ratio = ball_wh_ratio = 0
            percent_white = 0            

        ball_pos = Int32MultiArray()
        ball_pos.data = [ball_centre_x, ball_centre_y]        
        ball_bin_img_pub.publish(bridge.cv2_to_imgmsg(ball_bin_img, "mono8"))
        ball_rslt_img_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
        field_bin_img_pub.publish(bridge.cv2_to_imgmsg(field_bin_img, "mono8"))
        field_img_pub.publish(bridge.cv2_to_imgmsg(field_img, "bgr8"))
        ball_pos_pub.publish(ball_pos)
        rate.sleep()
    # cv.destroyAllWindows()
    rospy.loginfo("United Soccer Ball Detector - Shutdown")
    rospy.on_shutdown(kill_node)

if __name__ == "__main__":
    main()