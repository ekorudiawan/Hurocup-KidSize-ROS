#!/usr/bin/env python
import cv2 as cv
import numpy as np
import math
import rospy
import roslib
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
from keras.models import model_from_json

# CNN input image
# 100x100 grayscale
img_w = 100
img_h = 100
n_chn = 1
img_size = (img_w, img_h)

bridge = CvBridge()

# # global rgb_img
# rgb_img = np.zeros((640, 480, 3), np.uint8)
# untuk coba
test_file = "/home/tinker/catkin_ws/dataset/marathon/marker_images/test/test (28).jpg"
rgb_img = cv.imread(test_file)

black_h_min = 45
black_h_max = 255
black_s_min = 0
black_s_max = 100
black_v_min = 0
black_v_max = 120

def img_sub_callback(img_msg):
    try:
        global rgb_img
        rgb_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")

    except CvBridgeError as e:
        rospy.loginfo(e)

def update_parameter(x):
    pass

# def cosine_similarity(x, y):
#     cos = np.dot(x, y) / (np.sqrt(np.dot(x,x)) * np.sqrt(np.dot(y,y)))
#     return cos

def main():
    rospy.loginfo("Marathon Marker Detector - Running")
    rospy.init_node("marathon_marker_detector")
    marker_img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_sub_callback)
    #global marker_img_pub, marker_str_pub
    marker_img_pub = rospy.Publisher("/marathon/marker/image", Image, queue_size=1)
    roi_img_pub = rospy.Publisher("/marathon/marker/roi", Image, queue_size=1)
    marker_str_pub = rospy.Publisher("/marathon/marker/result", String, queue_size=1)
    json_file = open('/home/tinker/catkin_ws/src/marathon_marker_detector_with_color/src/model.json', 'r')
    loaded_model_json = json_file.read()
    json_file.close()
    #global loaded_model
    loaded_model = model_from_json(loaded_model_json)
    loaded_model.load_weights("/home/tinker/catkin_ws/src/marathon_marker_detector_with_color/src/model.h5")
    rospy.loginfo("Loaded model from disk successfull")

    cv.namedWindow('Control')
    cv.createTrackbar('HMin','Control',0,255,update_parameter)
    cv.createTrackbar('HMax','Control',255,255,update_parameter)
    cv.createTrackbar('SMin','Control',0,255,update_parameter)
    cv.createTrackbar('SMax','Control',255,255,update_parameter)
    cv.createTrackbar('VMin','Control',0,255,update_parameter)
    cv.createTrackbar('VMax','Control',255,255,update_parameter)

    global black_h_min, black_h_max, black_s_min, black_s_max, black_v_min, black_v_max

    cv.setTrackbarPos('HMin','Control', black_h_min)
    cv.setTrackbarPos('HMax','Control', black_h_max)
    cv.setTrackbarPos('SMin','Control', black_s_min)
    cv.setTrackbarPos('SMax','Control', black_s_max)
    cv.setTrackbarPos('VMin','Control', black_v_min)
    cv.setTrackbarPos('VMax','Control', black_v_max)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        global rgb_img
        result_img = rgb_img.copy()
        gray_image = cv.cvtColor(rgb_img, cv.COLOR_BGR2GRAY)
        hsv_image = cv.cvtColor(rgb_img, cv.COLOR_BGR2HSV)
        black_h_min = cv.getTrackbarPos("HMin", "Control")
        black_h_max = cv.getTrackbarPos("HMax", "Control")
        black_s_min = cv.getTrackbarPos("SMin", "Control")
        black_s_max = cv.getTrackbarPos("SMax", "Control")
        black_v_min = cv.getTrackbarPos("VMin", "Control")
        black_v_max = cv.getTrackbarPos("VMax", "Control")

        lower_black = np.array([black_h_min,black_s_min,black_v_min])
        upper_black = np.array([black_h_max,black_s_max,black_v_max])

        binary_black = cv.inRange(hsv_image,lower_black,upper_black)

        _, marker_contours, _ = cv.findContours(binary_black.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        str_result = "<UNK>"
        marker_roi = None
        contour_color = (0, 255, 0)
        if len(marker_contours) > 0:
            sorted_marker_contours = sorted(marker_contours, key=cv.contourArea, reverse=True)[:3]
            marker_cntr = sorted_marker_contours[0]
            box_x, box_y, box_w, box_h = cv.boundingRect(marker_cntr)
            cv.rectangle(result_img, (box_x, box_y), (box_x + box_w, box_y + box_h), contour_color, 2)

            marker_roi = gray_image[box_y:box_y + box_h, box_x:box_x + box_w]
            marker_roi = cv.resize(marker_roi, (100, 100))
            input_img = np.array(marker_roi).reshape(1, img_w, img_h, n_chn)
            pred_output = loaded_model.predict(input_img)
            rospy.loginfo(pred_output.reshape(3))
            index = np.argmax(pred_output[0])
            font = cv.FONT_HERSHEY_SIMPLEX
            red_color = (0, 0, 255)
            text_pos = (20, 60)
            
            if index == 0:
                cv.putText(result_img, "Forward", text_pos, font, 2, red_color, 2, cv.LINE_AA)
                str_result ="forward"
                rospy.loginfo("Marker = Forward")
            elif index == 1:
                cv.putText(result_img, "Right", text_pos, font, 2, red_color, 2, cv.LINE_AA)
                str_result = "right"
                rospy.loginfo("Marker = Right")
            elif index == 2:
                cv.putText(result_img, "Left", text_pos, font, 2, red_color, 2, cv.LINE_AA)
                str_result = "left"
                rospy.loginfo("Marker = Left")

        marker_img_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
        roi_img_pub.publish(bridge.cv2_to_imgmsg(marker_roi, "mono8"))
        marker_str_pub.publish(str_result)
        # cv.imshow("ROI", marker_roi)
        # cv.imshow("Gambar", result_img)
        rate.sleep()
if __name__ == "__main__":
    main()