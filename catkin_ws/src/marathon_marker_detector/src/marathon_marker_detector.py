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

# global rgb_img
rgb_img = np.zeros((640, 480, 3), np.uint8)

def img_sub_callback(img_msg):
    try:
        global rgb_img
        rgb_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")

    except CvBridgeError as e:
        rospy.loginfo(e)

def main():
    rospy.loginfo("Marathon Marker Detector - Running")
    rospy.init_node("marathon_marker_detector")
    marker_img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_sub_callback)
    #global marker_img_pub, marker_str_pub
    marker_img_pub = rospy.Publisher("/marathon/marker/image", Image, queue_size=1)
    marker_str_pub = rospy.Publisher("/marathon/marker/result", String, queue_size=1)
    json_file = open('/home/barelangfc/catkin_ws/src/marathon_marker_detector/src/model.json', 'r')
    loaded_model_json = json_file.read()
    json_file.close()
    #global loaded_model
    loaded_model = model_from_json(loaded_model_json)
    loaded_model.load_weights("/home/barelangfc/catkin_ws/src/marathon_marker_detector/src/model.h5")
    rospy.loginfo("Loaded model from disk successfull")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        global rgb_img
        result_img = rgb_img.copy()
        input_img = cv.cvtColor(rgb_img, cv.COLOR_BGR2GRAY)
        input_img = cv.resize(input_img, img_size)
        input_img = np.array(input_img).reshape(1, img_w, img_h, n_chn)
        #global loaded_model
        pred_output = loaded_model.predict(input_img)
        rospy.loginfo(pred_output.reshape(3))
        index = np.argmax(pred_output[0])
        font = cv.FONT_HERSHEY_SIMPLEX
        red_color = (0, 0, 255)
        text_pos = (20, 60)
        str_result = ""
        if index == 0:
            cv.putText(result_img, "Blank", text_pos, font, 2, red_color, 2, cv.LINE_AA)
            str_result ="blank"
            rospy.loginfo("Marker = Blank")
        elif index == 1:
            cv.putText(result_img, "Line", text_pos, font, 2, red_color, 2, cv.LINE_AA)
            str_result = "line"
            rospy.loginfo("Marker = Line")
        elif index == 2:
            cv.putText(result_img, "Right", text_pos, font, 2, red_color, 2, cv.LINE_AA)
            str_result = "right"
            rospy.loginfo("Marker = Right")

        marker_img_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
        marker_str_pub.publish(str_result)
        rate.sleep()
    rospy.loginfo("Marathon Marker Detector - Shut Down")

if __name__ == "__main__":
    main()
