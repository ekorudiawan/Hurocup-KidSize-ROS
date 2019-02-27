#!/usr/bin/env python
import cv2 as cv
import numpy as np
import math
import rospy
import roslib
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

# # global rgb_img
# rgb_img = np.zeros((640, 480, 3), np.uint8)
# untuk coba
test_file = "/home/tinker/catkin_ws/dataset/marathon/marker_images/test/test (28).jpg"
rgb_img = cv.imread(test_file)

dataset_location = "/home/tinker/catkin_ws/dataset/marathon/marker_images/train/random/"

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

def main():
    rospy.loginfo("Marathon Marker Detector - Running")
    rospy.init_node("marathon_marker_detector")
    marker_img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_sub_callback)
    #global marker_img_pub, marker_str_pub
    marker_img_pub = rospy.Publisher("/marathon/dataset/result_img", Image, queue_size=1)
    marker_bin_img_pub = rospy.Publisher("/marathon/dataset/binary_img", Image, queue_size=1)
    roi_img_pub = rospy.Publisher("/marathon/dataset/roi_img", Image, queue_size=1)

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
    dataset_number = 0
    while not rospy.is_shutdown():
        global rgb_img
        result_img = rgb_img.copy()
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

        marker_roi = np.zeros((640, 480, 3), np.uint8)
        contour_color = (0, 255, 0)
        if len(marker_contours) > 0:
            sorted_marker_contours = sorted(marker_contours, key=cv.contourArea, reverse=True)[:3]
            marker_cntr = sorted_marker_contours[0]
            box_x, box_y, box_w, box_h = cv.boundingRect(marker_cntr)
            cv.rectangle(result_img, (box_x, box_y), (box_x + box_w, box_y + box_h), contour_color, 2)
            marker_roi = rgb_img[box_y:box_y + box_h, box_x:box_x + box_w]
            
        marker_img_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
        marker_bin_img_pub.publish(bridge.cv2_to_imgmsg(binary_black, "mono8"))
        roi_img_pub.publish(bridge.cv2_to_imgmsg(marker_roi, "bgr8"))
        cv.imshow("Proposed ROI", marker_roi)
        cv.imshow("Binary Image", binary_black)
        cv.imshow("Result Image", result_img)
        key = cv.waitKey(1) 
        if key == ord("s"):
            dataset_file = "dataset_" + str(dataset_number) + ".jpg"
            cv.imwrite(dataset_location + dataset_file, marker_roi)
            dataset_number += 1
            print("Saving " + dataset_file+ " Success !")
        elif key == ord("x"):
            print("Exit From Dataset Creator")
            break
        rate.sleep()
    cv.destroyAllWindows()
if __name__ == "__main__":
    main()