import numpy as np
import cv2 as cv
from cv2 import aruco
import rospy
import roslib
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def img_sub_callback(img_msg):
    try:
        global rgb_img
        rgb_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.loginfo(e)

def main():
    rospy.loginfo("Sprint ArUco Detector - Running")
    rospy.init_node("sprint_aruco_detector")

    aruco_img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_sub_callback)
    aruco_img_pub = rospy.Publisher("/sprint/aruco/image", Image, queue_size=100)
    rate = rospy.Rate(1000)

    while not rospy.is_shutdown():
        global rgb_img
        image_file = "/home/tinker/catkin_ws/dataset/sprint/aruco/image0.png"
        rgb_img = cv.imread(image_file)
        result_img = rgb_img.copy()
        gray_img = cv.cvtColor(rgb_img, cv.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, aruco_dict, parameters=parameters)
        result_img = aruco.drawDetectedMarkers(result_img, corners, ids)
        aruco_img_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
    rospy.loginfo("Sprint ArUco Detector - Shut Down")
if __name__ == "__main__":
    main()