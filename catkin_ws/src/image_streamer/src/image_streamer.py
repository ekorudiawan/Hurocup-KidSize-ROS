import cv2 as cv
import numpy as np
import rospy
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def main():
    rospy.loginfo("Image Streamer - Running")
    rospy.init_node("image_streamer")
    img_pub = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=100)
    rate = rospy.Rate(1000)

    video_file = "/home/tinker/catkin_ws/dataset/marathon/videos/video2.mp4"
    cap = cv.VideoCapture(video_file)

    while (not rospy.is_shutdown()) and cap.isOpened():
        ret, rgb_img = cap.read()
        if ret:
            img_pub.publish(bridge.cv2_to_imgmsg(np.array(rgb_img), "bgr8"))
        rate.sleep()

    cap.release()

if __name__ == "__main__":
	main()
