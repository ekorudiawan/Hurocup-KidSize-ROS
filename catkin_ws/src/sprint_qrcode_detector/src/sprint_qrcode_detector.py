#!/usr/bin/env python
import cv2 as cv
import numpy as np
import math
import rospy
import roslib
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import zbar
import Image as PILImage
from rospy.numpy_msg import numpy_msg

bridge = CvBridge()
scanner = zbar.ImageScanner()
scanner.parse_config('enable')

def img_sub_callback(img_msg):
    try:
        rgb_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        result_img = rgb_img.copy()
        gray_img = cv.cvtColor(rgb_img, cv.COLOR_BGR2GRAY)
        pil = PILImage.fromarray(gray_img)
        width, height = pil.size
        raw = pil.tobytes()

        image = zbar.Image(width, height, 'Y800', raw)
        scanner.scan(image)
        red_color = (0, 0, 255)
	detected = False
	cx = -1
	cy = -1
        for symbol in image:
            p = np.array(symbol.location)
            cv.drawContours(result_img, [p], 0, red_color, 2)
            M = cv.moments(p)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centre_qrcode = (cx, cy)
            rospy.loginfo("Decoded QRCode :" + str(symbol.data) + "Position :" + str(centre_qrcode))
            cv.circle(result_img, centre_qrcode, 10, red_color, -1)
	    detected = True
	if detected:
	    rospy.loginfo("QRCode Detected")
	else:
	    rospy.loginfo("QRCode Not Detected")
	qrcode_posx_pub.publish(cx)
	qrcode_posy_pub.publish(cy)
        qrcode_img_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
    except CvBridgeError as e:
        rospy.loginfo(e)

def main():
    rospy.loginfo("Sprint QRCode Detector - Running")
    rospy.init_node("sprint_qrcode_detector")
    marker_img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_sub_callback)
    global qrcode_img_pub
    qrcode_img_pub = rospy.Publisher("/sprint/qrcode/image", Image, queue_size=1)
    global qrcode_posx_pub
    qrcode_posx_pub = rospy.Publisher("/sprint/qrcode/pos_x", Int32, queue_size=1)
    global qrcode_posy_pub
    qrcode_posy_pub = rospy.Publisher("/sprint/qrcode/pos_y", Int32, queue_size=1)
    #rate = rospy.Rate(5000)

    #while not rospy.is_shutdown():
    #    rate.sleep()
    rospy.spin()
    rospy.loginfo("Sprint QRCode Detector - Shut Down")

if __name__ == "__main__":
    main()
