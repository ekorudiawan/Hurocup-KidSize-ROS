import cv2 as cv
import numpy as np
import math
import rospy
import roslib
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import zbar
import Image as PILImage

bridge = CvBridge()

def img_sub_callback(img_msg):
    try:
        global rgb_img
        rgb_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.loginfo(e)

def main():
    rospy.loginfo("Sprint QRCode Detector - Running")
    rospy.init_node("sprint_qrcode_detector")
    marker_img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_sub_callback)
    qrcode_img_pub = rospy.Publisher("/sprint/qrcode/image", Image, queue_size=100)
    rate = rospy.Rate(100)

    scanner = zbar.ImageScanner()
    scanner.parse_config('enable')

    while not rospy.is_shutdown():
        global rgb_img
        image_file = "/home/tinker/catkin_ws/dataset/sprint/qr_code/image1.png"
        rgb_img = cv.imread(image_file)
        result_img = rgb_img.copy()
        gray_img = cv.cvtColor(rgb_img, cv.COLOR_BGR2GRAY)
        pil = PILImage.fromarray(gray_img)
        width, height = pil.size
        raw = pil.tobytes()

        image = zbar.Image(width, height, 'Y800', raw)
        scanner.scan(image)
        red_color = (0, 0, 255)

        list_centre_qrcode = []
        for symbol in image:
            rospy.loginfo("Decoded QRCode :" + str(symbol.data))
            p = np.array(symbol.location)
            cv.drawContours(result_img, [p], 0, red_color, 2)
            M = cv.moments(p)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centre_qrcode = (cx, cy)
            list_centre_qrcode.append(centre_qrcode)
            cv.circle(result_img, centre_qrcode, 10, red_color, -1)

        # perlu tambah program deteksi centre dari 2 marker
        # if len(list_centre_qrcode) == 2:



        qrcode_img_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))
        # cv.imshow("QR Dectector", result_img)
        # cv.waitKey(1)
        rate.sleep()
    cv.destroyAllWindows()

if __name__ == "__main__":
	main()
