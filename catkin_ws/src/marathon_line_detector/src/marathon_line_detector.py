import cv2 as cv
import numpy as np
import math
import rospy
import roslib
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

image_iter = 1
image_file = "/home/tinker/catkin_ws/dataset/marathon/line_images/image"+str(image_iter)+".png"
rgb_img = cv.imread(image_file)

red_h_min = 50
red_h_max = 255
red_s_min = 30
red_s_max = 255
red_v_min = 0
red_v_max = 255

total_roi = 5

def nothing(x):
	pass

def image_callback(img_msg):
	global bridge
	try:
		global rgb_img
		rgb_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
	except CvBridgeError as e:
		rospy.loginfo(e)

def main():
    rospy.loginfo("Marathon Line Detector - Running")
    rospy.init_node("marathon_line_detector")
    line_image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, image_callback)
    line_angle_pub = rospy.Publisher("/marathon/line/angle", Float32)
    line_bin_img_pub = rospy.Publisher("/marathon/line/binary_image", Image)
    line_rslt_img_pub = rospy.Publisher("/marathon/line/result_img", Image)

    rate = rospy.Rate(1000)

    # cap = cv.VideoCapture("/home/catkin_ws/dataset/videos/video2.mp4")

    cv.namedWindow("Control")

    cv.createTrackbar("H_min", "Control", 0, 255, nothing)
    cv.createTrackbar("H_max", "Control", 0, 255, nothing)
    cv.createTrackbar("S_min", "Control", 0, 255, nothing)
    cv.createTrackbar("S_max", "Control", 0, 255, nothing)
    cv.createTrackbar("V_min", "Control", 0, 255, nothing)
    cv.createTrackbar("V_max", "Control", 0, 255, nothing)

    global red_h_min, red_h_max, red_s_min, red_s_max, red_v_min, red_v_max

    cv.setTrackbarPos("H_min", "Control", red_h_min)
    cv.setTrackbarPos("H_max", "Control", red_h_max)
    cv.setTrackbarPos("S_min", "Control", red_s_min)
    cv.setTrackbarPos("S_max", "Control", red_s_max)
    cv.setTrackbarPos("V_min", "Control", red_v_min)
    cv.setTrackbarPos("V_max", "Control", red_v_max)

    image_iter = 540

    while not rospy.is_shutdown():
        global rgb_img
        image_file = "/home/tinker/catkin_ws/dataset/marathon/line_images/image"+str(image_iter)+".png"
        rgb_img = cv.imread(image_file)
        result_img = rgb_img.copy()
        img_h, img_w, _ = rgb_img.shape
        red_h_min = cv.getTrackbarPos("H_min", "Control")
        red_h_max = cv.getTrackbarPos("H_max", "Control")
        red_s_min = cv.getTrackbarPos("S_min", "Control")
        red_s_max = cv.getTrackbarPos("S_max", "Control")
        red_v_min = cv.getTrackbarPos("V_min", "Control")
        red_v_max = cv.getTrackbarPos("V_max", "Control")

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

                    box_x, box_y, box_w, box_h = cv.boundingRect(line_cntr)
                    # Update ukuran ke image sebenarnya
                    box_y = i * roi_height + box_y
                    box_centre_x = box_x + box_w / 2
                    box_centre_y = box_y + box_h / 2
                    cv.rectangle(result_img, (box_x, box_y), (box_x + box_w, box_y + box_h), contour_color, 1)
                    centre_dot = (box_centre_x, box_centre_y)
                    cv.circle(result_img, centre_dot, 5, (255, 0, 0), -1)
                    list_centre_dot.append(centre_dot)
        # ini di cek len centre dot nya
        if len(list_centre_dot) > 1:
            cv.arrowedLine(result_img, list_centre_dot[-1], list_centre_dot[0], arrow_color, 2)
            x1, y1 = list_centre_dot[-1]
            x2, y2 = list_centre_dot[0]

            c_dot = (x2, y1)
            cv.circle(result_img, c_dot, 5, (255, 0, 0), -1)
            h_length = np.linalg.norm(np.array(list_centre_dot[-1])-np.array(list_centre_dot[0]))
            a_length = np.linalg.norm(np.array(list_centre_dot[0])-np.array(c_dot))
    
            angle_sign = 1
            if x2 >= x1:
                angle_sign = 1
            elif x2 < x1:
                angle_sign = -1
            angle = (90 - (math.asin(a_length / h_length) * 180 / math.pi)) * angle_sign
            
        angle_msg = Float32
        angle_msg = angle
        rospy.loginfo("Angle = "+str(angle))
   
        list_centre_dot = None
        line_angle_pub.publish(angle_msg)
        line_bin_img_pub.publish(bridge.cv2_to_imgmsg(red_bin_img, "mono8"))
        line_rslt_img_pub.publish(bridge.cv2_to_imgmsg(result_img, "bgr8"))

        k = cv.waitKey(1)

        if k == ord("x"):
            break
        elif k == ord("n"):
            image_iter += 1
            if image_iter > 1191:
                image_iter = 0
        elif k == ord("p"):
            image_iter -= 1
            if image_iter < 0:
                image_iter = 1191

        rate.sleep()

    # cap.release()
    cv.destroyAllWindows()
    rospy.loginfo("Marathon Line Detector - Shutdown")

if __name__ == "__main__":
	main()
