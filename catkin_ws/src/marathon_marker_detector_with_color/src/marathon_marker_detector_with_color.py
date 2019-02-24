import numpy as np 
import cv2 as cv
import matplotlib.pyplot as plt 
from scipy.spatial import distance

black_h_min = 45
black_h_max = 255
black_s_min = 0
black_s_max = 100
black_v_min = 0
black_v_max = 120

def update_parameter(x):
    pass

def cosine_similarity(x, y):
    cos = np.dot(x, y) / (np.sqrt(np.dot(x,x)) * np.sqrt(np.dot(y,y)))
    return cos

def main():
    print("Hello")

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

    forward_file = "/home/tinker/catkin_ws/dataset/marathon/marker_images/template/forward.jpg"
    right_file = "/home/tinker/catkin_ws/dataset/marathon/marker_images/template/turn_right.jpg"
    left_file = "/home/tinker/catkin_ws/dataset/marathon/marker_images/template/turn_left.jpg"

    forward_template = cv.imread(forward_file, 0)
    right_template = cv.imread(right_file, 0)
    left_template = cv.imread(left_file, 0)

    forward_template = cv.resize(forward_template, (50, 50))
    right_template = cv.resize(right_template, (50, 50))
    left_template = cv.resize(left_template, (50, 50))

    forward_vector = forward_template.copy().reshape((2500))
    right_vector = right_template.copy().reshape((2500))
    left_vector = left_template.copy().reshape((2500))

    # print(forward_vector.shape)

    while True:
        test_file = "/home/tinker/catkin_ws/dataset/marathon/marker_images/forward/forward (1).jpg"
        rgb_image = cv.imread(test_file)
        gray_image = cv.cvtColor(rgb_image, cv.COLOR_BGR2GRAY)
        result_image = rgb_image.copy()
        hsv_image = cv.cvtColor(rgb_image, cv.COLOR_BGR2HSV)
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

        sorted_contours = []
        marker_roi = None
        contour_color = (0, 255, 0)
        if len(marker_contours) > 0:
            sorted_marker_contours = sorted(marker_contours, key=cv.contourArea, reverse=True)[:3]
            marker_cntr = sorted_marker_contours[0]
            box_x, box_y, box_w, box_h = cv.boundingRect(marker_cntr)
            cv.rectangle(result_image, (box_x, box_y), (box_x + box_w, box_y + box_h), contour_color, 1)

            marker_roi = gray_image[box_y:box_y + box_h, box_x:box_x + box_w]
            marker_roi = cv.resize(marker_roi, (50, 50))
            marker_roi_vector = marker_roi.copy().reshape((2500))
            # forward_sim = distance.cosine([10,10,5,5], [10,10,5,5])

            forward_sim = distance.cosine(forward_vector, marker_roi_vector)
            right_sim = distance.cosine(right_vector, marker_roi_vector)
            left_sim = distance.cosine(left_vector, marker_roi_vector)
            print("Forward : " + str(forward_sim))
            print("Right : " + str(right_sim))
            print("Left : " + str(left_sim))
            # hst_val = cv.calcHist([marker_roi], [0], None, [3], [0, 256])
            # hst_val_0, hst_val_1, hst_val_2 = hst_val
            # sum_hst_val = hst_val_0 + hst_val_1 + hst_val_2
            # hst_val_0 = hst_val_0 / sum_hst_val
            # hst_val_1 = hst_val_1 / sum_hst_val
            # hst_val_2 = hst_val_2 / sum_hst_val
            # print("hst_val_0" + str(hst_val_0))
            # print("hst_val_1" + str(hst_val_1))
            # print("hst_val_2" + str(hst_val_2))

        
        cv.imshow("ROI", marker_roi)
        # cv.imshow("Gambar", result_image)
        # cv.imshow("Binary", binary_black)
        cv.imshow("Forward", forward_template)
        cv.imshow("Left", left_template)
        cv.imshow("Right", right_template)
        cv.waitKey(1)
if __name__ == "__main__":
    main()