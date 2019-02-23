import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

def main():
    # FLANN parameters
    FLANN_INDEX_LSH = 6
    index_params= dict(algorithm = FLANN_INDEX_LSH,
                    table_number = 6, # 12
                    key_size = 16,     # 20
                    multi_probe_level = 1) #2
    search_params = dict(checks=50)   # or pass empty dictionary

    flann = cv.FlannBasedMatcher(index_params,search_params)

    orb = cv.ORB_create()
    forward_template = "/home/tinker/catkin_ws/dataset/marathon/marker_images/template/forward.png"
    right_template = "/home/tinker/catkin_ws/dataset/marathon/marker_images/template/right.png"
    left_template = "/home/tinker/catkin_ws/dataset/marathon/marker_images/template/left.png"
    iteration = 1
    while True:
        test_file = "/home/tinker/catkin_ws/dataset/marathon/marker_images/right/right ("+str(iteration)+").png"
        # test_file = "/home/tinker/catkin_ws/dataset/marathon/marker_images/line/line (8).png"
        forward_img = cv.imread(forward_template, 0)
        right_img = cv.imread(right_template, 0)
        left_img = cv.imread(left_template, 0)

        test_img = cv.imread(test_file, 0)

        kp_forward, des_forward = orb.detectAndCompute(forward_img, None)
        kp_left, des_left = orb.detectAndCompute(left_img, None)
        kp_right, des_right = orb.detectAndCompute(right_img, None)
        kp_test, des_test = orb.detectAndCompute(test_img, None)

        right_matches = []
        right_matches_mask = []
        total_right_match = 0
        if des_right is not None and des_test is not None:
            right_matches = flann.knnMatch(des_right, des_test, k=2)
            right_matches_mask = [[0,0] for i in range(len(right_matches))]
            for i in range(len(right_matches)):
                if len(right_matches[i]) > 1:
                    m, n = right_matches[i]
                    if m.distance < 0.9*n.distance:
                        right_matches_mask[i]=[1,0]
                        total_right_match += 1

        draw_params = dict(matchColor = (0,255,0),
                        singlePointColor = (255,0,0),
                        matchesMask = right_matches_mask,
                        flags = cv.DrawMatchesFlags_DEFAULT)
        right_result = cv.drawMatchesKnn(right_img,kp_right,test_img,kp_test,right_matches,None,**draw_params)
        cv.imshow("right_result", right_result)

        # Left
        left_matches = []
        left_matches_mask = []
        total_left_match = 0
        if des_left is not None and des_test is not None:
            left_matches = flann.knnMatch(des_left, des_test, k=2)
            left_matches_mask = [[0,0] for i in range(len(left_matches))]
            for i in range(len(left_matches)):
                if len(left_matches[i]) > 1:
                    m, n = left_matches[i]
                    if m.distance < 0.9*n.distance:
                        left_matches_mask[i]=[1,0]
                        total_left_match += 1

        draw_params = dict(matchColor = (0,255,0),
                        singlePointColor = (255,0,0),
                        matchesMask = left_matches_mask,
                        flags = cv.DrawMatchesFlags_DEFAULT)
        left_result = cv.drawMatchesKnn(left_img,kp_left,test_img,kp_test,left_matches,None,**draw_params)
        cv.imshow("left_result", left_result)

        # Forward
        forward_matches = []
        forward_matches_mask = []
        total_forward_match = 0
        if des_forward is not None and des_test is not None:
            forward_matches = flann.knnMatch(des_forward, des_test, k=2)
            forward_matches_mask = [[0,0] for i in range(len(forward_matches))]
            for i in range(len(forward_matches)):
                if len(forward_matches[i]) > 1:
                    m, n = forward_matches[i]
                    if m.distance < 0.9*n.distance:
                        forward_matches_mask[i]=[1,0]
                        total_forward_match += 1

        draw_params = dict(matchColor = (0,255,0),
                        singlePointColor = (255,0,0),
                        matchesMask = forward_matches_mask,
                        flags = cv.DrawMatchesFlags_DEFAULT)
        forward_result = cv.drawMatchesKnn(forward_img,kp_forward,test_img,kp_test,forward_matches,None,**draw_params)
        cv.imshow("forward_result", forward_result)

        match_result = [total_forward_match, total_right_match, total_left_match]
        max_idx = match_result.index(max(match_result))

        if (max_idx == 0):
            print("Forward")
        elif (max_idx == 1):
            print("Right")
        elif (max_idx == 2):
            print("Left")

        # if total_left_match > total_right_match:
        #     print("Kiri")
        # else:
        #     print("Kanan")

        k = cv.waitKey(1)
        if k == 27:
            break
        elif k == ord('n'):
            iteration += 1

if __name__ == "__main__":
    main()