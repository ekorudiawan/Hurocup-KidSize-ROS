import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

def main():
    test_file = "/home/tinker/catkin_ws/dataset/marathon/marker_images/right/right (1).png"
    img = cv.imread(test_file,0)
    img2 = img.copy()
    forward_template = "/home/tinker/catkin_ws/dataset/marathon/marker_images/template/forward.png"
    right_template = "/home/tinker/catkin_ws/dataset/marathon/marker_images/template/right.png"
    left_template = "/home/tinker/catkin_ws/dataset/marathon/marker_images/template/left.png"
    template = cv.imread(left_template,0)
    w, h = template.shape[::-1]

    method = cv.TM_SQDIFF_NORMED
    res = cv.matchTemplate(img,template,method)

    print(res)
    min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
    
    if method in [cv.TM_SQDIFF, cv.TM_SQDIFF_NORMED]:
        top_left = min_loc
    else:
        top_left = max_loc
    bottom_right = (top_left[0] + w, top_left[1] + h)

    cv.rectangle(img,top_left, bottom_right, 255, 2)

    # plt.subplot(121)
    # plt.imshow(res,cmap = 'gray')
    # plt.title('Matching Result')
    # plt.xticks([])
    # plt.yticks([])
    # plt.subplot(122)
    plt.imshow(img,cmap = 'gray')
    plt.title('Detected Point')
    # plt.xticks([])
    # plt.yticks([])
    # plt.suptitle(meth)

    plt.show()

if __name__ == "__main__":
    main()