import cv2
import numpy as np
import matplotlib.pyplot as plt
import os


def trim(frame):
    # crop top
    if not np.sum(frame[0]):
        return trim(frame[1:])
    # crop top
    if not np.sum(frame[-1]):
        return trim(frame[:-2])
    # crop top
    if not np.sum(frame[:, 0]):
        return trim(frame[:, 1:])
    # crop top
    if not np.sum(frame[:, -1]):
        return trim(frame[:, :-2])
    return frame


def convert_to_img_cv2(img_):
    img_resize = cv2.resize(img_, (0, 0), fx=1, fy=1)
    img_cvt_color = cv2.cvtColor(img_resize, cv2.COLOR_BGR2GRAY)

    return img_cvt_color


def concat_2_img(img_left, img_left_cvt, img_right, img_right_cvt):

    """
    :param img_left: image left
    :param img_left_cvt: image left cvt color
    :param img_right: image right
    :param img_right_cvt: image right cvt color
    :return: concat img of of the image
    """

    # function to detect key point on img
    sift = cv2.xfeatures2d.SIFT_create()
    # find key points
    kp1, des1 = sift.detectAndCompute(img_right_cvt, None)
    kp2, des2 = sift.detectAndCompute(img_left_cvt, None)

    # match key point in that are the same in the 2 img
    match = cv2.BFMatcher()
    matches = match.knnMatch(des1, des2, k=2)

    good = []

    print("number of matches between 2 photos : {}".format(len(matches)))

    for m, n in matches:
        # print("test m.distance({}) < 0.03 * n.distance({})".format(m.distance, 0.03 * n.distance))
        if m.distance < (0.3 * n.distance):
            good.append(m)

    min_match_count = 10

    if len(good) > min_match_count:
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        h, w = img_right_cvt.shape
        pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
    else:
        draw_params = dict(matchColor=(0, 255, 0), singlePointColor=None, flags=2)

        img3 = cv2.drawMatches(img_right, kp1, img_left, kp2, good, None, **draw_params)
        cv2.imshow("original_image_drawMatches.jpg", img3)

        print "Not enought matches are found - {} / {}, {}" \
              "".format(len(good), min_match_count, len(good) / min_match_count)
        plt.show()
        cv2.waitKey(0)

        return False, np.zeros(shape=(50, 50, 3))

    dst = cv2.warpPerspective(img_right, M, (img_left.shape[1] + img_right.shape[1], img_left.shape[0]))
    dst[0:img_left.shape[0], 0:img_left.shape[1]] = img_left

    return True, trim(dst)


def concat(img_left, img_right):
    img_left_cvt = convert_to_img_cv2(img_left)
    img_right_cvt = convert_to_img_cv2(img_right)

    verif, img = concat_2_img(img_left, img_left_cvt, img_right, img_right_cvt)

    # return concat img
    return verif, img


# use to debug
if __name__ == "__main__":

    print("peter pan")

    # cvt color

    # concat