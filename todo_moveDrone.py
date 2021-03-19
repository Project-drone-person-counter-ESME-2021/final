#!/usr/bin/env python
from __future__ import print_function

from typing import Any, Union

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import urllib
import numpy as np
from sensor_msgs.msg import Image
import roslib
import argparse

# import file
import function


class MoveDrone:
    def __init__(self):
        print('start')
        print("after subriscriber camera")
        self.takeoff_pub = rospy.Publisher("/ardrone/takeoff", Empty,
                                           queue_size=100)  # TODO put the takeoff topic name here
        self.landing_pub = rospy.Publisher("/ardrone/land", Empty,
                                           queue_size=100)  # TODO put the landing topic name here
        self.move_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)  # Publish commands to drone
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/ardrone/front/image_raw", Image, self.callback)

	self.first_img_taken = False

    def move_drone(self, speed=[0.0, 0.0, 0.0], orient=[0.0, 0.0, 0.0]):
        vel_msg = Twist()
        # TODO: fill the velocity fields here with the desired values
        vel_msg.linear.x = speed[0]
        vel_msg.linear.y = speed[1]
        vel_msg.linear.z = speed[2]
        # TODO: fill the angulare velocities here with the desired values
        vel_msg.angular.x = orient[0]
        vel_msg.angular.y = orient[1]
        vel_msg.angular.z = orient[2]
        self.move_pub.publish(vel_msg)
        return 0

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	    self.first_img_taken = True
        except CvBridgeError as e:
            print("error")
            print(e)

    def get_first_img_taken(self):
	return self.first_img_taken

    def take_a_screenshot(self):
        if self.first_img_taken:
            print("new image")
            return self.cv_image
        else:
            print("no image")
            return 0

    def takeoff_drone(self):
        empty_msg = Empty()
        self.takeoff_pub.publish(empty_msg)

    def land_drone(self):
        empty_msg = Empty()
        self.landing_pub.publish(empty_msg)


# keep it for now
def main(args):
    # ic = MoveDrone()
    print('OK')
    rospy.init_node('MoveDrone', anonymous=True)
    print('OK')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

def face_count(img) :
    face_cascade = cv2.CascadeClassifier('/home/yannb/Documents/dronetp/src/my_package/scripts/haarcascade_frontalface_default.xml')

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 1)
    face_count = len(faces)

    return face_count

if __name__ == '__main__':
    # keep it for now
    # main(sys.argv)
    # rospy.init_node('basic_controller', anonymous=True)
    # publisher_takeoff = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=100)
    rospy.init_node('MoveDrone', anonymous=True)
    move = MoveDrone()
    # TODO define your time counter here !
    # publisher_landing = rospy.Publisher("/ardrone/land", Empty, queue_size=100)
    # empty_msg = Empty()

    print('take off')
    t1 = time.time()
    while (time.time() - t1 < 10):
        move.takeoff_drone()

    print("Rotating")
    t1 = time.time()

    # define time to rotate and number of pictures
    nb_pictures = 8.0
    nb_secondes = 1.0

    index_take_picture = 0

    list_img_concat = []

    is_image = False

    for index in range(int(nb_pictures)):
        t1 = time.time()
        while time.time() - t1 < nb_secondes / nb_pictures:
            move.move_drone(orient=[0.0, 0.0, -1.0])

	t1 = time.time()
	while time.time()- t1 < 1:
		move.move_drone()

	img = move.take_a_screenshot()

	if not(move.get_first_img_taken()):
	    # if no picture is send from the function
	    print("error no picture")
	elif index_take_picture == 0:
	    # first iteration store picture to be the first right image
	    left_img = img
	    index_take_picture += 1

	    is_image = True
	else:
	    # get the right img
	    right_img = img

	    # concat two img
	    verif, concat_img = function.concat(left_img, right_img)

	    if verif:
		# concat_img become left_img
		left_img = concat_img
	    else:
		list_img_concat.append(left_img)
		left_img = right_img

	    #index_take_picture += 1

    if is_image == True:
	list_img_concat.append(left_img)
    else :
	print("no imsage")

    print("Hovering")
    t1 = time.time()
    while (time.time() - t1 < 5):
        move.move_drone()

    print("landing...")
    t1 = time.time()
    while (time.time() - t1 < 5):
        move.land_drone()

    print("Do you wish to save the image(s) \nY : yes \nanythings else : no")

    verify_save = False

    tmp_input = raw_input()
    if tmp_input == "Y":
        verify_save = True

    index = 1
    nb_faces = 0

    print("lenght list image : {}".format(len(list_img_concat)))

    for img in list_img_concat:
        cv2.imshow("test " + str(index), img)
	nb_faces += face_count(img)
	print("picture n {}, nb of faces : {}".format(index, nb_faces))
        if verify_save:
	    path = "/home/yannb/Pictures/result/image_" + str(index) + ".png"
            cv2.imwrite(path, img)
        index += 1
    print("number of face : {}".format(nb_faces))
    cv2.waitKey(0)
    cv2.destroyAllWindows()


