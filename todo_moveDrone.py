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
import propre


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
        except CvBridgeError as e:
            print("error")
            print(e)

    def take_a_screenshot(self):
        if not (isinstance(self.cv_image, int)):
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
    nb_pictures = 5
    nb_secondes = 10

    for index in range(5):
        t1 = time.time()
        while time.time() - t1 < nb_secondes / nb_pictures:
            move.move_drone(orient=[0.0, 0.0, -1.0])

        img = move.take_a_screenshot()

        if not(isinstance(img, int)):
            # if no picture is send from the function
            print("error no picture")
        elif index == 0:
            # first iteration store picture to be the first right image
            left_img = img
        else:
            # get the right img
            right_img = img

            # concat two img
            concat_img = propre.concat(left_img, right_img)

            # concat_img become left_img
            left_img = concat_img


    print("Hovering")
    t1 = time.time()
    while (time.time() - t1 < 5):
        move.move_drone()

    print("landing...")
    t1 = time.time()
    while (time.time() - t1 < 5):
        move.land_drone()

    cv2.imshow("concat image", concat_img)

    cv2.waitKey(0)
