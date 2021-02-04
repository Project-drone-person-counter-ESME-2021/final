#!/usr/bin/env python
from __future__ import print_function
import roslib
#roslib.load_manifest('my_package')
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


class MoveDrone:
	def __init__(self):
		print('start')
		print("after subriscriber camera")
 		self.takeoff_pub = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=100) # TODO put the takeoff topic name here
 		self.landing_pub = rospy.Publisher("/ardrone/land", Empty, queue_size=100) # TODO put the landing topic name here
 		self.move_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100) # Publish commands to drone 
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/ardrone/front/image_raw",Image,self.callback)
		self.last_image = 0


	def move_drone(self, speed=[0.0, 0.0, 0.0], orient=[0.0, 0.0, 0.0]):
 		vel_msg = Twist()
 		# TODO: fill the velocity fields here with the desired values
 		vel_msg.linear.x = speed[0]
 		vel_msg.linear.y = speed[1]
 		vel_msg.linear.z = speed[2]
 		#TODO: fill the angulare velocities here with the desired values
 		vel_msg.angular.x = orient[0]
 		vel_msg.angular.y = orient[1]
 		vel_msg.angular.z = orient[2]
 		self.move_pub.publish(vel_msg)
 		return 0

 		
 	def callback(self,data):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print("error")
			print(e)
		
		#(rows,cols,channels) = cv_image.shape
		#if cols > 60 and rows > 60 :
		#	cv2.circle(cv_image, (50,50), 10, 255)

		#self.last_image = cv_image

		#cv2.imshow("Image window", self.last_image)

		#cv2.waitKey(3)

 	# def get_rotation(self, msg):
	 #    global roll, pitch, yaw
	 #    orientation_q = msg.pose.pose.orientation
	 #    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	 #    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	 #    print(yaw)


	def take_a_screenshot(self, name="test"):
		if not(isinstance(self.cv_image, int)):
			print("new image")
			cv2.imshow(name, self.cv_image)
		else :
			print("no image")
		

 	def takeoff_drone(self):
 		empty_msg = Empty()
 		self.takeoff_pub.publish(empty_msg)



 	def land_drone(self):
 		empty_msg = Empty()
 		self.landing_pub.publish(empty_msg)

def main(args):
	#ic = MoveDrone()
	print('OK')
	rospy.init_node('MoveDrone', anonymous=True)
	print('OK')
	try:
	  rospy.spin()
	except KeyboardInterrupt:
	  print("Shutting down")

class IPCamera(object):
    def __init__(self, url):
        try:
            self.stream=urllib.urlopen(url)
        except:
            rospy.logerr('Unable to open camera stream: ' + str(url))
            sys.exit() #'Unable to open camera stream')
        self.bytes=''
        self.image_pub = rospy.Publisher("camera_image", Image)
        self.bridge = CvBridge()



bridge_c = CvBridge()

def take_screenshoot(name="test"):

	data = rospy.wait_for_message("/ardrone/front/image_raw", Image)

	print("opencv outside")
	try:
		cv_image_test = bridge_c.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print("error")
		print(e)

	cv2.imshow(name, cv_image_test)

	print("opencv outside finsish")


if __name__ == '__main__':
	"""

	print('test1')
	parser = argparse.ArgumentParser(prog='todo_moveDrone.py', description='reads a given url string and dumps it to a ros_image topic')
	parser.add_argument('-g', '--gui', action='store_true', help='show a GUI of the camera stream')
	parser.add_argument('-u', '--url', default='http://admin:admin@10.68.68.22/goform/video?channel=1&.mjpg', help='camera stream url to parse')
	args = parser.parse_args()
	print(args.url)
	#rospy.init_node('IPCamera', anonymous=True)
	ip_camera = IPCamera(args.url)
	print('test2')
	ip_camera.bytes += ip_camera.stream.read(1024)

	a = ip_camera.bytes.find('\xff\xd8')
	b = ip_camera.bytes.find('\xff\xd9')
	
	if a!=-1 and b!=-1:
	    jpg = ip_camera.bytes[a:b+2]
	    ip_camera.bytes= ip_camera.bytes[b+2:]
	    i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
	    image_message = cv.fromarray(i)
	    ip_camera.image_pub.publish(ip_camera.bridge.cv_to_imgmsg(image_message, "bgr8"))

	    if args.gui:
		cv2.imshow('IP Camera Publisher Cam',i)
	    if cv2.waitKey(1) ==27: # wait until ESC key is pressed in the GUI window to stop it
		exit(0) 

	"""
	
	#main(sys.argv)
	#rospy.init_node('basic_controller', anonymous=True)
	#publisher_takeoff = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=100)
	rospy.init_node('MoveDrone', anonymous=True)	
	move = MoveDrone()	
	# TODO define your time counter here !
	#publisher_landing = rospy.Publisher("/ardrone/land", Empty, queue_size=100)
	#empty_msg = Empty()

	"""

	Bridge_test = CvBridge()

	try:
		cv_image = Bridge_test.imgmsg_to_cv2("/ardrone/front/image_raw", "bgr8")
	except CvBridgeError as e:
		print(e)

	(rows,cols,channels) = cv_image.shape
	if cols > 60 and rows > 60 :
		cv2.circle(cv_image, (50,50), 10, 255)

	cv2.imshow("Image window", cv_image)
	cv2.waitKey(3)

	"""
	

	print('take off')
	#print(time.time()-t1)
	t1 = time.time()
	while(time.time()-t1 < 10):
		# print(time.time()-t1)
		move.takeoff_drone()

	#move.take_a_screenshot("Before")

	print("Rotating")
	t1 = time.time()
	
	nb_pictures = 5
	nb_secondes = 10

	for index in range(5):
		t1 = time.time()
		while(time.time()-t1 < nb_secondes / nb_pictures):
			
			move.move_drone(orient=[0.0, 0.0, -1.0])

		move.take_a_screenshot("stand by, index  : {}".format(index))

	print("Hovering")
	t1 = time.time()
	while(time.time()-t1 < 5):
		move.move_drone()

	print("landing...")
	t1 = time.time()
	while(time.time()-t1 < 5):
		move.land_drone()

	cv2.waitKey(0)
	
