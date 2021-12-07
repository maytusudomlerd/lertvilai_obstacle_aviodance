#!/usr/bin/env python2.7

import rospy

from sensor_msgs.msg import Image
from detection.msg import obstacle_bound
from cv_bridge import CvBridge, CvBridgeError

import cv2

import numpy as np
import random 

class viewer:
	
	def __init__(self):
		rospy.init_node("realsense_viewer")
		self.color_topic = '/camera/color/image_raw'
		self.bound_topic = '/detection/obstacle/bound'
		self.bridge = CvBridge()
		self.color_img = np.array([])
		self.depth_img = np.array([])
		
		self.points = []

		rospy.Subscriber(self.color_topic, Image, self.getColorCallback)
		rospy.Subscriber(self.bound_topic, obstacle_bound, self.boundingCallback)



	def sub_color(self):
		rospy.Subscriber(self.color_topic, Image, self.getColorCallback)
	
	def sub_depth(self):
		rospy.Subscriber(self.depth_topic, Image, self.getDepthCallback)

	def sub_obstacle_bound(self):
		rospy.Subscriber(self.depth_topic, Image, self.getDepthCallback)


	def getColorCallback(self, msg):

		try:
			cv2_cimg = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
			self.color_img = cv2_cimg
		
		except Exception as e:
			print(e)

	def getDepthCallback(self, msg):

		try:
			cv2_dimg = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
			self.depth_img = cv2_dimg
			# print(cv2_dimg.shape)

			depth3d = np.dstack((cv2_dimg,cv2_dimg,cv2_dimg))
			depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth3d, alpha=0.03), cv2.COLORMAP_JET)
			show = np.hstack((depth_colormap,self.color_img))
			cv2.imshow('depth',show)
			cv2.waitKey(1)
			# print(self.depth_img.shape)   
		
		except Exception as e:
			print(e)

	def boundingCallback(self,msg):
		tlx = msg.top_left_x
		tly = msg.top_left_y
		brx = msg.bottom_right_x
		bry = msg.bottom_right_y
		depth = msg.distance

		#using cv2
		cv2.rectangle(self.color_img, (tlx, tly), (brx, bry), (0,255,0), 2)
		text = 'obstacle at %.2f m' %(depth)
		cv2.putText(self.color_img, text, (tlx - 5, tly - 5), 0, 0.3, (0,255,0))

		cv2.imshow("Result", self.color_img)
		cv2.waitKey(1)  

if __name__ == '__main__':
	listen = viewer()
	rospy.spin()