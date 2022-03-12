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

		self.tlx = 0
		self.tly = 0
		self.brx = 0
		self.bry = 0
		self.depth = 0

		self.bound = []
		self.z = []

		rospy.Subscriber(self.color_topic, Image, self.getColorCallback)
		rospy.Subscriber(self.bound_topic, obstacle_bound, self.boundingCallback)

		while(not rospy.is_shutdown()):
			try:
				self.show()
			except:
				pass


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
		self.tlx = msg.top_left_x
		self.tly = msg.top_left_y
		self.brx = msg.bottom_right_x
		self.bry = msg.bottom_right_y
		self.depth = msg.distance

		# print('msg in: ',self.tlx,self.tly,self.brx,self.bry,self.depth)

		self.show()
		
	def show(self):
		if(not -1 in [self.tlx,self.tly,self.brx,self.bry]):
			if(not -99 in [self.tlx,self.tly,self.brx,self.bry]):
				self.bound.append([self.tlx,self.tly,self.brx,self.bry])
				self.z.append(self.depth)
			else:
				for i in range(len(self.bound)):
					cv2.rectangle(self.color_img, (self.bound[i][0], self.bound[i][1]), (self.bound[i][2], self.bound[i][3]), (0,255,0), 2)
					text = 'obstacle at %.2f m' %(self.z[i])
					cv2.putText(self.color_img, text, (self.bound[i][0] - 5, self.bound[i][1] - 5), 0, 0.3, (0,255,0))
				self.bound = []
				self.z = []

		cv2.imshow("Result", self.color_img)
		cv2.waitKey(1)  

if __name__ == '__main__':
	listen = viewer()
	rospy.spin()