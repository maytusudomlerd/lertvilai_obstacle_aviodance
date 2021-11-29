#!/usr/bin/env python2.7

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import cv2

import numpy as np
import random 

class viewer:
	
	def __init__(self):
		rospy.init_node("realsense_viewer")
		self.color_topic = '/camera/color/image_raw'
		self.depth_topic = '/camera/depth/image_raw'
		self.pointcloud_topic = '/camera/depth/color/points'
		self.bridge = CvBridge()
		self.color_img = np.array([])
		self.depth_img = np.array([])
		
		self.points = []


	def sub_color(self):
		rospy.Subscriber(self.color_topic, Image, self.getColorCallback)
	
	def sub_depth(self):
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

if __name__ == '__main__':
	
	listen = viewer()
	listen.sub_depth()
	listen.sub_color()
	rospy.spin()