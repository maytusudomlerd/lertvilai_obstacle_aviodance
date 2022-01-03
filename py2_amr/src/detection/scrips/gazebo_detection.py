#!/usr/bin/env python3.6

import rospy

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

from detection.msg import obstacle_bound


import numpy as np
import ros_numpy
import random 
import time

import csv

import pyrealsense2 as rs
import open3d as o3d
import random
import math
from matplotlib import pyplot as plt

import open3d.visualization.gui as gui

from sklearn.cluster import DBSCAN
from sklearn.cluster import KMeans

class obstacleDetection:
	
	def __init__(self):

		# init camera
		self.initial_camera=True
		self.depth_scale = 0.0
		self.pipeline = None

		# camera config
		self.w = 640
		self.h = 480
		# self.w = 320
		# self.h = 240
		# self.w = 160
		# self.h = 120
		
		self.hfov_deg_d = 85.2 * np.pi /180
		self.vfov_deg_d = 58.0 * np.pi /180
		self.hfov_deg_c = 69.4 * np.pi /180
		self.vfov_deg_c = 42.0 * np.pi /180

		self.rgb_img = np.array([])

		# ros
		self.color_topic = '/camera/color/image_raw'
		self.pointcloud_topic = '/camera/depth/color/points'
		self.bridge = CvBridge()

		# pointcloud object
		self.pc = o3d.geometry.PointCloud()
		
		self.origin = [0,0,0]
		self.axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=self.origin)

		self.points = []
		self.pc_size = 0

		self.dataprocessing_param = {
			'voxelize': True,
			'voxel_size': 0.05
		}
		# initial Segmentation Param
		self.initial_seg_param = {
			'estimate_normal' : True,
			'normals_neigh' : 30,
			'eps' : 0.06,
			'min_sample' : 3,
			'debug' : False
		}
		# split param 
		self.split_param = {
			# 'eps': 0.06,
			# 'min_sample' : 6,
			'eps': 0.1,
			'min_sample' : 3,
			'debug' : False
		}
		# Merge param
		self.merge_param = {
			'distanceFromOrigin_tol':0.5,
			'distanceBetweenPlane_th':0.7,
			'debug':False,
			'bounding': False
		}

		self.clusters = {'point': [] }
		self.merge = []

		self.timestamp = []
		self.processtime = 0.0
		self.start_time_flag = True
		self.start_time = 0

		self.iterations = 0

		self.source = 'ros'
		self.initial_camera=True
		self.pipeline = None

		# initialze
		rospy.init_node("detection")
		self.pointcloud_pub = rospy.Publisher("detection/result", PointCloud2,queue_size=1)
		self.obstacle_bound_pub = rospy.Publisher('detection/obstacle/bound',obstacle_bound,queue_size=1)
		rospy.Rate(30)

		if(self.source == 'ros'):
			# rospy.Subscriber(self.color_topic, Image, self.getColorCallback)
			rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.getPointcloudCallback)
		
		if(self.source == 'camera'):
			# run detection
			if(self.initial_camera):
			  self.initcamera(havebag=True)
			  self.initial_camera = False
			while(not rospy.is_shutdown()):
			  self.get_frames()

	def distance_point(self,xyz, xyz1):
		try:
			x = xyz[0:, 0]
			y = xyz[0:, 1]
			z = xyz[0:, 2]
			x1 = xyz1[0:, 0]
			y1 = xyz1[0:, 1]
			z1 = xyz1[0:, 2]
		except:
			x, y, z = xyz
			x1 ,y1, z1 = xyz1
	
		return np.sqrt((x-x1)**2+(y-y1)**2+(z-z1)**2)

	def SVD(self,X):
		"""
		Singular value decomposition method.
		Source: https://gist.github.com/lambdalisue/7201028
		"""
		# Find the average of points (centroid) along the columns
		C = np.mean(X, axis=0)

		# Create CX vector (centroid to point) matrix
		CX = X - C
		# Singular value decomposition
		U, S, V = np.linalg.svd(CX)

		return U,S,V

	def MSE(self,pc,S):
		mean, cov = pc.compute_mean_and_covariance()
		covof3axis = cov[0][0] + cov[1][1] + cov[2][2]
		mse = covof3axis - sum(S)
		return mse

	def getNormal(self,V):
		return V[-1]

	def getcoefofplane(self,pc):
		U,S,V = self.SVD(pc.points)
		n = self.getNormal(V)
		C = pc.get_center()
		x0,y0,z0 = C
		a,b,c = n
		d = -(a * x0 + b * y0 + c * z0)
		return a,b,c,d

	def angleBetweenPlane(self,coef0,coef):
		a0,b0,c0 = coef0
		a,b,c = coef
		norm0 = np.sqrt(a0**2 + b0**2 + c0**2)
		norm = np.sqrt(a**2 + b**2 + c**2)
		angle = np.abs(a0*a + b0*b + c0*c)/norm0*norm
		return angle * 180/np.pi


	def candidate_len_sort(self):
		n = len(self.clusters['len'])
		for i in range(n):
			for j in range(0, n - i - 1):
				if self.clusters['len'][j] < self.clusters['len'][j + 1]:
					self.clusters['point'][j], self.clusters['point'][j + 1] = self.clusters['point'][j + 1], self.clusters['point'][j]
					self.clusters['centroid'][j], self.clusters['centroid'][j + 1] = self.clusters['centroid'][j + 1], self.clusters['centroid'][j]
					self.clusters['model'][j], self.clusters['model'][j + 1] = self.clusters['model'][j + 1], self.clusters['model'][j]
					self.clusters['distanceFromOrigin'][j], self.clusters['distanceFromOrigin'][j + 1] = self.clusters['distanceFromOrigin'][
																								 j + 1], \
																							 self.clusters['distanceFromOrigin'][
																								 j]
					self.clusters['len'][j], self.clusters['len'][j + 1] = self.clusters['len'][j + 1], self.clusters['len'][j]

	def add_timestamp(self):

		now = time.time() - self.processtime
		self.processtime = self.processtime+now
		self.timestamp.append(format(now, ".4f"))

	def addtoCSV(self):
		with open('/home/maytus/internship_main/experiment/computation/simulations/computing_time.csv', 'a') as c:
			write = csv.writer(c)
			self.timestamp.insert(1,self.pc_size)
			write.writerows([self.timestamp])

	def pointcloudpub(self):
		rospy.loginfo("[INFO] Publishing")
		pcloud = PointCloud2()
		point = None
		point = self.merge.pop(0)
		for m in self.merge:
			point += m
		point.transform([[0, 0, 1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])

		# o3d.visualization.draw_geometries([self.axis]+[point])

		result = np.array(point.points).tolist()
		pcloud.header.frame_id = 'camera_link'
		pcloud.header.stamp = rospy.Time.now()
		pcloud = pc2.create_cloud_xyz32(pcloud.header, result)

		self.pointcloud_pub.publish(pcloud)
		print('[finish]')

 	
	def sub_pointcloud(self):
		rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.getPointcloudCallback)

	def getPointcloudCallback(self,msg):

		self.points = []
		self.points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
		# print(self.points)

		# self.points = [] 
		# for p in pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True):
		# 	self.points.append(p)
		if(len(self.points) != 0):
			self.dataprocessing()

	def initcamera(self,havebag=False):
		self.pipeline = rs.pipeline()
		config = rs.config()

		if(havebag):
			# rs.config.enable_device_from_file(config, '/home/maytus/internship_main/py2_amr/src/detection/bag/complexobstacle.bag')
			rs.config.enable_device_from_file(config, '/home/maytus/internship_main/py2_amr/src/detection/bag/env3_640480.bag')
			# rs.config.enable_device_from_file(config, '/home/maytus/internship_main/py2_amr/src/detection/bag/oneobstacle.bag')

		else:
			config.enable_stream(rs.stream.color, self.w, self.h, rs.format.rgb8, 30)
			config.enable_stream(rs.stream.depth, self.w, self.h, rs.format.z16, 30)

		profile = self.pipeline.start(config)
		self.get_frames()

	def get_frames(self):
		rospy.loginfo("[INFO] frames")

		self.iterations += 1
		# print(self.iterations)

		align = rs.align(rs.stream.color)

		frames = self.pipeline.wait_for_frames()
		profile = frames.get_profile()
		frames = align.process(frames)
		depth_frame = frames.get_depth_frame()
		color_frame = frames.get_color_frame()
		color = np.asanyarray(color_frame.get_data())
		rgb = color[..., ::-1].copy()
		self.rgb_img = rgb.copy()
		depth = np.asanyarray(depth_frame.get_data())
		self.depth = o3d.geometry.Image(depth)
		self.color = o3d.geometry.Image(rgb)
		self.camera_parameter = profile.as_video_stream_profile().get_intrinsics()

		self.dataprocessing()

	def getColorCallback(self, msg):

		try:
			cv2_cimg = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
			self.rgb_img = cv2_cimg
		
		except Exception as e:
			print(e)

	def dataprocessing(self):
		if(self.source == 'ros'):
			rospy.loginfo("[INFO] Data Processing with ROS Topic")

			# clear pointcloud
			self.pc.clear()

			# initial timer
			self.timestamp = []
			self.processtime = time.time()

			# timestamp
			if(self.start_time_flag):
				self.start_time = time.time()
				self.start_time_flag = False
			self.timestamp.append(format(time.time()-self.start_time, ".4f"))

			try:
				self.pc.points = o3d.utility.Vector3dVector(self.points) #use many time
				self.pc_size = len(self.pc.points)

				# o3d.visualization.draw_geometries([self.pc]+[self.axis])
				# print(np.array(self.pc.points))

				if(self.pc_size > 0):
					if(self.dataprocessing_param['voxelize']):
						self.pc = o3d.geometry.PointCloud.voxel_down_sample(self.pc, voxel_size=self.dataprocessing_param['voxel_size'])
					self.add_timestamp()
					self.initial_segmentation()

			except:
				pass

		elif(self.source == 'camera'):
			rospy.loginfo("[INFO] Data Processing from camera")

			# initial timer
			self.timestamp = []
			self.processtime = time.time()

			# timestamp
			if(self.start_time_flag):
				self.start_time = time.time()
				self.start_time_flag = False
			self.timestamp.append(format(time.time()-self.start_time, ".4f"))

			rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(self.color, self.depth)
			pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(self.camera_parameter.width, self.camera_parameter.height,
															 self.camera_parameter.fx,
															 self.camera_parameter.fy, self.camera_parameter.ppx,
															 self.camera_parameter.ppy)
			try:
				self.pc = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
				self.pc_size = len(self.pc.points)

				# o3d.visualization.draw_geometries([self.pc]+[self.axis])

				if(self.pc_size > 0):
					if(self.dataprocessing_param['voxelize']):
						self.pc = o3d.geometry.PointCloud.voxel_down_sample(self.pc, voxel_size=self.dataprocessing_param['voxel_size'])
					self.add_timestamp()
					
					self.initial_segmentation()

			except:
				pass	

	def initial_segmentation(self):
		rospy.loginfo("[INFO] Initial Segmentation")
		self.clusters['point'] = []

		if(self.initial_seg_param['estimate_normal']):
			search_param = o3d.geometry.KDTreeSearchParamKNN(knn=self.initial_seg_param['normals_neigh'])
			o3d.geometry.PointCloud.estimate_normals(self.pc, search_param=search_param)
			self.pc.orient_normals_consistent_tangent_plane(k=50)
			clustering_input_type = np.array(self.pc.normals)
		else:
			clustering_input_type = np.array(self.pc.points)
		
		db = DBSCAN(eps=self.initial_seg_param['eps'], min_samples=self.initial_seg_param['min_sample']).fit(clustering_input_type)
		labels = db.labels_
		max_label = labels.max()
		for i in range(0, max_label):
			indx = np.array(np.where(labels == i))
			self.clusters['point'].append(self.pc.select_by_index(indx[0]))

		if(self.initial_seg_param['debug']):

			for i, c in enumerate(self.clusters['point']):
				color = plt.get_cmap("hsv")(random.randint(0, 255))
				c.paint_uniform_color(list(color[:3]))
			print("[Result] Clustering We Got :",len(self.clusters['point']),'clusters')
			o3d.visualization.draw_geometries([c for c in self.clusters['point']]+[self.axis])


		self.add_timestamp()
		self.split()

	def split(self):

		rospy.loginfo("[INFO] Split")

		splits = {
			'point': [], 
			'centroid': [],
			'model' : [],
			'distanceFromOrigin': []
			}

		# Plane Removal
		if(self.initial_seg_param['estimate_normal']):
			plane = []
			obstacle = []

			for i in range(len(self.clusters['point'])):

				plane_theshold = int(self.pc_size / len(self.clusters['point'][i].points))

				# filtering small cluster
				if(len(self.clusters['point'][i].points)<=2):
					continue

				num = int(len(self.clusters['point'][i].points)*0.4) if int(len(self.clusters['point'][i].points)*0.4)>3 else 3

				model, indx = self.clusters['point'][i].segment_plane(distance_threshold=0.03,ransac_n=num,num_iterations=300)

				inlier = self.clusters['point'][i].select_by_index(indx)
				outlier = self.clusters['point'][i].select_by_index(indx,invert=True)

				try:
				# 	z_inlier = -1*(sum(np.array(inlier.points)[:,2]))/len(inlier.points)
				# 	z_outlier = -1*(sum(np.array(outlier.points)[:,2]))/len(outlier.points)

					# print(plane_theshold)
					# print(len(inlier.points))
					# print(len(outlier.points))

					if(len(inlier.points) > plane_theshold):
					# if(z_inlier > plane_theshold):
						inlier.paint_uniform_color([1,0,0])
						plane.append(inlier)
					else:
						inlier.paint_uniform_color([0,1,0])
						obstacle.append(inlier)

					if(len(outlier.points) > plane_theshold):
					# if(z_outlier > plane_theshold):
						outlier.paint_uniform_color([1,0,0])
						plane.append(outlier)

					else:
						outlier.paint_uniform_color([0,1,0])
						obstacle.append(outlier)

				except:
					continue
			if(self.split_param['debug']):
				o3d.visualization.draw_geometries([p for p in plane])
				o3d.visualization.draw_geometries([o for o in obstacle])
				o3d.visualization.draw_geometries([o for o in obstacle]+[p for p in plane]+[self.axis])
		else:
			obstacle = []
			obstacle = self.clusters['point'].copy()

		# Split
		if(len(obstacle) > 0):
			temp = obstacle.pop(0)
			for o in obstacle:
				# if(len(o.points) <= 0):
				# 	continue
				temp += o
			obstacle = temp

			point = np.array(obstacle.points)
			db = DBSCAN(eps=self.split_param['eps'], min_samples=self.split_param['min_sample']).fit(point)
			labels = db.labels_
			max_label = labels.max()
			for i in range(0, max_label):
				indx = np.array(np.where(labels == i))

				splits['point'].append(obstacle.select_by_index(indx[0]))

				C = splits['point'][-1].get_center()

				U,S,V = self.SVD(splits['point'][-1].points)
				n = self.getNormal(V=V)

				d = self.distance_point(xyz= C, xyz1= self.origin)

				splits['centroid'].append(C)
				splits['model'].append(n)
				splits['distanceFromOrigin'].append(d)

			self.clusters = splits

			if(self.split_param['debug']):
				for s in splits['point']:
					color = plt.get_cmap("hsv")(random.randint(0,255))
					s.paint_uniform_color(list(color[:3]))
				o3d.visualization.draw_geometries([s for s in splits['point']]+[self.axis])

			if(len(self.clusters['point']) >= 0):
				self.add_timestamp()
				self.merges()

			else:
				self.pointcloudpub()

	def merges(self):
		rospy.loginfo("[INFO] Merge")

		self.merge = []

		candidate = {
			'point': None,
			'centroid': None,
			'model' : None,
			'distanceFromOrigin': None
		}

		temp = {
			'point': None,
			'centroid': None,
			'model' : None,
			'distanceFromOrigin': None
		}

		if(len(self.clusters['point']) >= 1):

			if(len(self.clusters['point']) == 1):
				self.merge = self.clusters['point'].copy()

			elif(len(self.clusters['point']) >= 2):
				# find cadadate plane 
				self.clusters['len'] = []
				for p in self.clusters['point']:
					self.clusters['len'].append(len(p.points))

				self.candidate_len_sort()

				for i in range(len(self.clusters['point'])):

					count = 0

					if(len(self.clusters['point']) == 1):
						if(len(self.clusters['point']).point >= 50):
							self.clusters['centroid'][0] = self.clusters['point'].get_center()
							U,S,V = self.SVD(self.clusters['point'][0].points)
							n = self.getNormal(V=V)
							self.clusters['model'][0] = n
							d = self.distance_point(xyz= self.clusters['centroid'][0], xyz1= self.origin)
							self.clusters['distanceFromOrigin'][0] = append(d)

					elif(len(self.clusters['point']) > 1):
						# get candidate plane
						candidate['point'] = self.clusters['point'].pop(0)
						candidate['centroid'] = self.clusters['centroid'].pop(0)
						candidate['model'] = self.clusters['model'].pop(0)
						candidate['distanceFromOrigin'] = self.clusters['distanceFromOrigin'].pop(0)

						# initial case
						temp['point'] = self.clusters['point'].pop(count)
						temp['centroid'] = self.clusters['centroid'].pop(count)
						temp['model'] = self.clusters['model'].pop(count)
						temp['distanceFromOrigin'] = self.clusters['distanceFromOrigin'].pop(count)

						for j in range(len(self.clusters['point'])+1):

							if (self.merge_param['debug']):
								print('\t\t[Debug] Merging with distance between origin to centroid')
								print('\t\t',np.abs(candidate['distanceFromOrigin']-temp['distanceFromOrigin']))

								# candidate['point'].paint_uniform_color([0, 0, 1])
								# temp['point'].paint_uniform_color([1, 0, 0])

								# line = [
								# 	[self.origin, list(candidate['centroid']), list(temp['centroid'])],
								# 	[[0, 1], [0, 2]]
								# ]

								# line_set = o3d.geometry.LineSet(
								# 	points=o3d.utility.Vector3dVector(line[0]),
								# 	lines=o3d.utility.Vector2iVector(line[1]),
								# )
								# line_set.colors = o3d.utility.Vector3dVector([[0, 0, 1], [1, 0, 0]])
								# o3d.visualization.draw_geometries([candidate['point']]+[temp['point']]+[self.axis]+[line_set])

							if np.isclose(candidate['distanceFromOrigin'], temp['distanceFromOrigin'], atol=self.merge_param['distanceFromOrigin_tol']):

								distanceBetweenPlane = self.distance_point(xyz=candidate['centroid'],
																				   xyz1=temp['centroid'])

								if(self.merge_param['debug']):
									print('\t\t[Debug] Merging with distance between 2 plane')
									print('\t\t',distanceBetweenPlane)

								if (distanceBetweenPlane <= self.merge_param['distanceBetweenPlane_th']):

									if (self.merge_param['debug']):
										print('\t\t[Debug] Merged')
										# candidate['point'].paint_uniform_color([0, 1, 0])
										# temp['point'].paint_uniform_color([0, 1, 0])
										# line = [
										# 	[self.origin, list(candidate['centroid']), list(temp['centroid'])],
										# 	[[0, 1], [0, 2]]
										# ]
										# line_set = o3d.geometry.LineSet(
										# 	points=o3d.utility.Vector3dVector(line[0]),
										# 	lines=o3d.utility.Vector2iVector(line[1]),
										# )
										# line_set.colors = o3d.utility.Vector3dVector([[0, 1, 0], [0, 1, 0]])
										# o3d.visualization.draw_geometries([candidate['point'], temp['point'], self.axis, line_set])

									candidate['point'] += temp['point']

									try:
										temp['point'] = self.clusters['point'].pop(count)
										temp['centroid'] = self.clusters['centroid'].pop(count)
										temp['model'] = self.clusters['model'].pop(count)
										temp['distanceFromOrigin'] = self.clusters['distanceFromOrigin'].pop(count)
									except:
										pass

								else:
									self.clusters['point'].insert(count, temp['point'])
									self.clusters['centroid'].insert(count, temp['centroid'])
									self.clusters['model'].insert(count, temp['model'])
									self.clusters['distanceFromOrigin'].insert(count, temp['distanceFromOrigin'])
									count += 1
									try:
										temp['point'] = self.clusters['point'].pop(count)
										temp['centroid'] = self.clusters['centroid'].pop(count)
										temp['model'] = self.clusters['model'].pop(count)
										temp['distanceFromOrigin'] = self.clusters['distanceFromOrigin'].pop(count)
									except:
										pass
							else:
								self.clusters['point'].insert(count, temp['point'])
								self.clusters['centroid'].insert(count, temp['centroid'])
								self.clusters['model'].insert(count, temp['model'])
								self.clusters['distanceFromOrigin'].insert(count, temp['distanceFromOrigin'])
								count += 1
								try:
									temp['point'] = self.clusters['point'].pop(count)
									temp['centroid'] = self.clusters['centroid'].pop(count)
									temp['model'] = self.clusters['model'].pop(count)
									temp['distanceFromOrigin'] = self.clusters['distanceFromOrigin'].pop(count)
								except:
									pass

					# update candidate
					if(len(candidate['point'].points) >= 50):
						self.clusters['point'].append(candidate['point'])
						self.clusters['centroid'].append(candidate['point'].get_center())
						U,S,V = self.SVD(candidate['point'].points)
						n = self.getNormal(V=V)
						self.clusters['model'].append(n)
						d = self.distance_point(xyz= candidate['point'].get_center(), xyz1= self.origin)
						self.clusters['distanceFromOrigin'].append(d)

			self.merge = self.clusters['point'].copy()
			print(self.merge)

		# mapto2d
		if(not self.merge_param['bounding'] and not self.merge_param['debug']):
			end_f = 0
			for i in range(len(self.merge)):
				#position of obstacle bound
				try:
					b = self.merge[i].get_axis_aligned_bounding_box()
					boxnode = np.array(b.get_box_points())

					min_x = min(boxnode[0:,0])
					min_y = min(boxnode[0:,1])
					max_x = max(boxnode[0:,0])
					max_y = max(boxnode[0:,1])
					# min_z = min(np.abs(boxnode[0:,2]))
					min_z = (np.mean(boxnode[0:,2]))


					wd = min_z
					hfov = 2 * wd*math.tan(self.hfov_deg_c/2)
					# vfov = 2 * wd*math.tan(self.vfov_deg_c/2)
					vfov = hfov/4 * 3

					# m to mm
					min_x = (min_x + (hfov/2)) * 1000
					max_x = (max_x + (hfov/2)) * 1000
					min_y = (min_y + (vfov/2)) * 1000
					max_y = (max_y + (vfov/2)) * 1000

					# mm to pixel
					factor_x = self.w / (hfov*1000)
					factor_y = self.h / (vfov*1000)

					if(self.w == 160 and self.h == 120):
						top_left_pixel_x = int(min_x * factor_x)
						bottom_right_pixel_y = int(max_y * factor_y)
						bottom_right_pixel_x = int(max_x * factor_x)
						top_left_pixel_y = int(min_y * factor_y)
					elif(self.w == 640 and self.h == 480):
						top_left_pixel_x = int(min_x * factor_x)
						top_left_pixel_y = int(min_y * factor_y)
						bottom_right_pixel_x = int(max_x * factor_x)
						bottom_right_pixel_y = int(max_y * factor_y)
						
					elif(self.w == 320 and self.h == 240):
						top_left_pixel_x = int(min_x * factor_x)
						bottom_right_pixel_y = int(max_y * factor_y)
						bottom_right_pixel_x = int(max_x * factor_x)
						top_left_pixel_y = int(min_y * factor_y)

					print('xxx',top_left_pixel_x,top_left_pixel_y,bottom_right_pixel_x,bottom_right_pixel_y)

					if(self.source == 'ros'):
						#publish the point to use cv and show result
						msg = obstacle_bound()
						msg.top_left_x = int(top_left_pixel_x)
						msg.top_left_y = int(top_left_pixel_y)
						msg.bottom_right_x = int(bottom_right_pixel_x)
						msg.bottom_right_y = int(bottom_right_pixel_y)
						msg.distance = float(min_z)
						self.obstacle_bound_pub.publish(msg)
						end_f += 1

					elif(self.source == 'camera'):
						
						# insert Ground Truth
						#

						cv2.rectangle(self.rgb_img, (top_left_pixel_x, top_left_pixel_y), (bottom_right_pixel_x, bottom_right_pixel_y), (0,255,0), 2)
						text = 'obstacle at %.2f m' %(min_z)
						cv2.putText(self.rgb_img, text, (top_left_pixel_x - 5, top_left_pixel_y - 5), 0, 0.3, (0,255,0))
						end_f += 1

				except:
					self.merge.remove(self.merge[i])
					continue

			if(end_f > 0):
				msg = obstacle_bound()
				msg.top_left_x = -99
				msg.top_left_y = -99
				msg.bottom_right_x = -99
				msg.bottom_right_y = -99
				self.obstacle_bound_pub.publish(msg)
				end_f = 0

			if(self.source == 'camera'):
				cv2.imshow("Result", self.rgb_img)
				cv2.waitKey(1)



		if(self.merge_param['bounding']):
			boundingbox = []
			line = {
				'node' : [],         
				'edge' : [[4,5],[4,6],[5,3],[3,6],[0,3],[4,7],[6,1],[5,2],[0,2],[0,1],[1,7],[2,7]],
				'color' : []
			}
			for i in range(len(self.merge)):					
				#position of obstacle bound
				try:
					b = self.merge[i].get_axis_aligned_bounding_box()
					boxnode = np.array(b.get_box_points())
					line['node'].append(boxnode)
					line_set = o3d.geometry.LineSet(
						points=o3d.utility.Vector3dVector(line['node'][i].tolist()),
						lines=o3d.utility.Vector2iVector(line['edge']),
					)
					boundingbox.append(line_set)
				except:
					self.merge.remove(self.merge[i])
					continue

			for m in self.merge:
				color = plt.get_cmap("hsv")(random.randint(0,255))
				m.paint_uniform_color(list(color[:3]))

			o3d.visualization.draw_geometries([self.axis]+[box for box in boundingbox]+[m for m in self.merge])

		if(self.merge_param['debug']):
			print("[Result] After Merging We Got :",len(self.merge),'clusters')
			for m in self.merge:
				color = plt.get_cmap("hsv")(random.randint(0,255))
				m.paint_uniform_color(list(color[:3]))
		
			o3d.visualization.draw_geometries([m for m in self.merge]+[self.axis])

			
		# Total time usaged
		self.add_timestamp()
		# self.addtoCSV()
		self.pointcloudpub()



if __name__ == '__main__':
		obstacleDetection()
		rospy.spin()

	
"""note accuracy test GT
bound 		tlx 	tly 	brx 	bry
Real 
	env 1	 90		190		540		645
			cv2.rectangle(self.rgb_img, (90, 190), (540, 645), (255,0,0), 2)
			cv2.putText(self.rgb_img, 'Obstacle Ground Truth', (90 - 5, 190 - 5), 0, 0.3, (255,0,0))

	env 2    50 	230		370		450
			450		280		620		470

			cv2.rectangle(self.rgb_img, (50, 230), (370, 450), (255,0,0), 2)
			cv2.rectangle(self.rgb_img, (450, 280), (620, 470), (255,0,0), 2)
			cv2.putText(self.rgb_img, 'Obstacle Ground Truth', (50 - 5, 230 - 5), 0, 0.3, (255,0,0))
			cv2.putText(self.rgb_img, 'Obstacle Ground Truth', (450 - 5, 280 - 5), 0, 0.3, (255,0,0))

	env 3 	50		230		370		450
			380		230		580		470
			290		140		460		250
			cv2.rectangle(self.rgb_img, (50, 230), (370, 450), (255,0,0), 2)
			cv2.rectangle(self.rgb_img, (380, 230), (580, 470), (255,0,0), 2)
			cv2.rectangle(self.rgb_img, (290, 140), (460, 250), (255,0,0), 2)
			cv2.putText(self.rgb_img, 'Obstacle Ground Truth', (50 - 5, 230 - 5), 0, 0.3, (255,0,0))
			cv2.putText(self.rgb_img, 'Obstacle Ground Truth', (380 - 5, 230 - 5), 0, 0.3, (255,0,0))
			cv2.putText(self.rgb_img, 'Obstacle Ground Truth', (290 - 5, 140 - 5), 0, 0.3, (255,0,0))

	env 4	60		300		270		470
			280		260		410		410
			450		340		550		430
			cv2.rectangle(self.rgb_img, (60, 300), (270, 470), (255,0,0), 2)
			cv2.rectangle(self.rgb_img, (280, 260), (410, 410), (255,0,0), 2)
			cv2.rectangle(self.rgb_img, (450, 340), (550, 430), (255,0,0), 2)
			cv2.putText(self.rgb_img, 'Obstacle Ground Truth', (60 - 5, 300 - 5), 0, 0.3, (255,0,0))
			cv2.putText(self.rgb_img, 'Obstacle Ground Truth', (280 - 5, 260 - 5), 0, 0.3, (255,0,0))
			cv2.putText(self.rgb_img, 'Obstacle Ground Truth', (450 - 5, 340 - 5), 0, 0.3, (255,0,0))

	env 5	5		300		320		475
			160		230		305		300
			400		330		635		460
			cv2.rectangle(self.rgb_img, (5, 300), (320, 475), (255,0,0), 2)
			cv2.rectangle(self.rgb_img, (160, 230), (305, 300), (255,0,0), 2)
			cv2.rectangle(self.rgb_img, (400, 330), (635, 460), (255,0,0), 2)
			cv2.putText(self.rgb_img, 'Obstacle Ground Truth', (5 - 5, 300 - 5), 0, 0.3, (255,0,0))
			cv2.putText(self.rgb_img, 'Obstacle Ground Truth', (160 - 5, 230 - 5), 0, 0.3, (255,0,0))
			cv2.putText(self.rgb_img, 'Obstacle Ground Truth', (400 - 5, 330 - 5), 0, 0.3, (255,0,0))

	env 6 	220		50		480		480
			cv2.rectangle(self.rgb_img, (220, 50), (480, 480), (255,0,0), 2)
			cv2.putText(self.rgb_img, 'Obstacle Ground Truth', (220 - 5, 50 - 5), 0, 0.3, (255,0,0))

simulation
	env 1 	50		135		270		305
			160		90		530		270
			360		250		600		320
	env 2	80		135		270		305
			160		130		530		270
			380		260		600		320
	env 3
	env 4 	130		225		320		300
			170		70		320		230
			390		140		520		280
	env 5 	100		120		290		300
			305		190		395		280
			400		115		575		300
	env 6 	110		150		300		340
			190		40		430		280
			440		80		640		315

"""