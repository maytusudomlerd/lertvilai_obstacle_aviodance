#!/usr/bin/env python3.6

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import cv2

import numpy as np
import random 
import time

import csv

import pyrealsense2 as rs
import open3d as o3d
import random
import math
from matplotlib import pyplot as plt

from sklearn.cluster import DBSCAN
from sklearn.cluster import KMeans


class obstacleDetection:
	
	def __init__(self):
		self.color_topic = '/camera/color/image_raw'
		self.depth_topic = '/camera/depth/image_raw'
		self.pointcloud_topic = '/camera/depth/color/points'
		self.bridge = CvBridge()
		self.color_img = np.array([])
		self.depth_img = np.array([])
		
		self.points = []

		self.pc = o3d.geometry.PointCloud()
		self.pc_size = 0
		self.pc_number_initialseg = 0
		self.axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
		self.origin = [0,0,0]

		self.clusters = {'point': [] }
		self.merge = []

		self.timestamp = []
		self.epoch = []
		self.processtime = 0.0

		# initialze
		rospy.init_node("detection")
		self.pointcloud_pub = rospy.Publisher("detection/result", PointCloud2,queue_size=1)
		rospy.Rate(30)
		for i in range(50):
			rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.getPointcloudCallback)


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
		# now = time.time()
		self.timestamp.append(now)
		# print(len(self.timestamp))

	def addtoCSV(self):
		with open('computing_time', 'a') as c:
			write = csv.writer(c)
			self.timestamp.insert(0,self.pc_size)
			write.writerows([self.timestamp])

	def pointcloudpub(self):
		rospy.loginfo("[INFO] Publishing")
		pcloud = PointCloud2()
		point = self.merge.pop(0)
		for m in self.merge:
			# point = np.array(m.points).tolist()
			point += m
			# check  point is a list
		result = np.array(point.points).tolist()
		# print(point)
		# o3d.visualization.draw_geometries([point])
		pcloud.header.frame_id = 'camera_link'
		pcloud.header.stamp = rospy.Time.now()
		pcloud = pc2.create_cloud_xyz32(pcloud.header, result)
		# print(pcloud)

		self.pointcloud_pub.publish(pcloud)

	def sub_pointcloud(self):
		rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.getPointcloudCallback)

	def getPointcloudCallback(self,msg):
		self.points = []
		for p in pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True):
			self.points.append(p)

		self.dataprocessing(voxel_size=0.05)


	def dataprocessing(self,voxelize=True,voxel_size=0.05):
		rospy.loginfo("[INFO] Data Processing")
		self.pc.clear()
		# initial timer
		self.timestamp = []
		self.processtime = time.time()

		
		try:
			self.pc.points = o3d.utility.Vector3dVector(self.points) #use many time
			self.pc_size = len(self.pc.points)
			# print(self.pc_size)
			self.pc.transform([[0, 0, 1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]]) 
			# self.pc.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

			if(voxelize):
				self.pc = o3d.geometry.PointCloud.voxel_down_sample(self.pc, voxel_size=voxel_size)
			self.add_timestamp()
			# print(self.processtime)
			# print(self.timesta0mp)
			# self.groundtruth()
			x = np.array(self.pc.points)[0:, 0:1]
			y = np.array(self.pc.points)[0:, 1:2]
			z = np.array(self.pc.points)[0:, 2:3]
			print(np.array(self.pc.points[0]))
			x_max = max(x.tolist())
			y_max = max(y.tolist())
			z_max = max(z.tolist())
			x_min = min(x.tolist())
			y_min = min(y.tolist())
			z_min = min(z.tolist())
			print(x_min,x_max)
			print(y_min,y_max)
			print(z_min,z_max)
			o3d.visualization.draw_geometries([self.pc]+[self.axis])

			# self.initial_segmentation(estimate_normal=True,debug=False)

		except:
			pass

	def initial_segmentation(self,eps=0.05,min_sample=5,debug=False,estimate_normal=False,normals_neigh=30):
		
		rospy.loginfo("[INFO] Initial Segmentation")

		if(estimate_normal):
			search_param = o3d.geometry.KDTreeSearchParamKNN(knn=normals_neigh)
			o3d.geometry.PointCloud.estimate_normals(self.pc, search_param=search_param)
			self.pc.orient_normals_consistent_tangent_plane(k=50)
			clustering_input_type = np.array(self.pc.normals)
		else:
			clustering_input_type = np.array(self.pc.points)
		
		db = DBSCAN(eps=eps, min_samples=min_sample).fit(clustering_input_type)
		labels = db.labels_
		max_label = labels.max()
		for i in range(0, max_label):
			indx = np.array(np.where(labels == i))
			self.clusters['point'].append(self.pc.select_by_index(indx[0]))
		self.pc_number_initialseg = max_label+1

		if(debug):

			for i, c in enumerate(self.clusters['point']):
				color = plt.get_cmap("hsv")(random.randint(0, 255))
				c.paint_uniform_color(list(color[:3]))
			print("[Result] Clustering We Got :",len(self.clusters['point']),'clusters')
			o3d.visualization.draw_geometries([c for c in self.clusters['point']]+[self.axis])


		self.add_timestamp()
		self.split(debug=False)

	def split(self,debug=False):

		rospy.loginfo("[INFO] Split")

		kmean = KMeans(n_clusters=2, random_state=0)

		splits = {
			'point': [], 
			'centroid': [],
			'model' : [],
			'distanceFromOrigin': []
			}

		plane = []
		obstacle = []

		for i in range(len(self.clusters['point'])):

			plane_theshold = int(self.pc_size / len(self.clusters['point'][i].points))
			# print(plane_theshold)
			# print('plane theshold ',plane_theshold)

			# # filtering small cluster
			if(len(self.clusters['point'][i].points)<=2):
				continue

			# num = int(len(self.clusters['point'][i].points))
			num = int(len(self.clusters['point'][i].points)*0.4) if int(len(self.clusters['point'][i].points)*0.4)>3 else 3

			model, indx = self.clusters['point'][i].segment_plane(distance_threshold=0.03,ransac_n=num,num_iterations=300)

			inlier = self.clusters['point'][i].select_by_index(indx)
			outlier = self.clusters['point'][i].select_by_index(indx,invert=True)

			# print('len ',len(inlier.points))
			if(len(inlier.points) > plane_theshold):
				inlier.paint_uniform_color([0,0,0])
				plane.append(inlier)

			else:
				inlier.paint_uniform_color([0,1,0])
				obstacle.append(inlier)

			if( 10 <= len(outlier.points) <= plane_theshold):
				outlier.paint_uniform_color([0,1,0])
				obstacle.append(outlier)

		# o3d.visualization.draw_geometries([p for p in plane]+[self.axis])
		# o3d.visualization.draw_geometries([o for o in obstacle]+[p for p in plane]+[self.axis])

		if(len(obstacle) >= 0):

			temp = obstacle.pop(0)
			for o in obstacle:
				temp += o
			obstacle = temp

			point = np.array(obstacle.points)
			db = DBSCAN(eps=0.05, min_samples=3).fit(point)
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

			if(debug):
				for s in splits['point']:
					color = plt.get_cmap("hsv")(random.randint(0,255))
					s.paint_uniform_color(list(color[:3]))
				o3d.visualization.draw_geometries([s for s in splits['point']])

			if(len(self.clusters['point']) >= 0):
				self.add_timestamp()
				self.merges(debug=False,bounding=True)

			else:
				self.pointcloudpub()

	def merges(self,distanceFromOrigin_tol=3,distanceBetweenPlane_th=2,angleBetweenPlane_th=5,debug=False,bounding=False):
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

		if(len(self.clusters['point']) >= 2):
			# print(len(self.clusters['point']))
			# print('\t[status] can merge')

			# find cadadate plane 
			self.clusters['len'] = []
			for p in self.clusters['point']:
				self.clusters['len'].append(len(p.points))

			self.candidate_len_sort()

			for i in self.clusters['point']:

				count = 0

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

				for j in range(len(self.clusters['distanceFromOrigin'])+1):

					if (debug):
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
					if np.isclose(candidate['distanceFromOrigin'], temp['distanceFromOrigin'], atol=distanceFromOrigin_tol):
						distanceBetweenPlane = self.distance_point(xyz=candidate['centroid'],
																		   xyz1=temp['centroid'])

						if(debug):
							print('\t\t[Debug] Merging with distance between 2 plane')
							print('\t\t',distanceBetweenPlane)

						if (distanceBetweenPlane <= distanceBetweenPlane_th):
							if (debug):
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

				self.merge.append(candidate['point'])

			if (len(self.clusters['point']) != 0):
				for c in self.clusters['point']:
					self.merge.append(c)

			if(bounding):
				boundingbox = []
				line = {
					'node' : [],         
					'edge' : [[4,5],[4,6],[5,3],[3,6],[0,3],[4,7],[6,1],[5,2],[0,2],[0,1],[1,7],[2,7]],
					'color' : []
				}
				for i in range(len(self.merge)):
					C = self.merge[i].get_center()
					x,y,z = C
					print('x ',x,' y ',y,' z ',z)
					
					#position of obstacle bound
					try:
						b = self.merge[i].get_oriented_bounding_box()
						boxnode = np.array(b.get_box_points())
						line['node'].append(boxnode)
						line_set = o3d.geometry.LineSet(
							points=o3d.utility.Vector3dVector(line['node'][i].tolist()),
							lines=o3d.utility.Vector2iVector(line['edge'])
						)
						boundingbox.append(line_set)
					except:
						self.merge.remove(self.merge[i])
						continue

				for m in self.merge:
					color = plt.get_cmap("hsv")(random.randint(0,255))
					m.paint_uniform_color(list(color[:3]))

				# o3d.visualization.draw_geometries([m for m in self.merge])
				o3d.visualization.draw_geometries([self.axis]+[b for b in boundingbox]+[m for m in self.merge])  #

			if(debug):
				print("[Result] After Merging We Got :",len(self.merge),'clusters')
				i=50
				for m in self.merge:
					color = plt.get_cmap("hsv")(i)
					m.paint_uniform_color(list(color[:3]))
					i+=50
				
				self.pc.paint_uniform_color([0,0,0])
				o3d.visualization.draw_geometries([m for m in self.merge]+[self.axis])
				# o3d.visualization.draw_geometries([self.axis]+[self.pc]+[m for m in self.merge])

			
			# Total time usaged
			self.add_timestamp()
			# print('add')
			# self.addtoCSV()
			# print('csv')
			self.pointcloudpub()

		else:
			print('\t[status] 1 cluster')

			self.merge = self.clusters['point']
			self.add_timestamp()
			self.addtoCSV()
			self.pointcloudpub()




if __name__ == '__main__':
		obstacleDetection()
		rospy.spin()

	