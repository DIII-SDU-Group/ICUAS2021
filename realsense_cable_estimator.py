#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import threading
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
import sensor_msgs.point_cloud2
from cv_bridge import CvBridge
import time
import math
import copy
import random
from skimage.measure import LineModelND, ransac

stop_program = False
xyz_locked = False  ## pointcloud handling semaphore
xyz = []
gimg = []
inliers = []


def show_image():
	print("show image started")
	hfov = 55#55#69.4
	vfov = 40#35#42.5
	width = 640
	height = 480
	point_pub = rospy.Publisher('realsense_cable_estimate/point', PointStamped, queue_size=10)
	x_pub = rospy.Publisher('realsense_cable_estimate/x', Float64, queue_size=10)
	y_pub = rospy.Publisher('realsense_cable_estimate/y', Float64, queue_size=10)
	z_pub = rospy.Publisher('realsense_cable_estimate/z', Float64, queue_size=10)
	time.sleep(2.0)
	global xyz
	global xyz_locked
	global stop_program
	while 1:
		if stop_program == True:
			return

		while xyz_locked == True:
			pass
		xyz_locked = True

		xyz_local = np.asarray(copy.deepcopy(xyz))
		model, inliers = ransac(xyz_local, LineModelND, min_samples=2, residual_threshold=0.15, max_trials=1000)
		xyz_local = xyz_local[inliers]
		cable_estimate = np.mean(xyz_local, axis=0)
		cable_estimate[0] = cable_estimate[0] #+ 0.05
		cable_estimate[1] = cable_estimate[1] + 0.015
		cable_estimate[2] = cable_estimate[2] + 0.075
		print("Cable estimate (xyz): %.2f %.2f %.2f" % (cable_estimate[0], cable_estimate[1], cable_estimate[2]))

		cable_estimate_point = PointStamped()
		cable_estimate_point.header.stamp = rospy.Time.now()
		cable_estimate_point.header.frame_id = "/drone_center"
		cable_estimate_point.point.y = cable_estimate[0]
		cable_estimate_point.point.x = cable_estimate[1]
		cable_estimate_point.point.z = cable_estimate[2]
		
		point_pub.publish(cable_estimate_point)
		y_pub.publish(cable_estimate[0])
		x_pub.publish(cable_estimate[1])
		z_pub.publish(cable_estimate[2])
		
		xyz_local = np.asarray(copy.deepcopy(xyz))

		draw_img = copy.deepcopy(gimg)
		xyz_local[len(xyz_local)-1] = (cable_estimate)
		for i in range(len(xyz_local)):
			h_ang = -math.degrees(math.atan((xyz_local[i][0] / xyz_local[i][2])))
			v_ang = -math.degrees(math.atan((xyz_local[i][1] / xyz_local[i][2])))
			if abs(h_ang) < (hfov/2) and abs(v_ang) < (vfov/2):
				xpixel = int(round((h_ang / (hfov/2) * (width/2)) + (width/2)))
				ypixel = int(round((v_ang / (vfov/2) * (height/2)) + (height/2)))
				if xyz_local[i][0] > 4.5: #LiDAR color scheme
					ratio = ((xyz_local[i][2] - 4.5) / 4.5) * 255
					color_b = int(round(ratio))
					color_g = int(round(255 - ratio))
					color_r = 0
				else:
					ratio = (xyz_local[i][2] / 4.5) * 255
					color_b = 0
					color_g = int(round(ratio))
					color_r = int(round(255 - ratio))

				cv2.circle(draw_img, (xpixel, ypixel), 2, (color_b, color_g, color_r), 1)
				if i == len(xyz_local)-1:
					cv2.circle(draw_img, (xpixel, ypixel), 9, (255, 0, 255), 2)

		#cv2.imwrite("realsense_data.jpg", draw_img) 

		xyz_locked = False		

		scale_percent = 200
		dsize = (width*scale_percent/100, height*scale_percent/100)
		resized_img = cv2.resize(draw_img, dsize)
		cv2.imshow("image", resized_img)
		k=cv2.waitKey(10) & 0XFF

	cv2.destroyAllWindows()



def get_xyz_points(msg):
	global xyz_locked
	global xyz
	while xyz_locked == True:
		pass
	xyz_locked = True
	del xyz[:]

	low_threshold = 0.8
	high_threshold = 4.0
	
	for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
		if point[2] > low_threshold and point[2] < high_threshold:
			if random.randint(1, 10) == 1:
				xyz.append([-point[0], -point[1], point[2]])
	xyz_locked = False
		

def img_callback(msg):	
	global gimg
	gimg = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
	#cv2.imwrite("rbg_raw.jpg", gimg) 
	#print("Img: ", msg.header.stamp.secs - 1613120000)
	

if __name__ == '__main__':
	print("Subscribing to topics..")
	show_img_thread = threading.Thread(target=show_image)
	show_img_thread.start()
	rospy.init_node('realsense_cable_estimator')
	rospy.Subscriber('/camera/depth_registered/points', PointCloud2, get_xyz_points)
	rospy.Subscriber('/camera/color/image_rect_color', Image, img_callback)
	rospy.spin()
	xyz_locked = False
	stop_program = True


