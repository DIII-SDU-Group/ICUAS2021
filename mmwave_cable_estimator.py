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

stop_program = False
xyz_locked = False
pt_x = []
pt_y = []
pt_z = []
gimg = []


def show_image():
	print("show image started")
	hfov = 55
	vfov = 35
	width = 640
	height = 480
	x_cable_old = 0
	y_cable_old = 0
	z_cable_old = 0
	x_est_pixel = 0
	y_est_pixel = 0
	low_threshold = 1.2

	point_pub = rospy.Publisher('mmwave_cable_estimate/point', PointStamped, queue_size=10)
	x_pub = rospy.Publisher('mmwave_cable_estimate/x', Float64, queue_size=10)
	y_pub = rospy.Publisher('mmwave_cable_estimate/y', Float64, queue_size=10)
	z_pub = rospy.Publisher('mmwave_cable_estimate/z', Float64, queue_size=10)
    #rate = rospy.Rate(30) # 30hz

	time.sleep(0.4)
	while 1:
		global stop_program
		if stop_program == True:
			return

		global xyz_locked
		while xyz_locked == True:
			pass
		xyz_locked = True

		xp = copy.deepcopy(pt_x)
		yp = copy.deepcopy(pt_y)
		zp = copy.deepcopy(pt_z)

		draw_img = copy.deepcopy(gimg) #gimg.copy()
		if not isinstance(draw_img, np.ndarray):
			draw_img = np.zeros((height,width,3), np.uint8) + 255

		if isinstance(draw_img, np.ndarray):
			for i in range(len(xp)):
				if abs(xp[i]) > 0.05:
					h_ang = -math.degrees(math.atan((yp[i] / xp[i])))
					v_ang = -math.degrees(math.atan((zp[i] / xp[i])))
					if abs(h_ang) < (hfov/2) and abs(v_ang) < (vfov/2):
						xpixel = int(round((h_ang / (hfov/2) * (width/2)) + (width/2)))
						ypixel = int(round((v_ang / (vfov/2) * (height/2)) + (height/2)))

						if xp[i] > 4.5: #LiDAR color scheme
							ratio = ((xp[i] - 4.5) / 4.5) * 255
							color_b = int(round(ratio))
							color_g = int(round(255 - ratio))
							color_r = 0
						else:
							ratio = (xp[i] / 4.5) * 255
							color_b = 0
							color_g = int(round(ratio))
							color_r = int(round(255 - ratio))


						#color_r = int(round(255 - (xp[i] / 9) * 255))
						#color_g = 0
						#color_b = 0

						cv2.circle(draw_img, (xpixel, ypixel), 5, (color_b, color_g, color_r), 3)

		x_cable = 999 ## estimate cable as the closest point that is further away than 5cm
		idx = 0
		for i in range(len(xp)):
			if abs(xp[i]) > low_threshold:
				if xp[i] < x_cable:
					idx = i
					x_cable = xp[i]
		y_cable = yp[idx]
		z_cable = zp[idx]
		h_ang = -math.degrees(math.atan((y_cable / x_cable)))
		v_ang = -math.degrees(math.atan((z_cable / x_cable)))
		#if abs(h_ang) < (hfov/2) and abs(v_ang) < (vfov/2):
		x_est_pixel = int(round((h_ang / (hfov/2) * (width/2)) + (width/2)))
		y_est_pixel = int(round((v_ang / (vfov/2) * (height/2)) + (height/2)))
		x_cable_old = x_cable + 0.054 ## drone_center to sensor transform
		y_cable_old = y_cable - 0.014
		z_cable_old = z_cable - 0.04

		color_g = int(round(255 - (x_cable_old / 9) * 255))
		color_r = 0
		color_b = 0
		if isinstance(draw_img, np.ndarray):
			cv2.circle(draw_img, (x_est_pixel, y_est_pixel), 11, (255, 0, 255), 2) ## draw cable estimate in pink
		print("Estimated position (min dist %.1f m) (xyz): %.3f %.3f %.3f" % (low_threshold, -z_cable_old, y_cable_old, x_cable_old))

		cable_estimate_point = PointStamped()
		cable_estimate_point.header.stamp = rospy.Time.now()
		cable_estimate_point.header.frame_id = "/drone_center"
		cable_estimate_point.point.x = -z_cable_old
		cable_estimate_point.point.y = y_cable_old 
		cable_estimate_point.point.z = x_cable_old
		
		point_pub.publish(cable_estimate_point)
		x_pub.publish(-z_cable_old)
		y_pub.publish(y_cable_old)
		z_pub.publish(x_cable_old)
	
		#cv2.imwrite("mmwave_data.jpg", draw_img)

		xyz_locked = False

		if isinstance(draw_img, np.ndarray):
			scale_percent = 200
			dsize = (width*scale_percent/100, height*scale_percent/100)
			resized_img = cv2.resize(draw_img, dsize)
			cv2.imshow("image", resized_img)
			k=cv2.waitKey(10) & 0XFF

	cv2.destroyAllWindows()


def get_xyz_points(msg):
	global xyz_locked
	while xyz_locked == True:
		pass
	xyz_locked = True
	del pt_x[:]
	del pt_y[:]
	del pt_z[:]
	for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
		pt_x.append(point[0])
		pt_y.append(point[1])
		pt_z.append(point[2])
	#print("X: ", pt_x)
	#print("Y: ", pt_y)
	#print("Z: ", pt_z)
	xyz_locked = False

def img_callback(msg):	
	global gimg
	gimg = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")

	

if __name__ == '__main__':
	print("Subscribing to topics..")
	show_img_thread = threading.Thread(target=show_image)
	show_img_thread.start()
	rospy.init_node('mmwave_cable_estimator')
	rospy.Subscriber('/ti_mmwave/radar_scan_pcl', PointCloud2, get_xyz_points)
	rospy.Subscriber('/camera/color/image_rect_color', Image, img_callback)
	rospy.spin()
	stop_program = True



