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
pt_x = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
pt_y = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
pt_z = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
gimg = []
low_threshold = 1.2


def show_image():
	print("show image started")
	hfov = 50#69.4
	vfov = 35#42.5
	width = 640
	height = 480
	xmean = 0.00001
	ymean = 0.00001
	zmean = 0.00001	
	global low_treshold
	xp = copy.deepcopy(pt_x)
	yp = copy.deepcopy(pt_y)
	zp = copy.deepcopy(pt_z)
	point_pub = rospy.Publisher('vu8_cable_estimate/point', PointStamped, queue_size=10)
	x_pub = rospy.Publisher('vu8_cable_estimate/x', Float64, queue_size=10)
	y_pub = rospy.Publisher('vu8_cable_estimate/y', Float64, queue_size=10)
	z_pub = rospy.Publisher('vu8_cable_estimate/z', Float64, queue_size=10)
	time.sleep(0.4)
	while 1:
		global stop_program
		if stop_program == True:
			return

		global xyz_locked
		while xyz_locked == True:
			pass
		xyz_locked = True

		draw_img = copy.deepcopy(gimg)
		if not isinstance(draw_img, np.ndarray):
			draw_img = np.zeros((height,width,3), np.uint8) + 255

		xsum = 0.0
		ysum = 0.0
		zsum = 0.0	
		count = 0.0
		for i in range(16):
			if pt_x[i] > 0.05:
				count = count + 1
				xsum = xsum + pt_x[i]
				ysum = ysum + pt_y[i]
				zsum = zsum + pt_z[i]

		if count > 0.0:
			xmean = xsum / count
			ymean = ysum / count
			zmean = zsum / count

		if isinstance(draw_img, np.ndarray):
			for i in range(16):
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
						cv2.circle(draw_img, (xpixel, ypixel), 5, (color_b, color_g, color_r), 3) # draw average as green circle
		

		h_ang = -math.degrees(math.atan((ymean / xmean)))
		v_ang = -math.degrees(math.atan((zmean / xmean)))
		if abs(h_ang) < (hfov/2) and abs(v_ang) < (vfov/2):
			xpixel = int(round((h_ang / (hfov/2) * (width/2)) + (width/2)))
			ypixel = int(round((v_ang / (vfov/2) * (height/2)) + (height/2)))

			if isinstance(draw_img, np.ndarray):
				cv2.circle(draw_img, (xpixel, ypixel), 9, (255, 0, 255), 2)

		cable_estimate = [ymean, zmean, xmean]
		cable_estimate[0] = cable_estimate[0] + 0.0 ## drone_center to sensor transform
		cable_estimate[1] = cable_estimate[1] + 0.0
		cable_estimate[2] = cable_estimate[2] + 0.0
		print("Cable estimate (min dist %.2f m) (xyz): %.2f %.2f %.2f" % (low_threshold, cable_estimate[0], cable_estimate[1], cable_estimate[2]))
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


		xyz_locked = False
		if isinstance(draw_img, np.ndarray):
			scale_percent = 200
			dsize = (width*scale_percent/100, height*scale_percent/100)
			resized_img = cv2.resize(draw_img, dsize)
			cv2.imshow("image", resized_img)
			k=cv2.waitKey(10) & 0XFF

		#cv2.imwrite("vu8_data.jpg", draw_img)

		xp = copy.deepcopy(pt_x)
		yp = copy.deepcopy(pt_y)
		zp = copy.deepcopy(pt_z)

	cv2.destroyAllWindows()


def get_xyz_points(msg, args):
	global xyz_locked
	global pt_x
	global pt_y
	global pt_z
	global low_treshold
	while xyz_locked == True:
		pass
	xyz_locked = True
	its = 0
	#del pt_x[:]
	#del pt_y[:]
	#del pt_z[:]
	if args == 'VU100':
		pass
		for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
			if point[0] > low_threshold:
				#print(its)
				pt_x[its] = point[0]
				pt_y[its] = point[1]
				pt_z[its] = point[2]
				its = its + 1
		for j in range(8-its):
			pt_x[its+j] = 0 
			pt_y[its+j] = 0
			pt_z[its+j] = 0
	else:
		for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
			if point[0] > low_threshold:
				#print(its)
				pt_x[its+8] = point[0]
				pt_y[its+8] = -point[1]
				pt_z[its+8] = point[2]
				its = its + 1	
		for j in range(8-its):
			pt_x[its+8+j] = 0 
			pt_y[its+8+j] = 0
			pt_z[its+8+j] = 0
	xyz_locked = False

def img_callback(msg):	
	global gimg
	gimg = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")

	

if __name__ == '__main__':
	print("Subscribing to topics..")
	show_img_thread = threading.Thread(target=show_image)
	show_img_thread.start()
	rospy.init_node('vu8_cable_estimator')
	rospy.Subscriber('/LiDAR/VU8_100Deg/scan_cloud', PointCloud2, get_xyz_points, 'VU100')
	rospy.Subscriber('/LiDAR/VU8_48Deg/scan_cloud', PointCloud2, get_xyz_points, 'VU48')
	rospy.Subscriber('/camera/color/image_rect_color', Image, img_callback)
	rospy.spin()
	xyz_locked = False
	stop_program = True




