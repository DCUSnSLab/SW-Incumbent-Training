#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import numpy as np
import time
import math

laser_pub = rospy.Publisher('scan', sensor_msgs.msg.LaserScan, queue_size=10)

avg = 0
totalcnt = 0

def min_diff_pos(array, target):
    return np.abs(np.array(array)-target).argmin()

def callback(data):
	scan = sensor_msgs.msg.LaserScan()

	scan.header = data.header

	scan.angle_min = data.angle_min
	scan.angle_max = data.angle_max
	scan.angle_increment = data.angle_increment
	scan.time_increment = data.time_increment
	
	scan.scan_time = data.scan_time
	
	scan.range_min = data.range_min
	scan.range_max = float("inf")

	# scan.ranges = [0] * size
	tmp_range = list()

	size = len(data.ranges)

	for i in range(0, size):
		if data.ranges[i] >= 10.0:
			tmp_range.append(float("inf"))
		else:
			tmp_range.append(data.ranges[i])
	
	scan.ranges = tmp_range

	scan.intensities = [0] * size
	# scan.intensities = np.zeros(len(size))

	laser_pub.publish(scan)

def laserscan_refine():
	rospy.init_node('Refine')
	rospy.Subscriber('lidar2D', sensor_msgs.msg.LaserScan, callback)
	rospy.spin()

if __name__ == '__main__':
	laserscan_refine()
