#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import numpy as np
import ros_numpy
import time
import math

pc2_pub = rospy.Publisher('filtered_pc2', sensor_msgs.msg.PointCloud2, queue_size=10)
laser_pub = rospy.Publisher('filtered_laserscan', sensor_msgs.msg.LaserScan, queue_size=10)

avg = 0
totalcnt = 0

def min_diff_pos(array, target):
    return np.abs(np.array(array)-target).argmin()

def callback(data):
	global totalcnt
	global avg
	start = time.time()

	tmp_header = data.header

	pc = ros_numpy.numpify(data)
	points = np.zeros((pc.shape[0], 6))
	points[:, 0] = pc['x']
	points[:, 1] = pc['y']
	points[:, 2] = pc['z']
	points[:, 3] = pc['intensity']
	points[:, 4] = pc['ring']
	points[:, 5] = pc['time']

	# Define filtering area
	npp = points[np.logical_and(points[:,2] >= -0.075, points[:,2] <= 0.25)]

	data = np.zeros(npp.shape[0], dtype=[
		('x', np.float32),
		('y', np.float32),
		('z', np.float32),
		('intensity', np.float32),
		('ring', np.uint16),
		('time', np.float32),
	])
	data['x'] = npp[:,0]
	data['y'] = npp[:,1]
	data['z'] = npp[:,2]
	data['intensity'] = npp[:,3]
	data['ring'] = npp[:,4]
	data['time'] = npp[:,5]

	pc_length = len(data['x'])

	rtn_data = ros_numpy.msgify(sensor_msgs.msg.PointCloud2, data)
	rtn_data.header = tmp_header



	scan = sensor_msgs.msg.LaserScan()
	tmp_header.frame_id = "velodyne"
	scan.header = tmp_header
	scan.angle_increment = 0.007 # Velodyne 3d lidar resolution
	#scan.angle_increment = 0.0039269908 # Rplidar a3 resolution
	scan.angle_min = -3.14
	scan.angle_max = 3.14
	scan.range_min = 0.0
	scan.range_max = 200.0
	scan.time_increment = 0.0
	#scan.ranges = [0] * int(points_per_ring)
	size = int(2 * 3.14 / scan.angle_increment)

	scan.ranges = [0] * size
	# scan.ranges = [0] * len(data['x'])
	scan.intensities = [0] * size
	# scan.intensities = np.zeros(len(size))

	scan_angle = np.zeros(pc_length)

	for idx in range(0, pc_length): #
		scan_angle[idx] = math.atan2(data['y'][idx], data['x'][idx])

	for index in range(0, size):
		# print(index)
		current_angle = -3.14 + index * scan.angle_increment
		tmp_idx = min_diff_pos(scan_angle, current_angle) #
		if scan_angle[tmp_idx] > current_angle - scan.angle_increment and scan_angle[tmp_idx] < current_angle + scan.angle_increment:
			scan.ranges[index] = math.sqrt(data['x'][tmp_idx] * data['x'][tmp_idx] + data['y'][tmp_idx] * data['y'][tmp_idx])
			scan.intensities[index] = data['intensity'][tmp_idx]
		else:
			pass

	# remove 0 range, intensity in list
	# scan.ranges = [i for i in scan.ranges if i != 0]
	# scan.intensities = [i for i in scan.intensities if i != 0]

	totalcnt = totalcnt + 1
	endtime = time.time() - start
	avg = (avg * (totalcnt - 1) + endtime) / totalcnt
	# print(avg) # total time

	pc2_pub.publish(rtn_data)
	laser_pub.publish(scan)

def pc2_filter():
	rospy.init_node('pc2_filter')
	rospy.Subscriber('velodyne_points', sensor_msgs.msg.PointCloud2, callback)
	rospy.spin()

if __name__ == '__main__':
	pc2_filter()
