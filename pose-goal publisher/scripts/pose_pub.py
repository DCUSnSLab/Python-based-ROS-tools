#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
import sys

pose_x = float(sys.argv[1])
pose_y = float(sys.argv[2])
pose_z = float(sys.argv[3])

ori_x = float(sys.argv[4])
ori_y = float(sys.argv[5])
ori_z = float(sys.argv[6])
ori_w = float(sys.argv[7])

rospy.init_node('py_pose_pub')
pose_msg = geometry_msgs.msg.PoseWithCovarianceStamped()
pub = rospy.Publisher('initialpose', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=1)
rate = rospy.Rate(1)

pose_msg.header.seq = 0
pose_msg.header.stamp.secs = 0
pose_msg.header.stamp.nsecs = 0
pose_msg.header.frame_id = ''

pose_msg.pose.pose.position.x = pose_x
pose_msg.pose.pose.position.y = pose_y
pose_msg.pose.pose.position.z = pose_z

pose_msg.pose.pose.orientation.x = ori_x
pose_msg.pose.pose.orientation.y = ori_y
pose_msg.pose.pose.orientation.z = ori_z
pose_msg.pose.pose.orientation.w = ori_w
pose_msg.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

one_msg = False

while not one_msg:
	connections = pub.get_num_connections()
	if connections > 0:
		pub.publish(pose_msg)
		one_msg = True
		print("pose published")
	else:
		rate.sleep()
