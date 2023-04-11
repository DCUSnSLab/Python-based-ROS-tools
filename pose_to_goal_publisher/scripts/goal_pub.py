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

rospy.init_node('py_goal_pub')
goal_msg = geometry_msgs.msg.PoseStamped()
pub = rospy.Publisher('move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=1)
rate = rospy.Rate(1)

goal_msg.header.seq = 0
goal_msg.header.stamp.secs = 0
goal_msg.header.stamp.nsecs = 0
goal_msg.header.frame_id = ''

goal_msg.pose.position.x = pose_x
goal_msg.pose.position.y = pose_y
goal_msg.pose.position.z = pose_z

goal_msg.pose.orientation.x = ori_x
goal_msg.pose.orientation.y = ori_y
goal_msg.pose.orientation.z = ori_z
goal_msg.pose.orientation.w = ori_w

one_msg = False

while not one_msg:
	connections = pub.get_num_connections()
	if connections > 0:
		pub.publish(goal_msg)
		one_msg = True
		print("goal published")
	else:
		rate.sleep()
