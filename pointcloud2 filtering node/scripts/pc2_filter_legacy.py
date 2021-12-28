#!/usr/bin/env python3
import rospy
import getch
import sensor_msgs.msg
import numpy
import ros_numpy

from multiprocessing import Process, Queue

pc2_pub = rospy.Publisher('filtered_pc2', sensor_msgs.msg.PointCloud2, queue_size=10)
laser_pub = rospy.Publisher('filtered_laserscan', sensor_msgs.msg.LaserScan, queue_size=10)


def callback(data):
    # print()
    # print("Origin")
    # print("header")
    # print(data.header)
    # print("height")
    # print(data.height)
    # print("width")
    # print(data.width)
    # print("fields")
    # print(data.fields)
    # print("is_bigendian")
    # print(data.is_bigendian)
    # print("point_step")
    # print(data.point_step)
    # print("row_step")
    # print(data.row_step)
    # # print(data.data)
    # print("dense")
    # print(data.dense)

    tmp_header = data.header

    pc2 = ros_numpy.numpify(data)

    points = numpy.zeros((pc2.shape[0], 3))
    points[:, 2] = pc2['z']

    ### Delete numpy elements with python list

    pc2filter = [0] * len(points)

    for i, point in enumerate(points):
        if point[2] > 0.5 or point[2] < -0.1:  # z축 filter 범위 지정
            pc2filter[i] = i

    filtered_pc2 = numpy.delete(pc2, pc2filter)

    #### Filtering numpy array with numpy filter

    # pc2filter = numpy.zeros(pc2.shape[0], dtype=bool)
    #
    # for i, point in enumerate(points):
    # 	if point[2] < 0.5 and point[2] > -0.1: # z축 filter 범위 지정
    # 		pc2filter[i] = True
    #
    # np2list_filter = pc2filter.tolist()
    # filtered_pc2 = pc2[np2list_filter]

    ###

    rtn_data = ros_numpy.msgify(sensor_msgs.msg.PointCloud2, filtered_pc2)
    rtn_data.header = tmp_header

    # print()
    # print("Custom")
    # print("header")
    # print(rtn_data.header)
    # print("height")
    # print(rtn_data.height)
    # print("width")
    # print(rtn_data.width)
    # print("fields")
    # print(rtn_data.fields)
    # print("is_bigendian")
    # print(rtn_data.is_bigendian)
    # print("point_step")
    # print(rtn_data.point_step)
    # print("row_step")
    # print(rtn_data.row_step)
    # #print(rtn_data.data)
    # print("dense")
    # print(rtn_data.dense)

    scan = sensor_msgs.msg.LaserScan()

    scan.header = tmp_header

    pc2_pub.publish(rtn_data)


def pc2_filter():
    rospy.init_node('pc2_filter')
    rospy.Subscriber('velodyne_points', sensor_msgs.msg.PointCloud2, callback)
    rospy.spin()


if __name__ == '__main__':
    pc2_filter()
