#!/usr/bin/env python3
import rospy
import getch
import sensor_msgs.msg
#import sensor_msgs from Joy

def Key_joy():
	rospy.init_node('key_to_joy')
	Joy_set = sensor_msgs.msg.Joy()
	pub = rospy.Publisher('joy', sensor_msgs.msg.Joy, queue_size=10)
	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		k = ord(getch.getch())
#		print(str(k))
		print(chr(k))
		if (chr(k) == 'q'):
			rospy.loginfo("Exit")
			exit()
		elif (chr(k) == 'z'): # pause
			Joy_set.axes = [0, 0, 0, 0, 0, 0, 0, 0]
		elif (chr(k) == 'w'): # foward
			Joy_set.axes = [0, 1, 0, 0, 0, 0, 0, 0]
		elif (chr(k) == 's'): # reverse
			Joy_set.axes = [0, -1, 0, 0, 0, 0, 0, 0]
		elif (chr(k) == 'a'): # 
			Joy_set.axes = [0, 1, 1, 0, 0, 0, 0, 0]
		elif (chr(k) == 'd'): # 
			Joy_set.axes = [0, 1, -1, 0, 0, 0, 0, 0]
		print(Joy_set)
		pub.publish(Joy_set)

if __name__ == '__main__':
	try:
		Key_joy()
	except rospy.ROSInterruptException:
		pass
