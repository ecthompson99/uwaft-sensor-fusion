#!/usr/bin/env python
from __future__ import print_function
import rospy
import time
from kalman_filter.msg import filtered_object_msg

def main():
	rospy.init_node('kf_dummy')
	sender = rospy.Publisher('kf_dummy_data', filtered_object_msg, queue_size=10)

	dummy_objects = [filtered_object_msg() for i in range(3)]
	for i in range(3):
		dummy_objects[0].obj_id = i+1

	for i in range(10):
		for a in dummy_objects:
			a.obj_dx = i
			a.obj_lane = i
			a.obj_vx = i
			a.obj_dy = i
			a.obj_ax = i
			a.obj_in_lane = i%2 == 0
			a.obj_vy = i
			sender.publish(a)
			time.sleep(.01)
		time.sleep(.5)

	print('finished publishing, idling now')
	rospy.spin()

if __name__ == '__main__':
	main()