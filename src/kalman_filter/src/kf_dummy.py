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
		dummy_objects[i].obj_id = i+1

	i = 0
	try:
		while True:
			for a in dummy_objects:
				offset = a.obj_id
				a.obj_dx = i + offset
				a.obj_lane = offset - 1
				a.obj_vx = i + offset
				a.obj_dy = i + offset
				a.obj_ax = i + offset
				a.obj_timestamp = i + offset
				a.obj_path = i%2 == 0
				a.obj_vy = i + offset
				a.obj_count = i + offset
				sender.publish(a)
				time.sleep(.01)
			time.sleep(.5)
			i += 1
	except KeyboardInterrupt:
		exit()

	print('finished publishing, idling now')
	rospy.spin()

if __name__ == '__main__':
	main()
