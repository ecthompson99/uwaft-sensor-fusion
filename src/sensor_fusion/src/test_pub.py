#!/usr/bin/env python2
from __future__ import print_function, division
import rospy
from common.msg import radar_object_data

if __name__ == '__main__':
    rospy.init_node('dashboard_test')
    pub = rospy.Publisher('can_rx', radar_object_data, queue_size=10)
    rate = rospy.Rate(10)
    msg = radar_object_data()
    msg.RadarDx = 2
    msg.radar_num = 2
    speed = 5
    while not rospy.is_shutdown():
        msg.RadarDy = -1.5
        if msg.RadarDx > 150 or msg.RadarDx < 1:
            speed *= -1
        msg.RadarDx += speed
        msg.dLength = msg.RadarDx/200*7
        pub.publish(msg)
        msg.RadarDy = 1.5
        pub.publish(msg)
        rate.sleep()
