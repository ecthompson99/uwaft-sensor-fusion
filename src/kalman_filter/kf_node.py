#!/usr/bin/env python
import rospy
import numpy as np 
from __future__ import print_function
from kalman_filter.msg import filtered_object_msg, object_deletion_msg

class Kalman_Filter:
    def __init__(self):
        pass
    def update(self):
        pass

class KF_Node:
    def association_callback(self, filtered_object):
        pass
    def object_deletion_callback(self, obj):
        pass
    def __init__(self):
        rospy.init_node('kf_node')

def main():
    rospy.spin()

if __name__ == '__main__':
    main()
