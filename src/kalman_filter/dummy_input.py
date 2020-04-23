#!/usr/bin/env python
import rospy
from sensor_fusion.msg import radar_object_data, mobileye_object_data

rospy.init_node('grand_input_node')
me = rospy.Publisher('mobileye_from_matlab', mobileye_object_data, queue_size=10)
ra = rospy.Publisher('radar_from_matlab',radar_object_data, queue_size=10)

radar_sample = radar_object_data()
radar_sample.RadarDx, radar_sample.RadarDy, radar_sample.RadarVx, radar_sample.RadarVy, radar_sample.RadarAx = 1,1,0,0,0
radar_sample.RadarTimestamp = 0
radar_sample.RadarDxSigma, radar_sample.RadarDySigma, radar_sample.RadarVxSigma, radar_sample.RadarAxSigma = .5, .5, .5, .5

while not rospy.core.is_shutdown():
    raw_input()
    radar_sample.RadarTimestamp += 1/50
    ra.publish(radar_sample)
