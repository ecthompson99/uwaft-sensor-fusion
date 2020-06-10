#!/usr/bin/env python
from __future__ import division, print_function
import rospy
from common/sensor_fusion.msg import radar_object_data, mobileye_object_data
from numpy.random import randn

rospy.init_node('grand_input_node')
me = rospy.Publisher('mobileye_from_matlab', mobileye_object_data, queue_size=10)
ra = rospy.Publisher('radar_from_matlab',radar_object_data, queue_size=10)

radar_sample = radar_object_data()
radar_sample.RadarDx, radar_sample.RadarDy, radar_sample.RadarVx, radar_sample.RadarVy, radar_sample.RadarAx = 0,0,1,1,0
radar_sample.RadarTimestamp = 0
radar_sample.RadarDxSigma, radar_sample.RadarDySigma, radar_sample.RadarVxSigma, radar_sample.RadarAxSigma = .1, .2, .1, .2

me_sample = mobileye_object_data()
me_sample.MeDx, me_sample.MeDy, me_sample.MeVx, me_sample.MeTimestamp = 0,0,1,0

global_clk = 0

while not rospy.core.is_shutdown():
    raw_input()
    global_clk += .02
    radar_sample.RadarTimestamp += 1/50
    radar_sample.RadarDx = radar_sample.RadarTimestamp + randn()*.1
    radar_sample.RadarDy = radar_sample.RadarTimestamp + randn()*.1
    radar_sample.RadarVx, radar_sample.RadarVy = randn()*.1+1, randn()*.1+1
    print(radar_sample)
    ra.publish(radar_sample)
    if (me_sample.MeTimestamp + .04) <= global_clk:
        raw_input()
        me_sample.MeDx += 1/25 + randn()*.25
        me_sample.MeDy += 1/25 + randn()*.1
        me_sample.MeVx = randn()*.25+1
        me_sample.MeTimestamp += 1/25
        print(me_sample)
        me.publish(me_sample)
