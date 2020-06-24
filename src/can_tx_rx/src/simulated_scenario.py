#!/usr/bin/python
from __future__ import division, print_function, generators
import rospy
from math import sin, cos, pi, atan
from numpy.random import randn
from common.msg import radar_object_data, mobileye_object_data

# This scenario will last 3 mins, global_clk is a float in sec
radar_refresh = 1/16.7
me_refresh = 1/12

# Since our scenario is deterministic anyways, now I think it's a better idea to 
def fetch_scenario(clk):
    # for each frame there will be three objects for now:
    # one leading vehicle
    # one bypassing vehicle on the left and right lanes each
    # each object will be represented by a tuple (dx, radar_dy, me_dy, vx, vy)
    objects = []

    # left lane bypassing vehicle going from -3m to 200m in first 15s of 30s period
    if clk % 30 < 15:
        objects.append((clk%30*203/15-3, 2, 2, 203/15, 0))

    # right lane bypassing vehicle going from -3m to 200m in second 15s of 30s period
    if clk % 30 > 15:
        objects.append(((clk%30-15)*203/15-3, -2, -2, 203/15, 0))

    # lead oscillating vehicle of 30s period around 50m, 5m amplitude
    objects.append((5*sin(clk*pi/15)+50, 0, 0, pi/3*cos(clk*pi/15), 0))

    return objects

# frames here are lists (of tuples which are also iterables)
# pub is the publisher so call it with pub.publish(msg)
def simulate_radar(pub, frame, clk):
    for i in frame:
        if i[0] < .5 or i[0] > 160:
            continue
        
        distance = (i[0]**2 + i[1]**2)**.5

        if distance < 160 and abs(atan(i[1]/i[0])) < 10*pi/180 :
            obj = radar_object_data()
            obj.RadarDx = i[0] + randn()*.2
            obj.RadarDy = i[1] + randn()*.5
            obj.RadarVx = i[3] + randn()*.5
            obj.RadarVy = i[4] + randn()*.5
            obj.RadarTimestamp = clk
            pub.publish(obj)

        elif distance < 63 and abs(atan(i[1]/i[0])) < 45*pi/180 and abs(i[1]) < 20:
            obj = radar_object_data()
            obj.RadarDx = i[0] + randn()*.2
            obj.RadarDy = i[1] + randn()*.5
            obj.RadarVx = i[3] + randn()*.5
            obj.RadarVy = i[4] + randn()*.5
            obj.RadarTimestamp = clk
            pub.publish(obj)

def simulate_me(pub, frame, clk):
    for i in frame:
        if i[0] < 1 or i[0] > 200:
            continue
        distance = (i[0]**2 + i[2]**2)**.5
        if distance < 200 and abs(atan(i[2]/i[0])) < 38*pi/180:
            obj = mobileye_object_data()
            obj.MeDx = i[0] + randn()*.5
            obj.MeDy = i[2] + randn()*.5
            obj.MeVx = i[3] + randn()*.5
            obj.MeTimestamp = clk
            pub.publish(obj)


def main():
    global_clk = 0
    last_radar_clk, last_me_clk = radar_refresh, me_refresh
    rospy.init_node('sil_rx')
    radar_pub = rospy.Publisher('/radar_can_rx', radar_object_data, queue_size=10)
    me_pub = rospy.Publisher('/mobileye_can_rx', mobileye_object_data, queue_size=10)
    raw_input()
    while global_clk < 180:
        raw_input()
        # tick the clock until it catches up with the next refresh cycle of 
        while global_clk <= last_radar_clk and global_clk <= last_me_clk:
            global_clk += .01
        if global_clk >= last_radar_clk:
            simulate_radar(radar_pub, fetch_scenario(global_clk), global_clk)
            last_radar_clk += radar_refresh
        else:
            simulate_me(me_pub, fetch_scenario(global_clk), global_clk)
            last_me_clk += me_refresh

if __name__ == '__main__':
    main()
