#!/usr/bin/env python3
import rospy
import numpy as np 
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from __future__ import print_function, division
from kalman_filter.msg import object_deletion_msg, filtered_object_msg
from sensor_fusion.msg import associated_me_msg, associated_radar_msg

def determine_lane(dy):
    if dy < -1.5:
        return 0, False
    elif dy > 1.5:
        return 2, False
    else:
        return 1, True

class KF(KalmanFilter):
    def __init__(self, dt, initial_measurement):
        # initial_measurement is a list containing z and R's diagonal
        super().__init__(4, 4)
        self.dt = dt
        self.x = np.asarray(initial_measurement[0])
        np.fill_diagonal(self.R, initial_measurement[1])
        # Constant velocity model
        self.F = np.array([[1,0,dt,0],
                           [0,1,0,dt],
                           [0,0,1,0],
                           [0,0,0,1]])
        # Tunable process variance here
        self.Q = Q_discrete_white_noise(2, self.dt, .5**2, block_size=2)
        np.fill_diagonal(self.P, [100 for _ in range(4)])

class KF_Node:
    def me_association_callback(self, obj):
        measurement = [[obj.obj.me_dx,
                        obj.obj.me_dy,
                        obj.obj.me_vx,
                        0],
                        # Tunable
                        [.5**2,
                         .5**2,
                         .5**2,
                         .5**2]]

        if not obj.obj_id in self.objects:
            self.objects[obj.obj_id] = KF(1/15, measurement)
            self.objects[obj.obj_id]._vy = (0, 0)
        else:
            hashed = self.objects[obj.obj_id]
            hashed.predict()
            measurement[0][3] = (hashed._vy[1] - hashed._vy[0]) / hashed.dt
            # R is going to be constant for mobileye for now
            self.objects[obj.obj_id].update(measurement[0])
            hashed._vy = (hashed._vy[1], hashed.x[3])

        result = filtered_object_msg()
        result.obj_dx, result.obj_dy, result.obj_vx, result.obj_vy = self.objects[obj.obj_id].x
        result.obj_id = obj.obj_id 
        result.obj_lane, result.obj_path = determine_lane(result.obj_dy)
        result.ax = obj.obj.me_ax
        result.obj_timestamp = obj.obj.me_timestamp
        result.obj_count = 30
        
        self.output.publish(result)

    def radar_association_callback(self, obj):
        measurement = [[obj.obj.radar_dx,
                        obj.obj.radar_dy,
                        obj.obj.radar_vx,
                        obj.obj.radar_vy],
                        # Tunable
                        [obj.obj.radar_dx_sigma**2,
                         obj.obj.radar_dy_sigma**2,
                         obj.obj.radar_vx_sigma**2,
                         .25**2]]

        if not obj.obj_id in self.objects:
            self.objects[obj.obj_id] = KF(1/50, measurement)
        else:
            hashed = self.objects[obj.obj_id]
            hashed.predict()
            np.fill_diagonal(hashed.R, measurement[1])
            hashed.update(measurement[0])

        result = filtered_object_msg()
        result.obj_dx, result.obj_dy, result.obj_vx, result.obj_vy = self.objects[obj.obj_id].x
        result.obj_id = obj.obj_id 
        result.obj_lane, result.obj_path = determine_lane(result.obj_dy)
        result.ax = obj.obj.radar_ax
        result.obj_timestamp = obj.obj.radar_timestamp
        result.obj_count = 30

        self.output.publish(result)

    def object_deletion_callback(self, obj):
        del self.objects[obj.obj_id]

    def __init__(self):
        rospy.init_node('kf_node')
        self.objects = {}
        rospy.Subscriber('associated_me', self.me_association_callback)
        rospy.Subscriber('associated_radar', self.radar_association_callback)
        rospy.Subscriber('obj_deletion', self.object_deletion_callback)
        self.output = rospy.Publisher('filtered_obj', filtered_object_msg)

if __name__ == '__main__':
    KF_Node()
    rospy.spin()
