#!/usr/bin/env python
from __future__ import print_function, division
import rospy
import numpy as np 
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from kalman_filter.msg import object_deletion_msg, filtered_object_msg
from sensor_fusion.msg import associated_me_msg, associated_radar_msg
import time
import matplotlib.pyplot as plt

def determine_lane(dy):
    if dy < -1.5:
        return 0, False
    elif dy > 1.5:
        return 2, False
    else:
        return 1, True

class KF(KalmanFilter):
    def __init__(self, initial_measurement):
        # initial_measurement is a list containing z and R's diagonal
        super(KF, self).__init__(4, 4)
        # self.dt = dt
        self.x = np.asarray(initial_measurement[0])
        np.fill_diagonal(self.R, initial_measurement[1])
        # Constant velocity model
        self.F = np.array([[1,0,0,0],
                           [0,1,0,0],
                           [0,0,1,0],
                           [0,0,0,1]])
        np.fill_diagonal(self.P, [100 for _ in range(4)])
        self.last_timestamp = 0

class KF_Node(object):
    def me_association_callback(self, obj):
        print('me callback called')
        measurement = [[obj.obj.MeDx,
                        obj.obj.MeDy,
                        obj.obj.MeVx,
                        0],
                        # Tunable
                        [.5**2,
                         .5**2,
                         .5**2,
                         .5**2]]

        if not obj.obj_id in self.objects:
            self.objects[obj.obj_id] = KF(measurement)
            self.objects[obj.obj_id]._vy = (0, 0)
            self.objects[obj.obj_id].last_timestamp = obj.obj.MeTimestamp

            print('creating object', obj.obj_id)
            self.input_history[obj.obj_id] = np.array([measurement[0]])
            self.output_history[obj.obj_id] = np.array([self.objects[obj.obj_id].x])

        else:
            hashed = self.objects[obj.obj_id]
            hashed.dt = obj.obj.MeTimestamp - hashed.last_timestamp
            hashed.F[0][2] = hashed.F[1][3] = hashed.dt
            self.Q = Q_discrete_white_noise(2, hashed.dt, .5**2, block_size=2)
            hashed.predict()
            measurement[0][3] = (hashed._vy[1] - hashed._vy[0]) / hashed.dt
            # R is going to be constant for mobileye for now
            self.objects[obj.obj_id].update(measurement[0])
            hashed._vy = (hashed._vy[1], hashed.x[3])

            hashed_input = self.input_history[obj.obj_id]
            hashed_input = np.append(hashed_input, [measurement[0]], axis=0)
            hashed_output = self.output_history[obj.obj_id]
            hashed_output = np.append(hashed_output, [hashed.x], axis=0)

        result = filtered_object_msg()
        result.obj_dx, result.obj_dy, result.obj_vx, result.obj_vy = self.objects[obj.obj_id].x
        result.obj_id = obj.obj_id 
        result.obj_lane, result.obj_path = determine_lane(result.obj_dy)
        # result.obj_ax = obj.obj.me_ax
        result.obj_timestamp = obj.obj.MeTimestamp
        result.obj_count = 30
        
        self.output.publish(result)

    def radar_association_callback(self, obj):
        print('radar callback called', obj.obj_id)
        measurement = [[obj.obj.RadarDx,
                        obj.obj.RadarDy,
                        obj.obj.RadarVx,
                        obj.obj.RadarVy],
                        # Tunable
                        [obj.obj.RadarDxSigma**2,
                         obj.obj.RadarDySigma**2,
                         obj.obj.RadarVxSigma**2,
                         .25**2]]

        if not obj.obj_id in self.objects:
            self.objects[obj.obj_id] = KF(measurement)
            self.objects[obj.obj_id].last_timestamp = obj.obj.RadarTimestamp

            print('creating object', obj.obj_id)
            self.input_history[obj.obj_id] = np.array([measurement[0]])
            self.output_history[obj.obj_id] = np.array([self.objects[obj.obj_id].x])

        else:
            # print(self.objects[0])
            hashed = self.objects[obj.obj_id]
            hashed.dt = obj.obj.RadarTimestamp - hashed.last_timestamp
            hashed.F[0][2] = hashed.F[1][3] = hashed.dt
            hashed.Q = Q_discrete_white_noise(2, hashed.dt, .5**2, block_size=2)
            hashed.predict()
            np.fill_diagonal(hashed.R, measurement[1])
            hashed.update(measurement[0])

            hashed_input = self.input_history[obj.obj_id]
            hashed_input = np.append(hashed_input, [measurement[0]], axis=0)
            hashed_output = self.output_history[obj.obj_id]
            hashed_output = np.append(hashed_output, [hashed.x], axis=0)

        result = filtered_object_msg()
        result.obj_dx, result.obj_dy, result.obj_vx, result.obj_vy = self.objects[obj.obj_id].x
        result.obj_id = obj.obj_id 
        result.obj_lane, result.obj_path = determine_lane(result.obj_dy)
        result.obj_ax = obj.obj.RadarAx
        result.obj_timestamp = obj.obj.RadarTimestamp
        result.obj_count = 30

        self.output.publish(result)

    def object_deletion_callback(self, obj):
        del self.objects[obj.obj_id]

    def __init__(self):
        rospy.init_node('kf_node')
        self.objects = {}
        rospy.Subscriber('associated_me', associated_me_msg, callback=self.me_association_callback)
        rospy.Subscriber('associated_radar', associated_radar_msg, callback=self.radar_association_callback)
        rospy.Subscriber('obj_deletion', object_deletion_msg, callback=self.object_deletion_callback)
        self.output = rospy.Publisher('filtered_obj', filtered_object_msg, queue_size=10)

        self.input_history, self.output_history = {}, {}
        self.start = time.time()

    def plot_history(self, id):
        x = np.arange((time.time()-self.start)*50)
        plt.plot(x, self.input_history[id], 'r--', x, self.output_history[id], 'b-')
        plt.show()

if __name__ == '__main__':
    node = KF_Node()

    while not rospy.core.is_shutdown():
        arg = raw_input()
        if arg[0] == 'r':
            node.plot_history(int(arg[1:]))

    # rospy.spin()
