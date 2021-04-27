#!/usr/bin/env python
from __future__ import print_function, division
import rospy
import numpy as np 
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from common.msg import object_deletion_msg, filtered_object_msg
from common.msg import associated_me_msg, associated_radar_msg
import time
import matplotlib.pyplot as plt
from threading import Lock

# Shouldn't need this definition - have data from mobileye
# 1 = center lane, 2 = left lane, 3 = right lane
def determine_lane(dy):
    if dy < -1.5:
        return 2, False
    elif dy > 1.5:
        return 3, False
    else:
        return 1, True

class KF(KalmanFilter):
    def __init__(self, initial_measurement):
        '''
        initial_measurement is just z

        Nomenclature:
        x is the estimated state vector, ie in the state space
        F is the transition matrix, ie the prediction function
        P is the state covariance matrix, ie the accumulated covariance values for all state variables
        z is the measurement vector, ie in the measurement space
        R is the measurement covariance matrix, ie the covariance values of every measurement variable
        Q is the process noise matrix, ie it encodes the uncertainty regarding to F by adding itself to P when computing the prior
        H is the "conversion matrix", ie it linearly maps the state space to the measurement space
        '''
        super(KF, self).__init__(4, 4)
        self.x[0], self.x[1] = initial_measurement[0], initial_measurement[1]
        self.F = np.identity(4)
        np.fill_diagonal(self.P, [25 for _ in range(4)])
        self.last_radar_timestamp = 0
        self.last_me_timestamp = 0

class KF_Node(object):
    def me_association_callback(self, obj):
        rospy.loginfo('me callback called %d @ %f, %f' % (obj.obj_id, obj.me_dx, obj.me_dy))
        measurement = [obj.me_dx,
                       obj.me_dy,
                       obj.me_vx]

        self.resource_lock.acquire()

        if not obj.obj_id in self.objects:
            self.objects[obj.obj_id] = KF(measurement)
            self.objects[obj.obj_id].last_me_timestamp = obj.me_timestamp

            rospy.loginfo('creating object %d' % obj.obj_id)
            self.input_history[obj.obj_id] = np.array([[measurement[0], measurement[1]]])
            self.output_history[obj.obj_id] = np.array([self.objects[obj.obj_id].x])

        else:
            hashed = self.objects[obj.obj_id]

            if hashed.last_me_timestamp:
                hashed.dim_z = 3
                hashed.H = np.array([[1,0,0,0],
                                     [0,1,0,0],
                                     [0,0,1,0]], dtype='float')
                
                # change me for the mobileye covariance!!!
                hashed.R = np.identity(3) * .5**2
                hashed.dt = obj.me_timestamp - hashed.last_me_timestamp
                hashed.last_me_timestamp = obj.me_timestamp
                hashed.F[0][2] = hashed.F[1][3] = hashed.dt
                # change me for process noise (3rd argument)!!!
                hashed.Q = Q_discrete_white_noise(2, hashed.dt, 1.5**2, block_size=2)

                hashed.predict()
                hashed.update(measurement)

                if hashed.x[0] > 250:
                    hashed.x[0] = 250 
                if hashed.x[0] < 0:
                    hashed.x[0] = 0
                if hashed.x[1] > 32:
                    hashed.x[1] = 32 
                if hashed.x[1] < -32:
                    hashed.x[1] = -32 

                # print('x',self.objects[0].x, 'P', self.objects[0].P, 'z', self.objects[0].z, 'R', self.objects[0].R, 'dt', self.objects[0].dt, sep='\n', end='\n--------------\n')
            else:
                hashed.last_me_timestamp = obj.me_timestamp

            self.input_history[obj.obj_id] = np.append(self.input_history[obj.obj_id], [[measurement[0],measurement[1]]], axis=0)
            self.output_history[obj.obj_id] = np.append(self.output_history[obj.obj_id], [hashed.x], axis=0)

        result = filtered_object_msg()
        result.obj_dx, result.obj_dy, result.obj_vx, result.obj_vy = self.objects[obj.obj_id].x
        self.resource_lock.release()
        result.obj_id = obj.obj_id 
        result.obj_lane, result.obj_path = determine_lane(result.obj_dy)
        # result.obj_ax = obj.obj.me_ax
        result.obj_timestamp = obj.me_timestamp
        result.obj_count = 30
        
        self.output.publish(result)

    def radar_association_callback(self, obj):
        rospy.loginfo('radar callback called %d @ %f, %f' % (obj.obj_id, obj.radar_dx, obj.radar_dy))
        measurement = [obj.radar_dx,
                       obj.radar_dy,
                       obj.radar_vx,
                       obj.radar_vy]

        self.resource_lock.acquire()

        if not obj.obj_id in self.objects:
            self.objects[obj.obj_id] = KF(measurement)
            self.objects[obj.obj_id].last_radar_timestamp = obj.radar_timestamp

            rospy.loginfo('creating object %d' % obj.obj_id)
            self.input_history[obj.obj_id] = np.array([[measurement[0], measurement[1]]])
            self.output_history[obj.obj_id] = np.array([self.objects[obj.obj_id].x])

        else:
            hashed = self.objects[obj.obj_id]

            if hashed.last_radar_timestamp:
                hashed.dim_z = 4
                # hashed.H = np.array([[1,0,0,0],
                #                      [0,1,0,0]])
                hashed.H = np.identity(4)
                # hashed.R = np.array([[1**2,0],
                #                      [0,1**2]])
                hashed.R = np.identity(4)

                # change me for radar covariance!!!
                np.fill_diagonal(hashed.R, [1**2 for i in range(4)])
                hashed.dt = obj.radar_timestamp - hashed.last_radar_timestamp
                hashed.last_radar_timestamp = obj.radar_timestamp
                hashed.F[0][2] = hashed.F[1][3] = hashed.dt
                # change me for process noise (3rd argument)!!!
                hashed.Q = Q_discrete_white_noise(2, hashed.dt, 1.5**2, block_size=2)

                hashed.predict()
                hashed.update(measurement)

                if hashed.x[0] > 255:
                    hashed.x[0] = 255 
                if hashed.x[0] < 0:
                    hashed.x[0] = 0
                if hashed.x[1] > 128:
                    hashed.x[1] = 128 
                if hashed.x[1] < -128:
                    hashed.x[1] = -128 
                # print('x',self.objects[0].x, 'P', self.objects[0].P, 'z', self.objects[0].z, 'R', self.objects[0].R, 'dt', self.objects[0].dt, sep='\n', end='\n--------------\n')
            else:
                hashed.last_radar_timestamp = obj.radar_timestamp

            self.input_history[obj.obj_id] = np.append(self.input_history[obj.obj_id], [[measurement[0], measurement[1]]], axis=0)
            self.output_history[obj.obj_id] = np.append(self.output_history[obj.obj_id], [hashed.x], axis=0)

        result = filtered_object_msg()
        result.obj_dx, result.obj_dy, result.obj_vx, result.obj_vy = self.objects[obj.obj_id].x
        self.resource_lock.release()
        result.obj_id = obj.obj_id 
        result.obj_lane, result.obj_path = determine_lane(result.obj_dy)
        result.obj_ax = obj.radar_ax
        result.obj_timestamp = obj.radar_timestamp
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
        self.resource_lock = Lock()

        self.input_history, self.output_history = {}, {}
        self.start = time.time()

    def plot_history(self, id):
        # print(self.input_history[id])
        # print('-----\n',self.output_history[id])
        x = np.arange(len(self.input_history[id]))
        plt.figure(1)
        plt.suptitle('Kalman Filter Object Tracking Performance')
        plt.subplot(121)
        plt.plot(x, self.input_history[id][:,0], 'r--', x, self.output_history[id][:,0], 'b-')
        plt.ylabel('Longitudinal Distance (m)')
        plt.xlabel('Number of Updates to the Object')
        plt.title('Change in Longitudinal Distance Over Time')
        plt.legend(['Mixed Sensor Input','Filter Output'])
        plt.subplot(122)
        plt.plot(x, self.input_history[id][:,1], 'r--', x, self.output_history[id][:,1], 'b-')
        plt.ylabel('Lateral Distance (m)')
        plt.xlabel('Number of Updates to the Object')
        plt.title('Change in Lateral Distance Over Time')
        plt.legend(['Mixed Sensor Input','Filter Output'])
        plt.show()

if __name__ == '__main__':
    node = KF_Node()
    rospy.spin()
    
    # try:
    #     while not rospy.core.is_shutdown():
    #         rospy.spin()
    #         # arg = raw_input()
    #         #if arg[0] == 'r':
    #         #    node.plot_history(int(arg[1:]))
    #         pass
    # except:
    #     rospy.loginfo('keyboard interrupt')
