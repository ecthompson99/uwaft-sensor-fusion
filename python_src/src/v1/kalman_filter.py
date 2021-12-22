"""Kalman Filter Node of the Sensor Fusion Architecture.

Written by: Ethan Thompson (contact ec3thomp@uwaterloo.ca)
Date: 17 December 2021
"""

import numpy as np
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter

from src.v1.config import FilteredObject


class KF(KalmanFilter):
    def __init__(self, initial_measurement):
        """
        initial_measurement is just z

        Nomenclature:
        x is the estimated state vector, ie in the state space
        F is the transition matrix, ie the prediction function
        P is the state covariance matrix, ie the accumulated covariance values for all state variables
        z is the measurement vector, ie in the measurement space
        R is the measurement covariance matrix, ie the covariance values of every measurement variable
        Q is the process noise matrix, ie it encodes the uncertainty regarding to F by adding itself to P when computing the prior
        H is the "conversion matrix", ie it linearly maps the state space to the measurement space
        """
        super(KF, self).__init__(4, 4)
        self.x[0], self.x[1] = initial_measurement[0], initial_measurement[1]
        self.F = np.identity(4)
        np.fill_diagonal(self.P, [25 for _ in range(4)])
        self.last_radar_timestamp = 0
        self.last_me_timestamp = 0


class KFNode:
    def __init__(self, env_state):
        self.objects = {}
        self.input_history = {}
        self.output_history = {}
        self.env_state = env_state

    def me_callback(self, obj):
        measurement = [obj.dx,
                       obj.dy,
                       obj.vx]

        if obj.id not in self.objects:
            self.objects[obj.id] = KF(measurement)
            self.objects[obj.id].last_me_timestamp = obj.timestamp

            self.input_history[obj.id] = np.array([[measurement[0], measurement[1]]])
            self.output_history[obj.id] = np.array([self.objects[obj.id].x])

        else:
            hashed = self.objects[obj.id]

            if hashed.last_me_timestamp:
                hashed.dim_z = 3
                hashed.H = np.array([[1, 0, 0, 0],
                                     [0, 1, 0, 0],
                                     [0, 0, 1, 0]], dtype='float')

                # change me for the mobileye covariance!!!
                hashed.R = np.identity(3) * .5 ** 2
                hashed.dt = obj.timestamp - hashed.last_me_timestamp
                hashed.last_me_timestamp = obj.timestamp
                hashed.F[0][2] = hashed.F[1][3] = hashed.dt
                # change me for process noise (3rd argument)!!!
                hashed.Q = Q_discrete_white_noise(2, hashed.dt, 1.5 ** 2, block_size=2)

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
            else:
                hashed.last_me_timestamp = obj.timestamp

            self.input_history[obj.id] = np.append(self.input_history[obj.id],
                                                       [[measurement[0], measurement[1]]], axis=0)
            self.output_history[obj.id] = np.append(self.output_history[obj.id], [hashed.x], axis=0)

        result = FilteredObject()
        result.dx, result.dy, result.vx, result.vy = self.objects[obj.id].x
        result.id = obj.id
        result.lane = obj.lane
        result.path = True if obj.lane == 1 else False
        result.timestamp = obj.timestamp
        result.count = 30

        self.env_state.filtered_object_callback(result)

    def radar_callback(self, obj):
        measurement = [obj.dx,
                       obj.dy,
                       obj.vx,
                       obj.vy]

        if obj.id not in self.objects:
            self.objects[obj.id] = KF(measurement)
            self.objects[obj.id].last_radar_timestamp = obj.timestamp

            self.input_history[obj.id] = np.array([[measurement[0], measurement[1]]])
            self.output_history[obj.id] = np.array([self.objects[obj.id].x])

        else:
            hashed = self.objects[obj.id]

            if hashed.last_radar_timestamp:
                hashed.dim_z = 4
                # hashed.H = np.array([[1,0,0,0],
                #                      [0,1,0,0]])
                hashed.H = np.identity(4)
                # hashed.R = np.array([[1**2,0],
                #                      [0,1**2]])
                hashed.R = np.identity(4)

                # change me for radar covariance!!!
                np.fill_diagonal(hashed.R, [1 ** 2 for i in range(4)])
                hashed.dt = obj.timestamp - hashed.last_radar_timestamp
                hashed.last_radar_timestamp = obj.timestamp
                hashed.F[0][2] = hashed.F[1][3] = hashed.dt
                # change me for process noise (3rd argument)!!!
                hashed.Q = Q_discrete_white_noise(2, hashed.dt, 1.5 ** 2, block_size=2)

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
            else:
                hashed.last_radar_timestamp = obj.timestamp

            self.input_history[obj.id] = np.append(self.input_history[obj.id],
                                                       [[measurement[0], measurement[1]]], axis=0)
            self.output_history[obj.id] = np.append(self.output_history[obj.id], [hashed.x], axis=0)

        result = FilteredObject()
        result.dx, result.dy, result.vx, result.vy = self.objects[obj.id].x
        result.id = obj.id
        result.lane = obj.lane
        result.path = True if obj.lane == 1 else False
        result.timestamp = obj.timestamp
        result.count = 30

        self.env_state.filtered_object_callback(result)