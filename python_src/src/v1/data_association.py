"""Data Association Node of the Sensor Fusion Architecture.

Written by: Ethan Thompson (contact ec3thomp@uwaterloo.ca)
Date: 17 December 2021
"""
from src.v1.config import *


class DataAssociation:
    """Track maintenance and assignment of detections to objects.

    Arguments:
        data : single message stream with one timestamp's worth of information.
        kf_node : The Kalman filter node
        env_state : The environment state node
        time : Timestamp of the streamObject
    """
    valid_detections = []
    tracked_objects = []
    potential_objs = []
    global_timestamp = 0
    next_id = 0

    def __init__(self, data, kf_node, env_state, time):
        self.me_objects = data.me_objects
        self.radar_objects = data.radar_objects
        self.kf_node = kf_node
        self.env_state = env_state
        self._update_global_timestamp(time)

    def _update_global_timestamp(self, time):
        self.global_timestamp = time

    def get_filtered_me(self):
        """Filters the Mobileye data based on pre-defined thresholds."""
        filtered_objs = []
        for obj in self.me_objects:
            if not (obj.dx < MIN_DX or
                    obj.dx > MAX_DX or
                    abs(obj.dy) > MAX_DY):
                filtered_objs.append(obj)
        return filtered_objs

    def get_filtered_radar(self):
        """Filters the Radar data based on pre-defined thresholds."""
        filtered_objs = []
        for obj in self.radar_objects:
            if not (obj.dx < MIN_DX or
                    obj.dx > MAX_DX or
                    abs(obj.dy) > MAX_DY):
                filtered_objs.append(obj)
        return filtered_objs

    @staticmethod
    def do_objects_match(obj_a, obj_b):
        f"""Returns if two objects are the same if they are closed to one another.

        Conditions:
            1. They are within DX_TOL of one another along the x-axis.
            2. They are within DY_TOL of one another along the y-axis.
            3. They are in the same lane.

        Parameters
        ----------
        obj_a: {MobileyeObject, RadarObject, FilteredObject}
            First object.
        obj_b: {MobileyeObject, RadarObject, FilteredObject}
            Second object

        Returns
        -------
        bool:
            True if objects match, false otherwise.

        """
        dx = abs(obj_a.dx - obj_b.dx)
        dy = abs(obj_a.dy - obj_b.dy)

        if obj_a.lane == obj_b.lane and dx < DX_TOL and dy < DY_TOL:
            return True
        return False

    def delete_potential_objects(self):
        """Deletes list of potential objects if it has not been updated in SECONDS_TO_DELETE seconds"""
        for i, obj in enumerate(DataAssociation.potential_objs[:]):
            if self.global_timestamp - obj.timestamp > SECONDS_TO_DELETE:
                DataAssociation.potential_objs = DataAssociation.potential_objs[i+1:]

    def add_tracked_object(self, f_obj, obj_id=None):
        """Extends tracked object history, used for sensor fusion output visualization."""
        if obj_id is not None:
            f_obj.id = obj_id
        self.tracked_objects.append(f_obj)

    @staticmethod
    def determine_lane(dy):
        """Determines the lane of an object given its dy value."""
        if dy < -1.5:
            return 2
        elif dy > 1.5:
            return 3
        else:
            return 1

    def match_objects(self, filtered_objs, callback):
        f"""Matches detections against environment state or list of potential objects.
        
        - First, try matching against objects within environment state
        - Second, try matching against objects in potential_objs
        - If not matched, add detection to list of potential_objs
        
        Parameters
        ----------
        filtered_objs: list[{RadarObject, MobileyeObject}]
            List of detetions from Radar or Mobileye
        callback: function
            Callback in KF_node for either Radar detections or Mobileye Detections
        """
        for f_obj in filtered_objs:
            f_obj.lane = self.determine_lane(f_obj.dy)
            matched = False
            for env_obj_id, env_obj in self.env_state.tracked_objects.items():
                if self.do_objects_match(f_obj, env_obj):
                    self.add_tracked_object(f_obj, env_obj.id)
                    callback(f_obj)
                    matched = True
                    break

            if not matched:
                for obj in DataAssociation.potential_objs[:]:
                    if self.do_objects_match(obj, f_obj):
                        obj.timestamp = self.global_timestamp

                        # Weighted average
                        num_objs = obj.count + 1
                        obj_w = obj.count/num_objs
                        f_obj_w = 1/num_objs
                        obj.dx = obj_w*obj.dx + f_obj_w*f_obj.dx
                        obj.dy = obj_w*obj.dy + f_obj_w*f_obj.dy

                        obj.count += 1

                        if obj.count > POTENTIAL_THRESHOLD:
                            f_obj.id = DataAssociation.next_id
                            f_obj.dx = obj.dx
                            f_obj.dy = obj.dy
                            DataAssociation.next_id += 1
                            self.add_tracked_object(f_obj)
                            callback(f_obj)
                            DataAssociation.potential_objs.remove(obj)

                        matched = True
                        break

                self.delete_potential_objects()

            if not matched:
                DataAssociation.potential_objs.append(f_obj)

    def sensor_me_callback(self):
        """Called when mobileye produces a new detection."""
        filtered_objs = self.get_filtered_me()
        self.valid_detections.extend(filtered_objs)
        self.match_objects(filtered_objs, self.kf_node.me_callback)

    def sensor_radar_callback(self):
        """Called when radar produces a new detection."""
        filtered_objs = self.get_filtered_radar()
        self.valid_detections.extend(filtered_objs)
        self.match_objects(filtered_objs, self.kf_node.radar_callback)