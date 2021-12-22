from src.v0.config import *


class DataAssociation:
    """
    Arguments:
        data : single message stream with one timestamp's worth of information.
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
        self.update_global_timestamp(time)

    def update_global_timestamp(self, time):
        self.global_timestamp = time

    def get_filtered_me(self):
        filtered_objs = []
        for obj in self.me_objects:
            if not (obj.dx < MIN_DX or
                    obj.dx > MAX_DX or
                    abs(obj.dy) > MAX_DY):
                filtered_objs.append(obj)
        return filtered_objs

    def get_filtered_radar(self):
        filtered_objs = []
        for obj in self.radar_objects:
            if not (obj.dx < MIN_DX or
                    obj.dx > MAX_DX or
                    abs(obj.dy) > MAX_DY):
                filtered_objs.append(obj)
        return filtered_objs

    @staticmethod
    def do_objects_match(obj_a, obj_b):
        dx = abs(obj_a.dx - obj_b.dx)
        dy = abs(obj_a.dy - obj_b.dy)
        vx = abs(obj_a.vx - obj_b.vx)

        if dx < DX_TOL and dy < DY_TOL and vx < VX_TOL:
            return 1
        return 0

    def delete_potential_objects(self):
        for i, obj in enumerate(DataAssociation.potential_objs[:]):
            if self.global_timestamp - obj.timestamp > SECONDS_TO_DELETE:
                DataAssociation.potential_objs = DataAssociation.potential_objs[i+1:]

    def add_tracked_object(self, f_obj, obj_id=None):
        if obj_id is not None:
            f_obj.id = obj_id
        self.tracked_objects.append(f_obj)

    def match_objects(self, filtered_objs, callback):
        for f_obj in filtered_objs:
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
                        obj.dx = f_obj.dx
                        obj.dy = f_obj.dy
                        obj.count += 1

                        if obj.count > POTENTIAL_THRESHOLD:
                            f_obj.id = DataAssociation.next_id
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
        filtered_objs = self.get_filtered_me()
        self.valid_detections.extend(filtered_objs)
        self.match_objects(filtered_objs, self.kf_node.me_callback)

    def sensor_radar_callback(self):
        filtered_objs = self.get_filtered_radar()
        self.valid_detections.extend(filtered_objs)
        self.match_objects(filtered_objs, self.kf_node.radar_callback)