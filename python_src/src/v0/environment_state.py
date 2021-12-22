import pandas as pd
import plotly.express as px
from tqdm import tqdm

from src.v0.config import ObjectState


class EnvironmentState:
    history = []

    def __init__(self):
        self.tracked_objects = {}
        self.target_objects_in_lanes = []
        self.last_timestamp = None

    def reset_tracks(self):
        self.target_objects_in_lanes = [ObjectState() for _ in range(3)]

    def filtered_object_callback(self, obj):
        self.history.append(obj)
        self.update_env_state(obj)
        self.last_timestamp = obj.timestamp

    def update_env_state(self, obj):
        self.tracked_objects[obj.id] = obj

    def find_target_object(self):
        for tracked_object in self.tracked_objects:
            obj_id = tracked_object.id
            dx = tracked_object.dx
            lane_1 = self.target_objects_in_lanes[0]
            lane_2 = self.target_objects_in_lanes[1]
            lane_3 = self.target_objects_in_lanes[2]
            tracked_lane = self.tracked_objects[obj_id].lane
            if tracked_lane == 1:
                if dx <= lane_1.dx or obj_id == lane_1.id or lane_1.dx == 0:
                    self.target_objects_in_lanes[0] = tracked_object
                else:
                    self.target_objects_in_lanes[0] = ObjectState()
            elif tracked_lane == 2:
                if dx <= lane_2.dx or obj_id == lane_2.id or lane_2.dx == 0:
                    self.target_objects_in_lanes[1] = tracked_object
                else:
                    self.target_objects_in_lanes[1] = ObjectState()
            elif tracked_lane == 3:
                if dx <= lane_3.dx or obj_id == lane_3.id or lane_3.dx == 0:
                    self.target_objects_in_lanes[2] = tracked_object
                else:
                    self.target_objects_in_lanes[2] = ObjectState()

    def plot_history(self, animate: bool = True, **kwargs):
        df = pd.DataFrame(columns=['time', 'id', 'dx', 'dy'])
        for obj in tqdm(self.history, total=len(self.history)):
            row = {
                'time': obj.timestamp,
                'id': str(obj.id),
                'dx': obj.dx[0],
                'dy': obj.dy[0]
            }
            df = df.append(row, ignore_index=True)

        max_dx = df.dx.max() + 10
        if animate:
            df['bins'] = pd.cut(df.time.tolist(), 150).codes
            fig = px.scatter(df, x='dy', y='dx', color='id', animation_frame='bins',
                             hover_data=['time'], range_x=[-10, 10], range_y=[0, max_dx], **kwargs)
        else:
            fig = px.scatter(df, x='dy', y='dx', color='id', hover_data=['time'],
                             range_x=[-10, 10], range_y=[0, max_dx], **kwargs)
        fig.update_layout(legend=dict(title='Object ID'), font=dict(size=20))
        fig.show()
