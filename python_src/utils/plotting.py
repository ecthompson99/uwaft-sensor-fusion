"""Plotting utility for visualizing sensor fusion output.

Written by: Ethan Thompson (contact ec3thomp@uwaterloo.ca)
Date: 17 December 2021
"""
import pandas as pd
from plotly import express as px
from tqdm import tqdm

from src.v1.config import MobileyeObject, RadarObject


def plot_objects(objects: list, animate: bool = True, **kwargs):
    f"""Generates a plot showing the trajectory of detected objects.
    
    Can be a static plot showing all detections (dy vs. dx) and color coded by object ID, or
    can be an animated plot showing detections over time (dy vs. dx. vs. time) and color coded by object ID.
    
    Parameters
    ----------
    objects: list[{MobileyeObject, RadarObject}]
        List of objects detected by the sensors.
    animate: bool
        Generate an animated plot if set to True, otherwise generate a static plot. Defaults to True.
    """
    df = pd.DataFrame({'time': [], 'dx': [], 'dy': [], 'source': []})

    for object in tqdm(objects, total=len(objects)):
        source = 'radar' if isinstance(object, RadarObject) else 'mobileye'
        row = {
            'time': object.timestamp,
            'id': str(object.id),
            'dx': object.dx,
            'dy': object.dy,
            'vx': object.vx,
            'source': source
        }
        df = df.append(row, ignore_index=True)

    df = df.sort_values(by='time')

    if animate:
        df['bins'] = pd.cut(df.time.tolist(), 150).codes
        max_dx = df.dx.max() + 10
        fig = px.scatter(df, x='dy', y='dx', color='id', hover_data=['time', 'source', 'vx'], animation_frame='bins',
                         range_x=[-10, 10], range_y=[0, max_dx], **kwargs)
    else:
        fig = px.scatter(df, x='dy', y='dx', color='id', hover_data=['time', 'source', 'vx'], **kwargs)

    fig.update_layout(legend=dict(title='Object ID'), font=dict(size=14))
    fig.show()
