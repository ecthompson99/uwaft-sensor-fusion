"""Preprocessing utility for formatting sensor fusion input.

Written by: Ethan Thompson (contact ec3thomp@uwaterloo.ca)
Date: 17 December 2021
"""

import os
import pickle

import pandas as pd
from tqdm import tqdm

from src.v1.config import RadarObject, MobileyeObject, StreamObject
# from src.v1.config import *


def label_columns(data: pd.DataFrame) -> pd.DataFrame:
    """Labels the column names in the CSV file with the raw sensor data generated using
    kaiROS/sensor fusion testing/driving_scenario_data_extraction.m.

    The columns are:
        1 :  Timestamp
        2 - 33: dx for radar 1 objects (32 objects)
        34 - 65: dy for radar 1 objects (32 objects)
        66 - 97: vx for radar 1 objects (32 objects)
        98 - 129: vy for radar 1 objects (32 objects)
        130 - 161: dx for radar 2 objects (32 objects)
        162 - 193: dy for radar 2 objects (32 objects)
        194 - 225: vx for radar 2 objects (32 objects)
        226 - 257: vy for radar 2 objects (32 objects)
        258 - 289: dx for radar 3 objects (32 objects)
        290 - 321: dy for radar 3 objects (32 objects)
        322 - 353: vx for radar 3 objects (32 objects)
        354 - 385: vy for radar 3 objects (32 objects)
        386 - 395: dx for Mobileye objects (10 objects)
        396 - 405: dy for Mobileye objects (10 objects)
        406 - 415: vx for Mobileye objects (10 objects)
        416 : Ego velocity

    Parameters
    ----------
    data: pandas.core.frame.DataFrame
        Dataframe containing the raw sensor data.

    Returns
    -------
    pandas.core.frame.DataFrame
        Dataframe containing the raw sensor data with labelled columns.
    """
    data.columns = [
        "time",
        *[f"radar_1_object_{i + 1}_dx" for i in range(32)],
        *[f"radar_1_object_{i + 1}_dy" for i in range(32)],
        *[f"radar_1_object_{i + 1}_vx" for i in range(32)],
        *[f"radar_1_object_{i + 1}_vy" for i in range(32)],
        *[f"radar_2_object_{i + 1}_dx" for i in range(32)],
        *[f"radar_2_object_{i + 1}_dy" for i in range(32)],
        *[f"radar_2_object_{i + 1}_vx" for i in range(32)],
        *[f"radar_2_object_{i + 1}_vy" for i in range(32)],
        *[f"radar_3_object_{i + 1}_dx" for i in range(32)],
        *[f"radar_3_object_{i + 1}_dy" for i in range(32)],
        *[f"radar_3_object_{i + 1}_vx" for i in range(32)],
        *[f"radar_3_object_{i + 1}_vy" for i in range(32)],
        *[f"mobileye_object_{i + 1}_dx" for i in range(10)],
        *[f"mobileye_object_{i + 1}_dy" for i in range(10)],
        *[f"mobileye_object_{i + 1}_vx" for i in range(10)],
        "ego_velocity"
    ]

    return data


def to_long_df(data: pd.DataFrame, id_vars: list = None, value_vars: list = None) -> pd.DataFrame:
    """Converts the wide dataframe to a long format.

    See: https://pandas.pydata.org/docs/reference/api/pandas.melt.html

    Parameters
    ----------
    data: pandas.core.frame.DataFrame
    id_vars: list[str]
        List of column names that are unique identifiers.
    value_vars: list[str]
        List of column names that will be converted to rows.

    Returns
    -------
    pandas.core.frame.DataFrame:
        Dataframe converted to long format.
    """
    if id_vars is None:
        id_vars = ['time']
    if value_vars is None:
        value_vars = [
                        *[f"radar_1_object_{i + 1}_dx" for i in range(32)],
                        *[f"radar_1_object_{i + 1}_dy" for i in range(32)],
                        *[f"radar_1_object_{i + 1}_vx" for i in range(32)],
                        *[f"radar_1_object_{i + 1}_vy" for i in range(32)],
                        *[f"radar_2_object_{i + 1}_dx" for i in range(32)],
                        *[f"radar_2_object_{i + 1}_dy" for i in range(32)],
                        *[f"radar_2_object_{i + 1}_vx" for i in range(32)],
                        *[f"radar_2_object_{i + 1}_vy" for i in range(32)],
                        *[f"radar_3_object_{i + 1}_dx" for i in range(32)],
                        *[f"radar_3_object_{i + 1}_dy" for i in range(32)],
                        *[f"radar_3_object_{i + 1}_vx" for i in range(32)],
                        *[f"radar_3_object_{i + 1}_vy" for i in range(32)],
                        *[f"mobileye_object_{i + 1}_dx" for i in range(10)],
                        *[f"mobileye_object_{i + 1}_dy" for i in range(10)],
                        *[f"mobileye_object_{i + 1}_vx" for i in range(10)]
                        ]
    data_long = pd.melt(data,
                        id_vars=id_vars,
                        value_vars=value_vars)
    return data_long


def populate_message_stream(stream: pd.DataFrame, data_long: pd.DataFrame) -> pd.DataFrame:
    """Generates a list of messages

    Parameters
    ----------
    stream: pd.DataFrame
        Empty dataframe to be populated with stream objects for each timestamp. Formatted as follows:
         #   Column         Dtype
        ---  ------         -----
         0   time           float64
         1   stream_object  float64

    data_long: pd.DataFrame
        Dataframe containing the raw sensor data obtained by running to_long_df().

    Returns
    -------
    pd.Dataframe:
        Dataframe populated with the object detection data for each timestamp.
    """
    timestamps = sorted(data_long.time.unique().tolist())

    for time in tqdm(timestamps):
        # Select all entries for current timestamp
        df = data_long[data_long.time == time]

        radar_objects = []
        mobileye_objects = []

        # Front radar objects (appears every 0.01 seconds)
        for i in range(32):
            dx = df.loc[df.variable == f'radar_1_object_{i+1}_dx', 'value'].item()
            dy = df.loc[df.variable == f'radar_1_object_{i+1}_dy', 'value'].item()
            vx = df.loc[df.variable == f'radar_1_object_{i+1}_vx', 'value'].item()
            vy = df.loc[df.variable == f'radar_1_object_{i+1}_vy', 'value'].item()

            radar_object = RadarObject(
                dx=dx, dy=dy, vx=vx, vy=vy, ax=0, timestamp=time,
                ax_sigma=0, dx_sigma=0, dy_sigma=0, vx_sigma=0, w_exist=1,
                flag_valid=1, d_length=1
            )
            radar_objects.append(radar_object)

        # Mobileye objects (appears only every 0.1s)
        for i in range(10):
            dx = df.loc[df.variable == f'mobileye_object_{i+1}_dx', 'value'].item()
            dy = df.loc[df.variable == f'mobileye_object_{i+1}_dy', 'value'].item()
            vx = df.loc[df.variable == f'mobileye_object_{i+1}_vx', 'value'].item()

            mobileye_object = MobileyeObject(
                dx=dx, dy=dy, vx=vx, timestamp=time, valid=1
            )
            mobileye_objects.append(mobileye_object)

        # Create one streamObject containing the Mobileye and Radar objects for each timestamp
        stream_object = StreamObject(
            radar_objects=radar_objects,
            me_objects=mobileye_objects
        )
        msg_stream = {
            'time': time,
            'stream_object': stream_object
        }

        stream = stream.append(msg_stream, ignore_index=True)

    return stream


def create_stream(filename: str, data_long: pd.DataFrame) -> pd.DataFrame:
    """Creates a stream dataframe containing the object detections for each timestamp.

    Creates a stream_data folder which caches the stream object. This will speed up
    runtime if the main.py script is run multiple times. Rather than creating a new stream
    dataframe every time, it will simply load the stream dataframe from the local cache.

    Parameters
    ----------
    filename: str
        Name of the file to save in cache.
    data_long: pd.DataFrame
        Dataframe containing the raw sensor data obtained by running to_long_df().

    Returns
    -------
    pd.DataFrame:
        Dataframe populated with the object detection data for each timestamp.

    """
    os.makedirs('../stream_data', exist_ok=True)

    if filename not in os.listdir('../stream_data/'):
        stream = pd.DataFrame({
            'time': [],
            'stream_object': []
        })
        stream = populate_message_stream(stream, data_long)

        # Cache the data stream for faster runtime
        with open(f'../stream_data/{filename}', 'wb') as file:
            pickle.dump(stream, file)
    else:
        with open(f'../stream_data/{filename}', 'rb') as file:
            stream = pickle.load(file)

    return stream
