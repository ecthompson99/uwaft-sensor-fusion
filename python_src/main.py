"""Runner script for the sensor fusion sandbox.

Written by: Ethan Thompson (contact ec3thomp@uwaterloo.ca)
Date: 17 December 2021
"""

from pathlib import Path

import pandas as pd
from tqdm import tqdm

from src.v1.data_association import DataAssociation
from src.v1.environment_state import EnvironmentState
from src.v1.kalman_filter import KFNode
from utils.plotting import plot_objects
from utils.preprocessing import label_columns, to_long_df, create_stream

if __name__ == '__main__':
    # Read in the raw sensor data
    scenario_name = 'scenario3a'  # TODO: Change this value based on the scenario you want to run
    root = r'..\sensor fusion testing\Driving Scenario Designer\csv'
    data = pd.read_csv(Path(root)/Path(scenario_name).with_suffix('.csv'))

    # =================== Pre-process data ===================
    print('\n', '='*100, '\n PRE-PROCESSING DATA \n', '='*100, '\n')

    # 1. Label data columns
    print('\n1. Labeling data')
    data = label_columns(data)

    # 2. Remove side radar data (because we aren't currently using it for sensor fusion).
    print('\n2. Removing side radar data')
    data.columns = data.columns[~data.columns.isin(['radar_2', 'radar_3'])]

    # 3. Convert dataframe from wide to long format
    print('\n3. Converting to long format')
    data_long = to_long_df(data)

    # 4. Create data stream if it has not yet been cached in local files
    print('\n4. Creating scenario data stream')
    filename = f'{scenario_name}-stream.pickle'
    stream = create_stream(filename, data_long)

    # ======== Run message stream through sensor fusion ========
    print('\n'*2, '=' * 100, '\n RUNNING SENSOR FUSION \n', '=' * 100, '\n')

    # 1. Initialize environment state node
    env_state_node = EnvironmentState()

    # 2. Initialize kalman filter node
    kf_node = KFNode(env_state_node)

    # 3. Run data stream through data association node
    for i, row in tqdm(stream.iterrows(), total=len(stream)):
        data = row.stream_object
        timestamp = row.time
        da_node = DataAssociation(data, kf_node, env_state_node, timestamp)
        da_node.sensor_me_callback()
        da_node.sensor_radar_callback()

    # ======== Plot Sensor Fusion Object Detection Results ========
    print('\n', '=' * 100, '\n PLOTTING DATA \n', '=' * 100, '\n')
    # Generate static or animated plot of all valid detections.
    # plot_objects(da_node.valid_detections, animate=False, title='Valid Detections')

    # Generate static or animated plot of only tracked objects.
    # plot_objects(da_node.tracked_objects, animate=False, title='Data Association Tracked Objects')

    # Generate static or animated plot of the environment state objects.
    env_state_node.plot_history(animate=False, title='Environment State Tracked Objects')

    print('\n', '=' * 100, '\n SENSOR FUSION COMPLETE 🥳️ \n', '=' * 100, '\n')
