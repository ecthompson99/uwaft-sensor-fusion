# UWAFT Sensor Fusion Python Sandbox
The code here should be used to debug and visualize sensor fusion output for rapid prototyping.
Once code has been tested and validated in this sandbox, it should be translated back into ROS/C++ code
where it can be implemented into the current sensor fusion pipeline.

## Installation
1. Install [Python 3.8+ (recommended)](https://www.python.org/downloads/) for your system
2. Create a Python virtual environment by running
    ```terminal
    python -m venv .venv
    ```
3. Activate the virtual environment
    ```terminal
   # Windows users
    .\.venv\Scripts\activate
   # Mac users
    source bin/activate
    ```
4. Install required packages 
    ```terminal
    pip install -r requirements.txt
    ```
5. Run the main script, or use your IDE run command.
    ```terminal
    python main.py
    ```
## Versioning
We are keeping versions of this project to make it easy to rollback to previous
iterations of the sensor fusion pipeline. To change the version simply change the import
statements in `main.py` at the top. For instance, to use v1 modify the import statements
to look like this:
```python
from src.v1.data_association import DataAssociation
from src.v1.environment_state import EnvironmentState
from src.v1.kalman_filter import KFNode
```

### Version 0
This version only supports single-vehicle detection. Multi-vehicle detection
will only produce noisy and inaccurate results.

### Version 1
This version supports multi-vehicle detection on a straight road.

## Contributing
To contribute, please the clone the kaiROS repo and create your own branch based on the [UWAFT
github guidelines](https://uwaterloo.atlassian.net/wiki/spaces/UWAFT/pages/26939916568/Jira+and+Github+PR+Process+CAVs). If you are making significant structural changes to the code, please
consider creating a new version under the `/src` folder.

## Authors
* Ethan Thompson (contact: ec3thomp@uwaterloo.ca)

## Project Status
This project is ongoing. Recommendations for future steps are:
1. Testing multi-vehicle detection on a curved road
2. Adding track deletion functionality if a new detection is not made
within ~2 seconds. The Kalman filter will produce bad results if it does
not get frequent updates from data association (at least one every 0.2 seconds)