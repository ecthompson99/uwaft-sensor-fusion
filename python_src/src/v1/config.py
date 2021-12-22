"""Configuration for data assignment and Objects needed for the sensor fusion pipeline.

Written by: Ethan Thompson (contact ec3thomp@uwaterloo.ca)
Date: 17 December 2021
"""

DX_TOL = 8      # Radar is less accurate in the x-direction, between 6 and 10 is generally a good estimate.
DY_TOL = 2      # Tolerance should be tight, a car is usually not much wider than 2 m
POTENTIAL_THRESHOLD = 10    # 10 consecutive readings within 1 second to categorize detection as new object.
SECONDS_TO_DELETE = 1       # Beyond 1 second and this will cause a lag in the system.
MESSAGE_BUFFER_SIZE = 10
MIN_DX = 1.0    # Real value = 10.00
MIN_DX_ME = 13.00
MAX_DX_RADAR = 30.00
MAX_DX = 130.00 # Real value = 90.00
MAX_DY = 12.00  # Real value = 2.00


class FilteredObject:
    """FilteredObject is used by Kalman Filter to characterize a detection that is being tracked."""
    def __init__(self):
        self.id = None
        self.dx = None
        self.dy = None
        self.vx = None
        self.vy = None
        self.ax = None
        self.lane = None
        self.path = None
        self.timestamp = None
        self.count = None


class MobileyeObject:
    """Characterizes a detected object by the Mobileye sensor."""
    def __init__(self, id=None, dx=None, dy=None, vx=None, ax=None, type=None, status=None, valid=None,
                 cut_in_cut_out=None, age=None, lane=None, cipv_flag=None, timestamp=None):
        self.id = id
        self.dx = dx
        self.dy = dy
        self.vx = vx
        self.ax = ax
        self.type = type
        self.status = status
        self.valid = valid
        self.cut_in_cut_out = cut_in_cut_out
        self.age = age
        self.lane = lane
        self.cipv_flag = cipv_flag
        self.timestamp = timestamp
        self.count = 0


class RadarObject:
    """Characterizes a detected object by the Radar sensor."""
    def __init__(self, id=None, dx=None, dy=None, vx=None, vy=None, ax=None,
                 dx_sigma=None, dy_sigma=None, vx_sigma=None, vy_sigma=None,
                 ax_sigma=None, w_exist=None, w_obstacle=None, flag_valid=None,
                 w_non_obstacle=None, flag_meas=None, flag_hist=None,
                 d_length=None, radar_dz=None, moving_state=None, w_class=None,
                 obj_class=None, dx_rear_loss=None, num=None, timestamp=None, lane=None):
        self.id = id
        self.dx = dx
        self.dy = dy
        self.vx = vx
        self.vy = vy
        self.ax = ax
        self.dx_sigma = dx_sigma
        self.dy_sigma = dy_sigma
        self.vx_sigma = vx_sigma
        self.vy_sigma = vy_sigma
        self.ax_sigma = ax_sigma
        self.w_exist = w_exist
        self.w_obstalce = w_obstacle
        self.flag_valid = flag_valid
        self.w_non_obstacle = w_non_obstacle
        self.flag_meas = flag_meas
        self.flag_hist = flag_hist
        self.d_length = d_length
        self.radar_dz = radar_dz
        self.moving_state = moving_state
        self.w_class = w_class
        self.obj_class = obj_class
        self.dx_rear_loss = dx_rear_loss
        self.num = num
        self.timestamp = timestamp
        self.count = 0
        self.lane = lane


class StreamObject:
    """Equivalent to a ROS message which contains the detections from the radar and Mobileye."""
    def __init__(self, radar_objects=None, me_objects=None):
        self.radar_objects = radar_objects
        self.me_objects = me_objects


class ObjectState:
    """Used by the environment state node to keep track of lane objects."""
    def __init__(self):
        self.id = None
        self.dx = None
        self.dy = None
        self.vx = None
        self.vy = None
        self.ax = None
        self.lane = None
        self.path = None
        self.timestamp = None
        self.track_num = None