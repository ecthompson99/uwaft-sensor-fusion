#include <canlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "emc_fusion_object.h"
#include "common/tracked_output_msg.h"

#define TOPIC_SF "tracked_obj"

/** Class for variables/members pertaining to sensor fusion object output. */
class Sensor_Fusion_TX{
  public:
    ros::NodeHandle* node_handle;
    ros::Subscriber sensor_fusion_sub; 

    Sensor_Fusion_TX(ros::NodeHandle* node_handle);

    /** Tracked object parameter struct. */
    struct tracked_obj_out{
      double obj_id[3];
      double obj_dx[3]; 
      int obj_lane[3];
      double obj_vx[3];
      double obj_dy[3];
      double obj_ax[3]; 
      bool obj_path[3];
      double obj_vy[3]; 
      int obj_rc[3];
      double obj_timestamp[3]; 
      int obj_track_num[3];
    };
    Sensor_Fusion_TX::tracked_obj_out fusion_out; 
    void fusion_callback(const common::tracked_output_msg& recvd_data); /*!< Callback called when environment state publishes a tracked object.  */

};


