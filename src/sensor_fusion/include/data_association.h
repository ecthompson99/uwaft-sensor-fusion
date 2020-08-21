#ifndef __DATA_ASSOCIATION_H__
#define __DATA_ASSOCIATION_H__

#include "ros/ros.h"
#include "object_state_da.h"

#include "sensor_fusion/env_state_srv.h"

#include "common/mobileye_object_data.h"
#include "common/radar_object_data.h"
#include "common/sensor_diagnostic_flag_msg.h"
#include "common/associated_me_msg.h"
#include "common/associated_radar_msg.h"

#define MOBILEYE_TOPIC "MOBILEYE_TOPIC"
#define FRONT_RADAR_TOPIC "FRONT_RADAR_TOPIC"
#define LEFT_CORNER_RADAR_TOPIC "LEFT_CORNER_RADAR_TOPIC"
#define RIGHT_CORNER_RADAR_TOPIC "RIGHT_CORNER_RADAR_TOPIC"
#define KALMAN_FILTER_RADAR_TOPIC "associated_radar"
#define KALMAN_FILTER_ME_TOPIC "associated_me"
#define SENSOR_DIAG_TOPIC "sensor_diagnostic_flags"
#define DX_TOL 5
#define DY_TOL 1.5
#define DX_RANGE 100
#define DY_RANGE 10
#define POTENTIAL_THRESHOLD 5
#define secondsToDelete 5
#define MESSAGE_BUFFER_SIZE 10

class DataAssociation {
	public:
		DataAssociation(ros::NodeHandle* node_handle);
		void delete_potential_objects();

		// ros::Publisher mock_me_pub;

		// ros::Publisher mock_radar_pub;

		friend class ObjectState;

	private:
		ros::NodeHandle* node_handle;

		ros::ServiceClient client;

		ros::Publisher radar_to_kf_pub;
		ros::Publisher me_to_kf_pub;

		ros::Subscriber sensor_front_radar_data_obj_sub;
		ros::Subscriber sensor_left_corner_radar_sub;
		ros::Subscriber sensor_right_corner_radar_sub;
		void sensor_radar_data_obj_callback(const common::radar_object_data& sensor_data);

		ros::Subscriber sensor_me_data_obj_sub;
		void sensor_me_data_obj_callback(const common::mobileye_object_data& sensor_data);

		ros::Subscriber sensor_diag_sub;
		void sensor_diagnostics_callback(const common::sensor_diagnostic_flag_msg& sensor_diag);

		std::vector<ObjectState> potential_objs;

		bool objects_match(ObjectState obj, double sensor_dx, double sensor_dy);

		bool radar_match(ObjectState obj, double sensor_dx, double sensor_dy);

		unsigned long long next_id;

};

#endif  // __DATA_ASSOCIATION_H__
