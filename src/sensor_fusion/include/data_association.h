#ifndef __DATA_ASSOCIATION_H__
#define __DATA_ASSOCIATION_H__

#include "ros/ros.h"
#include "object_state_da.h"

#include "sensor_fusion/env_state_srv.h"

#include "sensor_fusion/mobileye_object_data.h"
#include "sensor_fusion/radar_object_data.h"
#include "sensor_fusion/sensor_diagnostic_flag_msg.h"
#include "sensor_fusion/associated_me_msg.h"
#include "sensor_fusion/associated_radar_msg.h"

#define MOBILEYE_TOPIC "mobileye_topic"
#define RADAR_TOPIC "radar_topic"
#define KALMAN_FILTER_RADAR_TOPIC "kf_radar"
#define KALMAN_FILTER_ME_TOPIC "kf_me"
#define SENSOR_DIAG_TOPIC "sensor_diagnostic_flags"
#define TOL 5
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

		ros::Subscriber sensor_radar_data_obj_sub;
		void sensor_radar_data_obj_callback(const sensor_fusion::radar_object_data& sensor_data);

		ros::Subscriber sensor_me_data_obj_sub;
		void sensor_me_data_obj_callback(const sensor_fusion::mobileye_object_data& sensor_data);

		ros::Subscriber sensor_diag_sub;
		void sensor_diagnostics_callback(const sensor_fusion::sensor_diagnostic_flag_msg& sensor_diag);

		std::vector<ObjectState> potential_objs;

		bool objects_match(ObjectState obj, double sensor_dx, double sensor_dy);

		unsigned long long next_id;

};

#endif  // __DATA_ASSOCIATION_H__
