#ifndef __DATA_ASSOCIATION_H__
#define __DATA_ASSOCIATION_H__

#include "ros/ros.h"
#include "object_state_da.h"
#include "radar_obj.h"
#include "mobileye_obj.h"
#include <vector>

#include "sensor_fusion/env_state_srv.h"
#include "common/sensor_diagnostic_flag_CH2.h"
#include "common/sensor_diagnostic_flag_CH3.h"
#include "common/sensor_diagnostic_flag_CH4.h"

#include "common/associated_me_msg.h"
#include "common/associated_radar_msg.h"
#include "common/mobileye_object_data.h"
#include "common/radar_object_data.h"

#define MOBILEYE_TOPIC "Mobileye_CAN_Rx"
#define RADAR_FRONT_TOPIC "Front_Radar_CAN_Rx"
#define RADAR_RIGHT_TOPIC "Right_Radar_CAN_Rx"
#define RADAR_LEFT_TOPIC "Left_Radar_CAN_Rx"
#define KALMAN_FILTER_RADAR_TOPIC "associated_radar"
#define KALMAN_FILTER_ME_TOPIC "associated_me"
#define SENSOR_DIAG_TOPIC "sensor_diagnostic_flags"
#define ENV_SERVICE "env_service_topic"
#define CH2_SERVICE "sensor_diagnostic_CH2"
#define CH3_SERVICE "sensor_diagnostic_CH3"
#define CH4_SERVICE "sensor_diagnostic_CH4"

#define DX_TOL 5.00
#define DY_TOL 1.50
#define VX_TOL 10.00
#define POTENTIAL_THRESHOLD 5
#define secondsToDelete 3
#define MESSAGE_BUFFER_SIZE 10
#define RADAR_OBJ 32
#define ME_OBJ 1  // how many objects ME is set to track
// Values below for testing purposes
#define MIN_DX 1.00   // Real value = 10.00
#define MAX_DX 20.00  // Real value = 90.00
#define MAX_DY 2.00   // Real value = 2.00
#define MAX_VX 1.00
#define EXIST 0.95


class DataAssociation {
	public:
		DataAssociation(ros::NodeHandle* node_handle);
		void delete_potential_objects();

		friend class ObjectState;

		void sensor_radar_data_obj_callback(const common::radar_object_data& sensor_data);
		void sensor_me_data_obj_callback(const common::mobileye_object_data& sensor_data);
       
        bool sensor_diagnostic_callback_CH2(common::sensor_diagnostic_flag_CH2::Request& req, common::sensor_diagnostic_flag_CH2::Response &res);
        bool sensor_diagnostic_callback_CH3(common::sensor_diagnostic_flag_CH3::Request& req, common::sensor_diagnostic_flag_CH3::Response &res);
        bool sensor_diagnostic_callback_CH4(common::sensor_diagnostic_flag_CH4::Request& req, common::sensor_diagnostic_flag_CH4::Response &res);

        std::vector<ObjectState> potential_objs;
        //std::vector<RadarObject> filtered_radar_obj;// = std::vector<RadarObject>(32);
        std::vector<MobileyeObject> filtered_me_obj;// = std::vector<MobileyeObject>(10);

        bool objects_match_radar(ObjectState obj, RadarObject& filtered_data);
        bool objects_match_me(ObjectState obj, MobileyeObject& filtered_data);

        std::vector<RadarObject> filter_radar(const common::radar_object_data& recvd_data);
        std::vector<MobileyeObject> filter_me(const common::mobileye_object_data& recvd_data);

        void pub_radar_signals(common::associated_radar_msg &matched, RadarObject &r_obj);
        void pub_me_signals(common::associated_me_msg &matched, MobileyeObject &me_obj);

        // for unit testing
        void set_front_radar(bool fr){FRONT_RADAR = fr;}
        void set_right_corner_radar(bool rc){RIGHT_CORNER_RADAR = rc;}
        void set_left_corner_radar(bool lc){LEFT_CORNER_RADAR = lc;}
        void set_me(bool me){MOBILEYE = me;}
        common::associated_radar_msg get_associated_radar_msg();
        common::associated_me_msg get_associated_me_msg();
        
        bool FRONT_RADAR;
        bool LEFT_CORNER_RADAR;
        bool RIGHT_CORNER_RADAR;
        bool MOBILEYE;

        double global_clk;
        unsigned long long next_id;


	private:
		ros::NodeHandle* node_handle;

		ros::ServiceClient client;
        ros::ServiceServer srv_ch2;
        ros::ServiceServer srv_ch3;
        ros::ServiceServer srv_ch4;


		ros::Publisher radar_to_kf_pub;
		ros::Publisher me_to_kf_pub;
        
		ros::Subscriber sensor_front_radar_data_obj_sub;
		ros::Subscriber sensor_left_corner_radar_sub;
		ros::Subscriber sensor_right_corner_radar_sub;
		ros::Subscriber sensor_me_data_obj_sub;

        common::associated_radar_msg associated_radar_msg;
        common::associated_me_msg associated_me_msg;
        
};

#endif  // __DATA_ASSOCIATION_H__