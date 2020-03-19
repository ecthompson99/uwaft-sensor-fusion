#include "data_association.h"
#include <ctime>
#include <cstdlib>

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_fusion");
    ros::NodeHandle data_association_handle;
    DataAssociation data_assc = DataAssociation(&data_association_handle);

    std::srand(std::time(nullptr));

    while (ros::ok()){ 

        // mock mobileye pub
        // sensor_fusion::mobileye_object_data me_data;

        // me_data.me_dx = (rand()%100) + 1;
        // me_data.me_dy = (rand()%100) + 1;
        // me_data.me_vx = (rand()%10) + 1;
        // me_data.me_timestamp = std::time(nullptr);

        // data_assc.mock_me_pub.publish(me_data);

        
        // mock radar pub
        sensor_fusion::radar_object_data radar_data;

        radar_data.radar_dx = (rand()%100) + 1;
        radar_data.radar_dy = (rand()%100) + 1;
        radar_data.radar_vx = (rand()%10) + 1;
        radar_data.radar_vy = (rand()%10) + 1;
        radar_data.radar_timestamp = std::time(nullptr);

        data_assc.mock_radar_pub.publish(radar_data);


        ros::spinOnce();
    }

    return 0;
}
