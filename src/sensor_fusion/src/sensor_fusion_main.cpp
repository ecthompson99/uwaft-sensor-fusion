#include "data_association.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_fusion");
    ros::NodeHandle data_association_handle;
    DataAssociation data_assc = DataAssociation(&data_association_handle);

    while (ros::ok()){ 
        data_assc.delete_potential_objects();
        ros::spinOnce();
    }

    return 0;
}
