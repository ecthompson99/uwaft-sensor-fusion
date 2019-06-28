#include <stdio.h>
#include <sstream>
#include "sensor_diag_dummy/SensorDiagnosticDataMsg.h"
#include "sensor_diag_dummy/SensorDiagnosticFlagMsg.h"
#include "ros.h"

void CANcallback(const sensor_diag_dummy::SensorDiagnosticDataMsg& message) {
  ROS_INFO_STREAM("\n"
            << "starterConsistency %f " << message.starterConsistency << "\n"
            << "timeStamp %f " << message.timeStamp << "\n"
            << "enderConsistency %f " << message.enderConsistency << "\n"
            << "counter %f " << message.counter << "\n"
            << "checkSum %f " << message.checkSum << "\n"
            << "horizontalMisalign %f " << message.horizontalMisalign << "\n"
            << "absorbBlind %f " << message.absorbBlind << "\n"
            << "distortBlind %f " << message.distortBlind << "\n"
            << "ITCinfo %d " << message.ITCinfo << "\n"
            << "HWfail %d " << message.HWfail << "\n"
            << "SGUFail %d " << message.SGUFail << "\n"
            << "messageCounter %d " << message.messageCounter << "\n"
            << "messageCRC %d " << message.messageCRC << "\n");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensor_diag");
  ros::NodeHandle sensor_diag_handle;

  ros::Subscriber sub = sensor_diag_handle.subscribe("CANmsg", 1000, CANcallback);

  ros:: Publisher pub = sensor_diag_handle.advertise<
                  sensor_diag_dummy::SensorDiagnosticFlagMsg>("ReliabilityMsg", 1000);


  ros::Rate rate(1);
  
    while (ros::ok()) {
    sensor_diag_dummy::SensorDiagnosticFlagMsg radarMsg;

    radarMsg.radarReliability = {0,5,15,100,200,255};
      
    pub.publish(radarMsg);
    ros::spinOnce();
  }
}
