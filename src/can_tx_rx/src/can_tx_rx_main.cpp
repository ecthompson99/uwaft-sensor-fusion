#include "can_tx_rx/can_tx_rx.h"
#include <canlib.h>
#include "ros/ros.h"

void get_nums(int id, uint8_t &case_n, uint8_t &radar_n, uint8_t &frame_n, uint8_t &obj_n, uint8_t &target_obj_n) {
  if (id == 1985 || id == 1958 || id == 1879 || id == 1957) {
    case_n = 1;
  } else if (id > 1604 && id < 1659) {
    case_n = 2;
  } else if (id == 1665 || id == 1667 || id == 1280 || id == 1282 || id == 1670 || id == 1672) {
    case_n = 3;
  } else if (id > 1284 && id < 1599) {
    case_n = 4;
  } else if (id == 512 || id == 513 || id == 514 || id == 1168 || id == 1170) {
    case_n = 5;
  } else {
    case_n = 0;
  }

  switch (case_n) {
    case 1:
      if (id == 1985 || id == 1879) {
        radar_n = 1;
      } else {
        radar_n = 2;
      }
      break;

    case 2:
      if (id % 10 == 5 || id % 10 == 6) {
        radar_n = 1;
      } else {
        radar_n = 2;
      }

      if (id % 10 == 5 || id % 10 == 7) {
        frame_n = 1;
      } else {
        frame_n = 2;
      }

      target_obj_n = (id - 1600 - (id % 10)) / 10;

      break;

    case 3:
      if (id == 1665 || id == 1280 || id == 1670) {
        radar_n = 1;
      } else {
        radar_n = 2;
      }
      break;

    case 4:
      if (id % 10 == 5 || id % 10 == 6) {
        radar_n = 1;
      } else {
        radar_n = 2;
      }

      if (id % 10 == 5 || id % 10 == 7) {
        frame_n = 1;
      } else {
        frame_n = 2;
      }

      obj_n = (id - 1280 - (id % 10)) / 10;

      break;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "can_tx_rx");
  ros::NodeHandle can_tx_rx_feeder_handle;
  Can_Tx_Rx_Class Can_Tx_Rx_Class(&can_tx_rx_feeder_handle);
  canHandle hnd_0;
  canHandle hnd_1;
  canHandle hnd_2;
  canHandle hnd_3;

  canInitializeLibrary();

  hnd_0 = canOpenChannel(0, canOPEN_EXCLUSIVE);
  hnd_1 = canOpenChannel(1, canOPEN_EXCLUSIVE);
  hnd_2 = canOpenChannel(2, canOPEN_EXCLUSIVE);
  hnd_3 = canOpenChannel(3, canOPEN_EXCLUSIVE);

  if (hnd_0 < 0) {
    char msg[64];
    canGetErrorText((canStatus)hnd_0, msg, sizeof(msg));
    fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
    exit(1);
  }

  if (hnd_1 < 0) {
    char msg[64];
    canGetErrorText((canStatus)hnd_1, msg, sizeof(msg));
    fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
    exit(1);
  }

  if (hnd_2 < 0) {
    char msg[64];
    canGetErrorText((canStatus)hnd_2, msg, sizeof(msg));
    fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
    exit(1);
  }

  if (hnd_3 < 0) {
    char msg[64];
    canGetErrorText((canStatus)hnd_3, msg, sizeof(msg));
    fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
    exit(1);
  }


  canSetBusParams(hnd_0, canBITRATE_250K, 0, 0, 0, 0, 0);
  canSetBusParams(hnd_1, canBITRATE_250K, 0, 0, 0, 0, 0);
  canSetBusParams(hnd_2, canBITRATE_250K, 0, 0, 0, 0, 0);
  canSetBusParams(hnd_3, canBITRATE_250K, 0, 0, 0, 0, 0);

  canSetBusOutputControl(hnd_0, canDRIVER_NORMAL);
  canSetBusOutputControl(hnd_1, canDRIVER_NORMAL);
  canSetBusOutputControl(hnd_2, canDRIVER_NORMAL);
  canSetBusOutputControl(hnd_3, canDRIVER_NORMAL);

  canBusOn(hnd_0);
  canBusOn(hnd_1);
  canBusOn(hnd_2);
  canBusOn(hnd_3);

  uint8_t id;
  uint8_t can_data[8] = {0};
  unsigned int dlc;
  unsigned int flag;
  unsigned long time;
  uint8_t case_num = 0;
  uint8_t radar_num = 0;           // 1 or 2 == valid
  uint8_t frame_num = 0;           // 1 = Frame_A, 2 = Frame_B, 3 = general, other = error
  uint8_t obj_num = -1;            // 0 to 31 = valid
  uint8_t target_object_num = -1;  // 0 to 5 = valid
  while (ros::ok()) {
        /*  canStatus stat = canRead(hnd_0, &id, &can_data, &dlc, &flag, &time);

      if (canOK == stat) {
        // MABx and Jetson
      }

      stat = canRead(hnd_1, &id, &can_data, &dlc, &flag, &time);
    */
    // if (canOK == stat) {
    // Front radar = radar_1 and rear radar = radar_2
    get_nums(id, case_num, radar_num, frame_num, obj_num, target_object_num);
    switch (id) {
      case 1:
        break;
      case 2:
        break;
      case 3:
        break;
      case 4:
        break;
      case 5:
        break;
      case 0:
        break;
        //}
    }
    /*
          stat = canRead(hnd_2, &id, &can_data, &dlc, &flag, &time);

          if (canOK == stat) {
            // left corner radar = radar_1 and right corner radar = radar_2
          }

          stat = canRead(hnd_3, &id, &can_data, &dlc, &flag, &time);

          if (canOK == stat) {
            // mobileye
          }
        */
    // sensor_diagnostic_pub.publish(diag_msg);
    // raw_sensor_data_pub.publish(raw_object_msg);
    // drive_control_input_pub.publish(drive_ctrl_msg);
    // ROS_INFO_STREAM("\n" << data << "\n");

      canBusOff(hnd_0);
      canBusOff(hnd_1);
      canBusOff(hnd_2);
      canBusOff(hnd_3);

      canClose(hnd_0);
      canClose(hnd_1);
      canClose(hnd_2);
      canClose(hnd_3);

    Can_Tx_Rx_Class.publish_drive_ctrl_input();
    Can_Tx_Rx_Class.publish_raw_sensor_object();
    Can_Tx_Rx_Class.publish_sensor_diag();
    ros::spinOnce();
  }
  return 0;
}