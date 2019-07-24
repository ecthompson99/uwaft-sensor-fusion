#ifndef __MASTER_TASK_H__
#define __MASTER_TASK_H__

#include "master_task/can_comms_data_msg.h"
#include "master_task/drive_ctrl_input_msg.h"
#include "master_task/sensor_diagnostic_flag_msg.h"
#include "master_task/sudo_driver_input_msg.h"

void drive_ctrl_msg_callback(const master_task::drive_ctrl_input_msg& drive_ctrl_msg);
void sensor_diag_flag_msg_callback(const master_task::sensor_diagnostic_flag_msg& sensor_msg);
void sudo_driver_input_msg_callback(const master_task::sudo_driver_input_msg& input_msg);

#endif  // __MASTER_TASK_H__
