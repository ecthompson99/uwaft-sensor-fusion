#include "mobileye_struct.h"

void Mobileye_RX::get_nums(int id, int &case_num, int &obj_num){ 
            
    // deafault values set to -1
    obj_num = -1;

    if(id >=CAN_message.traffic1 && id <= CAN_message.traffic2){
        case_num = 1; //Traffic Sensor 
    } else if(id >= CAN_message.obstacle_A1 && id <= CAN_message.obstacle_A2 && id % 3 == 1){
        case_num = 2; //Obstacle A Frame
    } else if(id >= CAN_message.obstacle_B1 && id <= CAN_message.obstacle_B2 && id % 3 == 2){
        case_num = 3; //Obstacle B Frame
    } else if(id >= CAN_message.obstacle_C1 && id <= CAN_message.obstacle_C2 && id % 3 == 0){
        case_num = 4; //Obstacle C Frame
    } else if(id == CAN_message.left_lane_A){
        case_num = 5; //LKA Left Lane Frame A
    } else if(id == CAN_message.left_lane_B){
        case_num = 6; //LKA Left Lane Frame B 
    } else if(id == CAN_message.right_lane_A){
        case_num = 7; //LKA Right Lane Frame A 
    } else if(id == CAN_message.right_lane_B){
        case_num = 8; //LKA Right Lane Frame B 
    } else if(id == CAN_message.diagnostics){ // Diagnostics
        case_num == 9;
    }else{
        case_num = 0; // faulted 
    }
    
    if (case_num >=2 && case_num <= 4){
        /* Obs A frame: 1876, 1873, 1870, 1867, 1864, 1861, 1858, 1855, 1852, 1849	
        Obs B frame: 1877, 1874, 1871, 1868, 1865, 1862, 1859, 1856, 1853, 1850
        Obs C frame: 1878, 1875, 1872, 1869, 1866, 1863,1860, 1857, 1854, 1851	
        Lowest ID is 1849, increments by 3. Formula allows for calculating object ID from 0-9 given the case where the IDs are A,B, or C frame
        eg. if the id is 1867, (1867 - (1849 + 2-2))/3 = object #6
        */
        obj_num = (id - (CAN_message.obstacle_A1 + case_num - 2)) / 3; 
    }

}; 