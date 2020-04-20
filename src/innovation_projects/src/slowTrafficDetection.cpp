//
//  slowTrafficDetection
//
//  Created by Geoffrey on 2019-06-30.
//

#include <iostream>
#include <chrono>
#include <thread>
#include <time.h>
#include "slowTrafficDetection.h"
using namespace std::this_thread;
using namespace std::chrono; // ns, us, ms, s, h, etc.

bool traffic_monitor(double current_speed, float front_car_distance, double top_traffic_speed, double min_duration, float min_distance){
    /*
     This functions flags the slow traffic based on two checks:
     1). the car is consistently slow(<= top_traffic_speed, i.e. 30 km/h) for a while(>= min_duration, i.e. 5 min)
     2). the distance to the car in the front is consistently small(<= min_distance, i.e. 20m)
     
     TODO:Change the parameter variable types to the appropriate ones in ROS system, and find the appropriate range for speed, distance parameters in real road tests
     */
    static unsigned int jam_duration;
    if(current_speed <= top_traffic_speed && front_car_distance <= min_distance){   //flag when speed is slow and distance to the car in front is smaller than the set parameter
        jam_duration ++;
        if(jam_duration >= min_duration){
            std::cout << "Stuck in Traffic for " << jam_duration << " mins!\n"<< std::endl;
            return true;
        }//end if (stuck)
    }else{
        jam_duration = 0;
        std::cout << "Clear Traffic" << std::endl;
    }
    return false;
}


int main(int argc, const char * argv[]) {
    // method testing codes
    const int curiseSpeed{30};
    const int triggerAfter{5};
    const float triggerWithin{5};
    float testDistance[100];
    int testSpeed[100];
    for(int i = 0; i < 100; i++){
        testDistance[i] = 4;
        testSpeed[i] = 0;
    }
    testSpeed[0] = 20;
    int incrementSign{1};
    //simulate the speed changes in actual driving
    for(int i = 1; i < 100; i++){
        //testDistance[i] = 0;
        if(testSpeed[i-1] <= 0){
            incrementSign = 1;
        }else if(testSpeed[i-1] >= 90){
            incrementSign = -1;
        }
        testSpeed[i] = testSpeed[i-1] + incrementSign* (rand()% 10 + 1);
        
        if(testSpeed[i] <= 0){
            testSpeed[i] = 0;
            incrementSign = 1;
        }else{
            if(testSpeed[i] >= 90){
                testSpeed[i]=90;
                incrementSign = -1;
            }
        }
    }

    for(int i = 0; i < 100; i++){
        std::cout << "Current Speed: " << testSpeed[i] << std::endl;
        std::cout << traffic_monitor(testSpeed[i],testDistance[i],curiseSpeed, triggerAfter, triggerWithin) << "(Traffic Jam)"<< std::endl;
        sleep_for(nanoseconds(1000000000));
        //add 1s delay to simulate 1 min in real life
    }
        //trafficMonitor(15,1,curiseSpeed, triggerAfter);
    return 0;
}
