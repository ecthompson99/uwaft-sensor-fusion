#ifndef __OJBECT_STATE_H__
#define __OBJECT_STATE_H__
#include "ros/ros.h"
class ObjectState {
    public:

    ObjectState(){}

    ObjectState(double dx, double vx, double dy, double vy, double timestamp) :
    dx(dx),
    vx(vx),
    dy(dy),
    vy(vy),
    timestamp(timestamp)



    // ADD MORE MEMBERS 
    // THIS IS THE SRV FILE :
    // uint8[] id
    // float64[] dx
    // uint8[] lane
    // float64[] vx
    // float64[] dy
    // float64[] ax
    // bool[] path
    // float64[] vy
    // float64[] timestamp
    // uint8[] count
    
    {
        count = 0;
    }

    friend class DataAssociation;

    // virtual ~ObjectState();

    uint8_t id; // object ID
    double dx; // longitudinal range
    uint8_t lane; // lane assignment
    double vx; // relaive longitudinal velocity
    double dy; // lateral range
    double ax; // relative longitudinal accel
    bool path; // 1:object in path 2:object not in path
    double vy; // lateral velocity
    double timestamp; //time last object detection
    int count;

};
#endif  // __OBJECT_STATE_H__