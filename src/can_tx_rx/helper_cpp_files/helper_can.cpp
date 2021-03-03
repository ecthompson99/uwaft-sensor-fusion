#include "helper_can.h"

void CAN_Helper::initialize_can(int channel_number, canHandle &hnd, canStatus &stat) {
    canInitializeLibrary();

    hnd = canOpenChannel(channel_number, canOPEN_ACCEPT_VIRTUAL);
    if (hnd < 0) {
        char msg[64];
        
        canGetErrorText((canStatus)hnd, msg, sizeof(msg));
        fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
        exit(1);
    }
    canSetBusParams(hnd, canBITRATE_250K, 0, 0, 0, 0, 0);
    canSetBusOutputControl(hnd, canDRIVER_NORMAL);
    canBusOn(hnd);
    std::cout << "Set up CAN Parameters, starting to read in loop" << std::endl;
}

double CAN_Helper::signal_in_range(double val, bool cond){
    return (cond) ? (val) : 0; 
};