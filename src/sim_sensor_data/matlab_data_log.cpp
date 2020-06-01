#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "mat.h"
#include <iostream>
#include <vector>

void read_data_from_mat(const char *file, std::vector<double>& data)
{
    MATFile *pmat = matOpen(file, "r");
    if (pmat == NULL) return;

    mxArray *arr = matGetVariable(pmat, "Time");
    if (arr != NULL && mxIsDouble(arr) && !mxIsEmpty(arr))
    {
        mwSize size = mxGetNumberOfElements(arr); 
        double *time = mxGetDoubles(arr);
        if (time != NULL)
        {
            v.reserve(size);
            v(time, time + size);
        }
    }
    mxDestroyArray(arr);
    matClose(pmat);
}

int main() //int argc, char **argv
{
    // ros::init(argc, argv, "logger");
    // ros::NodeHandle nh;
    // rosbag::Bag bag;
    // bag.open("sensor_data_from_matlab.bag", rosbag::bagmode::Write);
 
    // std_msgs::String str;
    // str.data = std::string("foo");
    
    // std_msgs::Int32 i;
    // i.data = 42;
    
    // bag.write("chatter", ros::Time::now(), str);
    // bag.write("numbers", ros::Time::now(), i);
    
    // bag.close();
    std::vector<double> v;
    read_data_from_mat("highway_sensor_data.mat", &v);
    for (size_t i=0; i<v.size(); ++i)
    {
        std::cout << v[i] << std::endl;
    }
    return 0;

}

