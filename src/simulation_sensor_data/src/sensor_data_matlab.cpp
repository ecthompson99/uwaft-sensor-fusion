#include "mat.h"
#include <iostream>
#include <vector>
#include "ros/ros.h"


void matread(const char *file, std::vector<double>& v)
{
    // open MAT-file
    MATFile *pmat = matOpen(file, "r");
    ROS_INFO("Hello %s", "World");
    if (pmat == NULL)
    {
        std::cout << "Error opening file." << std::endl;
        return;
    }
    else
    {
        std::cout << "Successfully opened file!" << std:: endl;
    }

    // int ndir;
    // const char **dir;
    // dir = (const char **)matGetDir(pmat, &ndir);
    // if (dir == NULL) std::cout << "Error reading directory of file %s" << std::endl;
    // std::cout << ndir << std::endl;

    // extract the specified variable
    
    mxArray *arr;
    arr = matGetVariable(pmat, "sensor_data");

    std::cout << "number of cells is " << mxGetNumberOfElements(arr) << std::endl;
    
    if (arr == NULL) std::cout << "arr is NULL" << std::endl;
    if (arr != NULL && !mxIsEmpty(arr)) {
        // copy data
        std::cout << "arr is not NULL" << std::endl;
        mwSize mat_size = mxGetNumberOfElements(arr);
        double *time = mxGetDoubles(arr);
        if (time != NULL) {
            v.reserve(mat_size); //is faster than resize :-)
            v.assign(time, time+mat_size);
        }
    }
    // cleanup
    mxDestroyArray(arr);
    matClose(pmat);
}

int main()
{
    std::vector<double> v;
    matread("/home/joannadiao/kaiROS/src/simulation_sensor_data/highway_sensor_data_table.mat", v);
    for (size_t i=0; i<v.size(); ++i)
        std::cout << v[i] << std::endl;
        std::cout << sizeof(v) << std::endl;
    return 0;
}