#include <cmath>
#include <iterator>
#include <iostream>
#include <algorithm>
#include <vector>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"

class MedianFilter 

{

    public:

        MedianFilter(int window_size_, int num_meas_);
        MedianFilter();
        ~MedianFilter();
        void setData(double data, unsigned int index);
        void computeMedian();
        Eigen::VectorXd getMedian();


    private: 

        int window_size, num_meas;
        std::vector<std::vector<double>> window_data;
        std::vector<double> window_data_index;
        Eigen::VectorXd outputVector;
        void saveData();

};