#include "medianfilter.hpp"

#define assertm(exp, msg) assert(((void)msg, exp))

MedianFilter::MedianFilter(int window_size_, int num_meas_) : window_size(window_size_), num_meas(num_meas_)

{

    assertm(window_size_%2 == 1, "Window size must be an odd number, because I am lazy.");
    outputVector.resize(num_meas_);

    for (int k = 0; k < window_size_; ++k) 
    
    {

        window_data_index.push_back(0);

    }

    for (int i = 0; i < num_meas; ++i) 
    
    {

        window_data.push_back(window_data_index);

    }


}

MedianFilter::MedianFilter() {}

MedianFilter::~MedianFilter() {}

void MedianFilter::setData(double data, unsigned int index) 

{

    window_data[index].erase(window_data[index].begin());
    window_data[index].push_back(data);

}

void MedianFilter::computeMedian() 


{

    double median;


    for (int i = 0; i < num_meas; ++i)

        {

            std::vector<double> window_data_tmp = window_data[i];
            std::sort(window_data_tmp.begin(), window_data_tmp.end());
            median = window_data_tmp[window_data_tmp.size() / 2];
            outputVector(i) = median;


        }




} 


Eigen::VectorXd MedianFilter::getMedian() 

{

    return outputVector;

}