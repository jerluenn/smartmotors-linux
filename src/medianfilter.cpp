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

            std::sort(window_data[i].begin(), window_data[i].end());
            median = window_data[i][window_data[i].size() / 2];
            outputVector(i) = median;

        }


} 


Eigen::VectorXd MedianFilter::getMedian() 

{

    return outputVector;

}