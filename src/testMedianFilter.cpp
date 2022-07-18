#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <Eigen/Dense>
#include <medianfilter.hpp>


class TestMedianFilter 

{

    private:

        MedianFilter* filter;
        std_msgs::Float64MultiArray filteredResultsMsg;
        Eigen::VectorXd outputVector;
        ros::Publisher pubFilteredResults;
        ros::Subscriber subLoadCell;

    public: 

        TestMedianFilter(ros::NodeHandle *n, MedianFilter* filter_) 
        
        {

            ros::Rate rate(1);
            pubFilteredResults = n->advertise<std_msgs::Float64MultiArray>("/loadcell_filtered", 10);
            rate.sleep();
            filter = filter_;
            rate.sleep();
            subLoadCell = n->subscribe("/loadcell_measurements", 1, &TestMedianFilter::loadCellCallback, this);
            ros::spin();

        }

        void loadCellCallback(const std_msgs::Float64MultiArray& msg) 
        
        {

            for (unsigned int i = 0; i < msg.data.size(); ++i) 
            
            {

                filter->setData(msg.data.data()[i], i);


            }

            filter->computeMedian(); 
            outputVector = filter->getMedian();
            std::vector<double> filteredResults(outputVector.data(), outputVector.data() + outputVector.size());
            filteredResultsMsg.data.clear();
            filteredResultsMsg.data.insert(filteredResultsMsg.data.end(), filteredResults.begin(), filteredResults.end());
            pubFilteredResults.publish(filteredResultsMsg);

        }

};

int main (int argc, char** argv) 

{

    ros::init(argc, argv, "tension_median_filter");
    MedianFilter m(5, 6);
    ros::NodeHandle n; 
    TestMedianFilter(&n, &m);

    return 0; 

}


