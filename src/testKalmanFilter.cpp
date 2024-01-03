#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <Eigen/Dense>
#include <kalmanfilter.hpp>
#include <cstring>
#include <string>


class TestKalmanFilter 

{

    private:

        KalmanFilter* filter;
        std_msgs::Float64MultiArray filteredResultsMsg;
        Eigen::VectorXd outputVector;
        ros::Publisher pubFilteredResults;
        ros::Subscriber subLoadCell;

    public: 

        TestKalmanFilter(ros::NodeHandle *n, KalmanFilter* filter_) 
        
        {

            ros::Rate rate(1);
            pubFilteredResults = n->advertise<std_msgs::Float64MultiArray>("/loadcell_kalmanfiltered", 10);
            rate.sleep();
            filter = filter_;
            rate.sleep();
            subLoadCell = n->subscribe("/loadcell_measurements", 1, &TestKalmanFilter::loadCellCallback, this);
            ros::spin();

        }

        void loadCellCallback(const std_msgs::Float64MultiArray& msg) 
        
        {


            Eigen::VectorXd measurement;
            measurement.resize(1);
            measurement(0) = msg.data.data()[0];


            filter->computeEstimate(measurement); 
            outputVector = filter->getEstimate();
            std::vector<double> filteredResults(outputVector.data(), outputVector.data() + outputVector.size());
            filteredResultsMsg.data.clear();
            filteredResultsMsg.data.insert(filteredResultsMsg.data.end(), filteredResults.begin(), filteredResults.end());
            pubFilteredResults.publish(filteredResultsMsg);

        }

};

int main (int argc, char** argv) 

{

    ros::init(argc, argv, "tension_kalman_filter");

    KalmanFilter m(20, 30, 0.01);
    ros::NodeHandle n; 
    TestKalmanFilter(&n, &m);

    return 0; 

}


