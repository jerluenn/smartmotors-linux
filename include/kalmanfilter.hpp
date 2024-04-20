#include <cmath>
#include <iterator>
#include <iostream>
#include <algorithm>
#include <vector>
#include <cstring>
#include <string>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"

class KalmanFilter 

{

    public:

        KalmanFilter();
        ~KalmanFilter();
        void setParameters(double a_, double r1_, double dt_);
        Eigen::MatrixXd testVector();
        double testA();
        void computeEstimate(Eigen::VectorXd measurements);
        Eigen::VectorXd getEstimate();


    private: 

        double a, r1, dt;
        Eigen::MatrixXd F, H, Pk_minus, Pk_plus, I_2x2, x_minus, x_plus, Q, R, Kk;;

};