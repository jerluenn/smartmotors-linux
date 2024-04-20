#include "kalmanfilter.hpp"

KalmanFilter::KalmanFilter()

{

    //Initialisation
    dt = 0.01; 
    a = 20; 
    r1 = 30; 
    x_minus.resize(2, 1);
    x_plus.resize(2, 1);
    Q.resize(2, 2);
    R.resize(1, 1); 
    Kk.resize(2, 1); 
    F.resize(2, 2); 
    H.resize(1, 2);
    Pk_minus.resize(2, 2);
    Pk_plus.resize(2, 2);
    I_2x2.resize(2,2); 

    I_2x2.setIdentity();
    Pk_plus.setIdentity();

    F.setIdentity(); 
    F(0, 1) = dt;

    Q(0, 0) = dt*dt*a/2;
    Q(1, 1) = dt*a;

    R(0, 0) = 20.0;

    H(0, 1) = 0.0; 
    H(0, 0) = 1.0; 
    


}

KalmanFilter::~KalmanFilter(){} 

void KalmanFilter::setParameters(double a_, double r1_, double dt_){

    a = a_; 
    r1 = r1_; 
    dt = dt_;

}

Eigen::MatrixXd KalmanFilter::testVector() 

{

    return Q;

}

double KalmanFilter::testA() 

{

    return a;

}

void KalmanFilter::computeEstimate(Eigen::VectorXd measurements) 

{

    Pk_minus = F*Pk_plus*F.transpose() + Q; 
    // Pk_minus = F*Pk_plus*F.transpose(); 
    Kk = Pk_minus*H.transpose()*(H*Pk_minus*H.transpose() + R).inverse();
    x_minus = F*x_plus; 
    x_plus = x_minus + Kk * (measurements - H*x_minus);
    Pk_plus = (I_2x2 - Kk*H)*Pk_minus;

} 

Eigen::VectorXd KalmanFilter::getEstimate() 

{

    return x_plus;

}