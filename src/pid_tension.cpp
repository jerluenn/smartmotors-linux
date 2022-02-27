#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "smartmotors_linux/command.h"
#include "smartmotors_linux/arraycommand.h"
#include <stdio.h>
#include <string.h>
#include "sensor_msgs/Joy.h"
#include <stack>
#include <chrono>
#include <cmath>
#include <Eigen/Dense>

class pid_controller {

    private: 
    double Kp, Ki, Kd;
    double Kp_rail = 250;
    double Kd_rail = 0.0; 
    double Kp_gripper = 250;
    double Kd_gripper = 0.0; 
    double dt; 
    const int sensor_count = 6;
    double gripper_gain = -5.0;
    double linear_rail_gain = 5.0;
    double max_vel = 350000;
    ros::Publisher pub_sm_array; 
    ros::Publisher pub_sm;
    ros::Publisher str_pub;
    ros::Subscriber sub;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    Eigen::VectorXd integral_error;
    Eigen::VectorXd previous_error;
    Eigen::VectorXd ref; 
    Eigen::VectorXd ref_arm; 
    Eigen::VectorXd error;
    Eigen::VectorXd control_input; 
    Eigen::VectorXd initial_input;
    smartmotors_linux::arraycommand control_msg;
    smartmotors_linux::command sm_msg;
    // std::vector<float> control_vector;   
    std::string str_command; 


    public: 
    pid_controller(ros::NodeHandle *n, double K_p, double K_i, double K_d) {

        initial_input.resize(sensor_count);
        initial_input.setOnes();
        initial_input *= 50000; 
        previous_error.resize(sensor_count); 
        previous_error.setZero();
        ref.resize(sensor_count); 
        control_input.resize(sensor_count);
        error.resize(sensor_count);
        // ref.setZero();
        ref << 50.0, 50.0, 50.0, 50.0, 50.0, 80.0;
        integral_error.resize(sensor_count); 
        integral_error.setZero();
        str_command = "VT";
        control_msg.motorcommand = str_command;
        dt = 1.0/80.0;
        Kp = K_p; 
        Ki = K_i; 
        Kd = K_d;
        pub_sm_array = n->advertise<smartmotors_linux::arraycommand>("smartmotor_array_command", 10); // for arrays.
        pub_sm = n->advertise<smartmotors_linux::command>("smartmotor_command", 10);
        ros::Rate rate(1);
        rate.sleep();
        initialise();
        sub = n->subscribe("/ref_tensions", 1, &pid_controller::ref_tensions_callback , this);
        sub2 = n->subscribe("/loadcell_measurements", 1, &pid_controller::loadcell_measurements_callback , this);
        sub3 = n->subscribe("/joy", 1, &pid_controller::joy_callback, this);

    }

    void joy_callback(const sensor_msgs::Joy &msg) {

        ref[4] -= msg.axes[7] * linear_rail_gain; 
        ref[5] += msg.axes[6] * gripper_gain; 

        if (msg.buttons[1] == 1) {

            ros::Rate looprate(10);

            str_command = "MT ";
            sm_msg.motorcommand = str_command;
            sm_msg.motorno = 0;
            pub_sm.publish(sm_msg);
            looprate.sleep();

            str_command = "T=0 ";
            sm_msg.motorcommand = str_command;
            pub_sm.publish(sm_msg);
            looprate.sleep();

            str_command = "G ";
            sm_msg.motorcommand = str_command;
            pub_sm.publish(sm_msg);
            looprate.sleep();

            str_command = "X ";
            sm_msg.motorcommand = str_command;
            pub_sm.publish(sm_msg);
            looprate.sleep();

            ros::shutdown();

        }

    }

    void ref_tensions_callback(const std_msgs::Float64MultiArray& msg) {

        ref_arm = Eigen::VectorXd::Map(msg.data.data(), msg.data.size());
        
        for (int i = 0; i < msg.data.size(); i++ ) {

            ref[i] = ref_arm[i];

        }


    } 

    void loadcell_measurements_callback(const std_msgs::Float64MultiArray& msg) {

        Eigen::VectorXd measurements = Eigen::VectorXd::Map(msg.data.data(), msg.data.size());
        error = ref - measurements;

        // for (int i = 0; i < 4; i++) {

        //     control_input[i] = - 1000 * pow(abs(error[i]), 0.6) * tanh(error[i]);

        // }
        control_input.head(4) = - (Kp * error.head(4) + Kd * (error.head(4) - previous_error.head(4))/dt);  // This is for P PID controller! 
        control_input[4] = - (Kp_rail * error[4] - Kd_rail * (error[4] - previous_error[4])/dt);
        control_input[5] = - (Kp_gripper * error[5] + Kd_gripper * (error[5] - previous_error[5])/dt);
        apply_upper_limit(control_input);

        std::cout << "de: " << (error - previous_error)/dt << std::endl;
        std::cout << "Ref: " << ref << std::endl;
        std::cout << "Control input: " << control_input.array() << std::endl;
        std::vector<int> control_vector(control_input.data(), control_input.data() + control_input.size());
        control_msg.motorarrayvalues.clear();
        control_msg.motorarrayvalues.insert(control_msg.motorarrayvalues.end(), control_vector.begin(), control_vector.end());
 
        pub_sm_array.publish(control_msg);

        previous_error = error;


    }

    void apply_upper_limit(Eigen::VectorXd& con){

        for (int i = 0; i < con.size(); i++) {

            if (abs(con[i]) > max_vel) {

                con[i] = con[i]/abs(con[i])*max_vel;

            }

        }

    }

    void initialise() {

        str_command = "ADT=1000 "; 
        sm_msg.motorcommand = str_command;
        pub_sm.publish(sm_msg);

        str_command = "MV ";
        sm_msg.motorcommand = str_command;
        pub_sm.publish(sm_msg);
        ros::Rate looprate(10);

        for (int i = 0; i < sensor_count; i ++) {

            sm_msg.motorno = i + 1; 
            pub_sm.publish(sm_msg);
            looprate.sleep();
        
        }

        // str_command = "MP ";
        // sm_msg.motorcommand = str_command;
        // pub_sm.publish(sm_msg);
        // ros::Rate looprate(10);

        // sm_msg.motorno = 5;
        // pub_sm.publish(sm_msg);
        // sm_msg.motorno = 6;
        // pub_sm.publish(sm_msg);


        auto t_start = std::chrono::high_resolution_clock::now();
        float loosen_time = 5.0;

        while (true) {

            std::vector<int> control_vector(initial_input.data(), initial_input.data() + initial_input.size());
            control_msg.motorarrayvalues.clear();
            control_msg.motorarrayvalues.insert(control_msg.motorarrayvalues.end(), control_vector.begin(), control_vector.end());
            pub_sm_array.publish(control_msg);

            auto t_end = std::chrono::high_resolution_clock::now();
            double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();

            if (elapsed_time_ms/1000 > loosen_time){
                break;
            }

        }

        for (int i = 0; i < 3; ++i) {   
            
            looprate.sleep();
            str_command = "X ";
            sm_msg.motorcommand = str_command;
            sm_msg.motorno = 0; 
            pub_sm.publish(sm_msg);

        }



    } 

};


int main(int argc, char** argv){

    ros::init(argc, argv, "sm_pid_tension");
    ros::NodeHandle n;
    pid_controller controller = pid_controller(&n, 600, 0.0, -0.0); 
    ros::spin();

} 