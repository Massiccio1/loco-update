
#ifndef ROBOT__H
#define ROBOT__H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
//#include <std_msgs/msg/float64_multi_array.hpp>
#include "Eigen/Eigen/Dense"
#include "my_vision_messages/Pointxyz.h"
#include "my_vision_messages/Block.h"
#include "my_vision_messages/BlockList.h"

#include <sstream>
#include <iostream>
#include <string>
#include <thread>
//#include "Helper.cpp"
#include "Helper.h"
//#include "Kin.cpp"
#include "Kin.h"


#define N_GRIP 2
#pragma once

using namespace std;


void robot_msg_callback(const std_msgs::String::ConstPtr& message);


void js_callback(const sensor_msgs::JointState& joint);

/**
@file robot.cpp
@brief robot class for ur5 manipulator
*/

/*! @brief robot class for ur5 manipulator
*
*/



class Robot{
    public: 

        static inline sensor_msgs::JointState joint;
        static inline std::vector<double> gripper_j;
        static inline std_msgs::String msg;
        static inline int stat=0;
        static inline bool started = false;
        static inline bool moving = false;
        //ros::init(argc, argv,"talker");
        static inline ros::Publisher js_pub;
        static inline int n_grip = 0;
        ros::Publisher grip_pub;
    
        Robot();
        Robot(bool init,int argc, char ** argv);
/*! @brief base constructor
*needs argc and argv to innitialize the joint publisher
*/
        Robot(int argc, char ** argv);

        void spin();
/*! @brief subscribes and starts callbacks
* subscribes to /ur5/joint_states and starts the callbash
*/
        void start_callback(int argc, char ** argv);
/*! @brief publishes a joint configuration
*
* @param th joitn configuration to publish
* @param fill for previous version debugging
* @param gripper for previous version debugging
*/

        int publish(Eigen::Vector < double, 6 > th, bool fill = true, double gripper = 0);
/*! @brief publishes grip joints in simulation \n
* and publishes on /gripper the opening value
* @param gripper gripper joint value/opening size
*/
        int publish_grip(double gripper = 0);

/*! @brief rotates the end effector
*
* @param ang in radiants
*/
        int rotate(double ang = 0);
/*! @brief prints a float64 array
*
* @param f64j ros float 64 array
*/
        void print_f64j(std_msgs::Float64MultiArray f64j);
        /*
        int publish(sensor_msgs::JointState& joint_2, bool fill = true, double gripper = 0){

            std_msgs::Float64MultiArray f64j;
            Eigen::Vector < double, 6 > th;

            //if(fill){
            if(false){
                //std::cout << "filling with 0" << std::endl;
                joint_2.velocity={0.0,0.0,0.0,0.0,0.0,0.0};
                joint_2.effort={0.0,0.0,0.0,0.0,0.0,0.0};
            }

            for(int i=0; i<6;i++){
                th(i) = joint.position[i];
            }

            //std::cout << "argc: " << argc << "\nargv: " << argv << std::endl;
            //std::cout << "joint to publish: " << joint << std::endl;
            //js_pub.publish(joint_2);

            publish(th,fill);

            return 0;
        }
        */
/*! @brief moves only the shoulder joint for the robot \n 
* used to reduce the chance of collision of the robot arm
* @see float move_to(Eigen::Vector < double, 6 > pr_f, int steps = 3000, float k_coeff=0.01, int verbose=false)
*/
        float move_to_shoulder(Eigen::Vector < double, 6 > pr_f, int steps = 3000, float k_coeff=0.01, int verbose=false);

/*! @brief moves the robot to a given position \n
* calculates the path from the current position and \n 
* the final one, publishes the joints for the manipulator
* 
* @see Kin::p2p()
* @see js_callback(const sensor_msgs::JointState& j)
* @param pr_f position and rotation of the final position
* @param steps number of 1ms steps
* @param k_coeff for debugging and compatibility
* @param verbose verbose option
*/
        float move_to(Eigen::Vector < double, 6 > pr_f, int steps = 3000, float k_coeff=0.01, bool verbose=false);
    
/*! @brief converts joint state message to Eigen vector of 6 joints
* 
* @param  j_tmp Joint state message from ros
* @return Eigen vector of angles for each of the 6 joints
*/
        Eigen::Vector < double, 6 > j_to_q(sensor_msgs::JointState j_tmp);
        void test();
        void set_msg(string s);
        string get_msg();
        static int print_position(sensor_msgs::JointState j);
        static int print_position();


//std_msgs::String Robot::msg = new std_msgs::String();
};



void robot_msg_callback(const std_msgs::String::ConstPtr& message);
/*! @brief callback for ur5/joint_states \n
* detects the number of joints, saves the eventual gripper values, \n
* reorders the joints values and saves them
*/

void js_callback(const sensor_msgs::JointState& j);

ostream& operator<<(ostream& os, const my_vision_messages::Pointxyz& p);

ostream& operator<<(ostream& os, const my_vision_messages::BlockList& bl);

bool operator==( const my_vision_messages::BlockList& bl1, const my_vision_messages::BlockList& bl2 );

void print_BlockList(my_vision_messages::BlockList& bl);

void swap(double& f1,double& f2);

#endif
