
#ifndef HELPER__H
#define HELPER__H

#include <unistd.h> 
#include <stdio.h> 
#include <fcntl.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include "Eigen/Eigen/Dense"
#include <cmath>


#define BL_DIFF_RANGE 0.03
#define BL_DIFF_CONFIDANCE 0.05

//#include "robot.cpp"



using namespace std;

/**
@file helper.cpp
@brief helper function for different applications
*/

/*! @brief helper function for different applications
*
*/

class Helper{
    public:
        Helper();

        float** sin_square(float start[], float end[], int steps);

        int fill_pr_i_f(Eigen::Vector < double, 6 > & pr_i, Eigen::Vector < double, 6 > & pr_f);

    /*! @brief debugging method to manually input positions for the robot \n 
*
* reads from src/locosim/test_cpp/src/pr_next.txt \n
* only reads the first 6 lines
*
* @param pr_f reference to the vector to store what's read
*/

        int fill_pr_next(Eigen::Vector < double, 6 > & pr_f);

        void print_pr(Eigen::Vector < double, 6 > & pr_i);

/*! @brief calculates the distance between 2 positions in space \n 
*
* takes eigen vectors of 6
*
* @see dist(Eigen::Vector3d pr_i,Eigen::Vector3d pr_f)
*
* @return float distance in absolute value 
*/

    float dist(Eigen::Vector < double, 6 >pr_i,Eigen::Vector < double, 6 > pr_f);
/*! @brief calculates the distance between 2 positions in space \n
*
* takes eigen vectors of 3
*
* @see dist(Eigen::Vector < double, 6 >pr_i,Eigen::Vector < double, 6 > pr_f)
* @return float distance in absolute value
*/
    float dist(Eigen::Vector3d pr_i,Eigen::Vector3d pr_f);

    float abs(Eigen::Vector < double, 6 >pr_i);

    
    Eigen::Vector3d cam_to_world(Eigen::Vector3d camera);
/*! @brief converts world frame fo robot frame coordinates
*
* @param camera x,y,z coordinates of the world frame
* @return float distance in absolute value 
*/
    Eigen::Vector3d tavolo_to_robo(Eigen::Vector3d world);

    
    double constrainAngle(double x);

/*! @brief wrapps differences in angles
* @fn dist_constrain(double x)
* @see dist_constrain(Eigen::Vector<double, 6> q)
* @param x angle
* @return wrapped angle difference
*/
    double dist_constrain(double x);

/*! @brief constrains angle to [-180,180]
* @fn constrainAngle180(double x)
*   @see constrainAngle180(Eigen::Vector<double, 6> q)
* @param x angle
*/
    double constrainAngle180(double x);

/*! @brief constrains angle to [-360,360]
* @fn constrainAngle720(double x)
*   @see constrainAngle720(Eigen::Vector<double, 6> q)
* @param x angle
*/
    double constrainAngle720(double x);

    Eigen::Vector<double, 6> constrainAngle(Eigen::Vector<double, 6> q);

/*! @brief constrains angle vector to [-180,180]
* @fn constrainAngle180(Eigen::Vector<double, 6> q)
*   @see constrainAngle180(double x)
* @param q vector of 6 angles
*/
    Eigen::Vector<double, 6> constrainAngle180(Eigen::Vector<double, 6> q);
/*! @brief constrains angle vector to [-360,360]
* @fn constrainAngle720(Eigen::Vector<double, 6> q)
*   @see constrainAngle720(double x)
* @param q vector of 6 angles
*/


    Eigen::Vector<double, 6> constrainAngle720(Eigen::Vector<double, 6> q);
/*! @brief wrapps differences in angles
* @fn dist_constrain(Eigen::Vector<double, 6> q)
* @see dist_constrain(double x)
* @param q vector of 6 angles 
* @return wrapped angle difference vector
*/

    Eigen::Vector<double, 6> dist_constrain(Eigen::Vector<double, 6> q);

    Eigen::Vector<double, 6> decode_final_pos(string s);

    
    Eigen::Vector3d get_extra_correction(string s);

    float get_extra_h(string s);


};



#endif
