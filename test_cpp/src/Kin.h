#ifndef KIN__H
#define KIN__H


#include "Eigen/Eigen/Dense"
#include <iostream>
#include <vector>
#include <cmath>
//#include "Helper.cpp"
#include "Helper.h"
#include <fstream>

//using namespace pinocchio;
using namespace std;


#ifndef MODEL_PATH
  #define MODEL_PATH "~/ros_ws/src/robo/ros/robot_urdf/generated_urdf/ur5.urdf"
#endif

/**
@file kin.cpp
@brief kinematics class for the ur5 robot
*/

/*! @brief kinematics class for the ur5 robot
*
*
*/

class Kin {
   

    public:

        Eigen::Vector < double, 7 > arm_l; //lunghezza bracci
        //Eigen::Vector<double, 6> joint_q; //rotazione joints
        Eigen::Vector < double, 6 > Th;
        Eigen::Vector < double, 6 > A;
        Eigen::Vector < double, 6 > D;
        Eigen::Vector <double , 6> alfa;
        
        Eigen::Matrix4d T10m;
        Eigen::Matrix4d T21m;
        Eigen::Matrix4d T32m;
        Eigen::Matrix4d T43m;
        Eigen::Matrix4d T54m;
        Eigen::Matrix4d T65m;
        Eigen::Matrix4d T0e;
        Eigen::Matrix4d T0;
        //Eigen::Matrix4d T10m;
        Eigen::Matrix4d T20m;
        Eigen::Matrix4d T30m;
        Eigen::Matrix4d T40m;
        Eigen::Matrix4d T50m;
        Eigen::Matrix4d T60m;
        Eigen::MatrixXd J;
        static inline int ik_index=7;


        Kin();
    
/*! @brief returns the rotation matrix of the end effector relativo to the base frame
*
* @return Eigen::Matrix4d T0e end effector rotation matrix
*/
    Eigen::Matrix4d get_T0e();
    Eigen::Vector3d get_rot_p();
/*! @brief returns current position and euler rotation for the loaded \n
*
* must be used after @ref compute_fc(Eigen::Vector < double, 6 > , bool)
*
* @return Eigen::Vector < double, 6 > pr
*/

    Eigen::Vector < double, 6 > get_pr_now();
/*! @brief returns the position of the end effector
*
* @return Eigen::Vector3d position in x,y,z
*/

    Eigen::Vector3d get_ee_p();
/*! @brief computes forward kinematic and loads the inner varables of the class for future uses
*
*
* @param Eigen::Vector < double, 6 > the joint configuration of the robot 
* @param print verbose mode, default = flase
* @return 0 if everything worked, does not return a position, that must be accessed with get_pr_now()
*/

    int compute_fc(Eigen::Vector < double, 6 > Th, bool print_ = false);
/*! @brief the Jacobian of a given joint configuration
*
*
* @param Eigen::Vector < double, 6 > q the joint configuration
* @param print verbose mode, default = flase
* @return Eigen::MatrixXd J full jacobian matrix
*/


    Eigen::MatrixXd compute_J(Eigen::Vector < double, 6 > q, bool print_ = false);
/*! @brief evaluates the pose of a given joint configuration \n 
*
* the pose is used as an index for compute_ik(Eigen::Vector < double, 6 > ) \n 
* use this function to obtain consistent pose for different joint configuration
* 
* @param Eigen::Vector < double, 6 > the joint configuration
* @return index of the inverse kinematics configuration
*/

    int eval_ik_index(Eigen::Vector < double, 6 > j_now);

    std::vector<Eigen::Vector < double, 6 >> da_a(std::vector<Eigen::Vector < double, 6 >> path_pr,Eigen::Vector < double, 6 > th0, Eigen::MatrixXd k, int steps);
    Eigen::Vector < double, 6 > invDiffKinematiControlComplete2(Eigen::Vector < double, 6 > q_temp,  Eigen::Vector < double, 6 > pr_now, Eigen::Vector < double, 6 > delta, Eigen::Vector < double, 6 > path_pr_i,    Eigen::MatrixXd k);
/*! @brief creates a joint path between 2 points in space \n
*
* uses path and trajectory cubical interpolation in the joint space \n 
* extract the joint position with compute_ik(Eigen::Vector < double, 6 > ) \n 
* and interpolates with a cubical, and calculates the joint position of each step \n 
* coses the shortest path on rotations and constrains the final angle with \n 
* constrainAngle180(Eigen::Vector<double, 6>). \n 
* returns a standard vector of joint positions \n \n 
* @see Helper::constrainAngle180(Eigen::Vector<double, 6>) \n 
* @fn p2p()
* @param pr_i initial position and rotation of the end effector
* @param pr_f final position and rotation of the end effector
* @param steps number of 1ms steps
* @return standard vector of joint positions
*/

    std::vector<Eigen::Vector < double, 6 >> p2p(Eigen::Vector < double, 6 > pr_i, Eigen::Vector < double, 6 > pr_f, int steps = 3000, double minT = 0);
/*! @brief computes inverse kinematics
*
* @see safe_acos() \n 
*
* @param pr_f position and rotation of the end effector
* @return standard vector of joints positions
*/

    vector<Eigen::Vector < double, 6 >> compute_ik(Eigen::Vector < double, 6 > pr_f);
    Eigen::Vector3d get_col(Eigen::Matrix4d M, int i);

    double hypot(double a, double b);
    
/*! @brief extracts euler angles from position and rotation vector
*
* @see pr_to_p() \n 
*
* @param pr position and rotation of the end effector
* @return vector of euler angles
*/
    Eigen::Vector3d pr_to_r(Eigen::Vector < double, 6 > pr);
/*! @brief extracts x,y,z coordinates from position and rotation vector
*
* @see pr_to_r() \n 
*
* @param pr position and rotation of the end effector
* @return vector of x,y,z coordinates
*/
    
    Eigen::Vector3d pr_to_p(Eigen::Vector < double, 6 > pr);
/*! @brief conversion function from euler angles XYZ to rotation matrix
*
* @see eul2rotZYX() \n 
*
* @param rpy roll pitch yaw
* @return rotation matrix
*/

    Eigen::Matrix3d eul2rotXYZ(Eigen::Vector3d rpy);
/*! @brief conversion function from euler angles ZYX to rotation matrix
*
* @see eul2rotXYZ() \n 
*
* @param rpy roll pitch yaw
* @return rotation matrix
*/

    Eigen::Matrix3d eul2rotZYX(Eigen::Vector3d rpy);

    void swap(double& f1,double& f2);

    std::vector<Eigen::Vector < double, 6 >> fillpath(Eigen::Vector < double, 6 > q, Eigen::Vector < double, 6 > pr_f, int steps);
    Eigen::Vector < double, 6 > invDiffKinematiControlComplete(Eigen::Vector < double, 6 > q_temp,  Eigen::Vector3d p_now, Eigen::Vector3d rpy_now, Eigen::Vector3d p_f, Eigen::Vector3d pry_f,  Eigen::Vector3d vel,    Eigen::Vector3d rot,    Eigen::Matrix3d k,  Eigen::Matrix3d kphi);
    Eigen::Vector3d lin_vel(Eigen::Vector3d p_i, Eigen::Vector3d p_f, int steps = 5000);
    Eigen::Vector3d lin_rot(Eigen::Vector3d rpy_i, Eigen::Vector3d rpy_f, int steps = 5000);
/*! @brief conversion function from rotation matrix to euler angles in ZYX
*
*
* @param rpy rotation martix
* @return euler angles
*/
    Eigen::Vector3d rotm2eul(Eigen::Matrix4d R);
    int test_tot2eul();
    int isnan (double f);
    double r_nan(double& d);
/*! @brief safe real acos for c++ \n
*
* returns real value for acos, even with inputs outside of [-1,1]
*
* @see safe_asin()
*
* @param value cos value
* @return real value for acos, even with inputs outside of [-1,1]
*/

    double safe_acos(const double& value);
/*! @brief safe real asin for c++
*
* returns real value for asin, even with inputs outside of [-1,1]
*
* @see safe_acos()
*
* @param value cos value
* @return real value for asin, even with inputs outside of [-1,1]
*/
    double safe_asin(const double& value);
    Eigen::MatrixXd geo2anJ(Eigen::MatrixXd J, Eigen::MatrixXd T0e_);
    void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove);
    int wrap(Eigen::Vector3d& v);

    private:
        Eigen::Matrix4d T10f(double th1);
        Eigen::Matrix4d T21f(double th2);
        Eigen::Matrix4d T32f(double th3);
        Eigen::Matrix4d T43f(double th4);
        Eigen::Matrix4d T54f(double th5);
        Eigen::Matrix4d T65f(double th6);
    };


#endif