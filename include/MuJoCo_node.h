#pragma once

// General Includes
#include <cmath>
#include <string>
#include <iostream>
#include <thread>
#include <vector>

// ROS includes
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float64MultiArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <franka_msgs/FrankaState.h>
// Controller includes work straight away when including ROS, nothing needed extra in the cmake
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/LoadController.h"

// MuJoCo Simulator
//#include "mujoco.h"
//#include <GLFW/glfw3.h>

#include <Eigen/Dense>

#include <boost/bind.hpp>
#include <boost/function.hpp>


using namespace Eigen;

typedef Eigen::Matrix<double, 3, 1> m_point;
typedef Eigen::Matrix<double, 4, 1> m_quat;
typedef Eigen::Matrix<double, 6, 1> m_pose;
typedef Eigen::Matrix<double, 7, 1> m_pose_quat;

#define NUM_JOINTS          7
#define PI                  3.14159265359
#define OPTITRACK           1

struct objectTracking{
    std::string parent_id;
    std::string target_id;
    std::string mujoco_name;
};

struct robot_real{
    std::string name;
    std::vector<double> joint_positions;
};

struct object_real{
    std::string name;
    double positions[3];
    // x, y ,z, w
    double quaternion[4];
};

struct sceneState{
    std::vector<robot_real> robots;
    std::vector<object_real> objects;
};

class MuJoCo_realRobot_ROS{
    public:
        // Constructor
        MuJoCo_realRobot_ROS(int argc, char **argv, std::vector<std::string> optitrack_topic_names);
        ~MuJoCo_realRobot_ROS();

        // -----------------------------------------------------------------------------------
        // ROS subscribers
        ros::Subscriber jointStates_sub;
        void jointStates_callback(const sensor_msgs::JointState &msg);

        ros::Subscriber frankaStates_sub;
        void frankaStates_callback(const franka_msgs::FrankaState &msg);

        std::vector<ros::Subscriber> optiTrack_sub;
        void optiTrack_callback(const geometry_msgs::PoseStamped::ConstPtr &msg, const std::string& topic_name);

        ros::Subscriber robotBase_sub;
        void robotBasePose_callback(const geometry_msgs::PoseStamped &msg);
        // -----------------------------------------------------------------------------------

        // Return a struct representing the state of the scene
        sceneState returnScene();

        bool switchController(std::string controllerName);
        void sendTorquesToRealRobot(double torques[]);
        void sendPositionsToRealRobot(double positions[]);
        void sendVelocitiesToRealRobot(double velocities[]);

        void resetTorqueControl();

        bool jointsCallBackCalled;
        bool objectCallBackCalled;

    private:

        int numberOfObjects;
        std::vector<std::string> optitrack_objects;
        std::vector<bool> optitrack_objects_found;

        std::vector<objectTracking> objectTrackingList;
        std::vector<m_pose_quat> objectPoseList;
        std::vector<m_point> objectPosOffsetList;

        m_pose_quat robotBase;

        ros::NodeHandle *n;
        tf::TransformListener *listener;

        // Different publishers for different controllers
        ros::Publisher *torque_pub;
        ros::Publisher *position_pub;
        ros::Publisher *velocity_pub;

        std::string currentController;

        int getObjectId(std::string itemName);

        // Updates the joint states of the robot
        std::vector<robot_real> returnRobotState();
        // Loops through all known objects in the scene and updates their position and rotation
        std::vector<object_real> returnObjectsStates();
//        m_pose_quat filterObjectHistory(std::vector<m_pose_quat> objectPoses);

        // Sets the qpos value for the corresponding body id to the specified value


        double jointVals[7];
        double jointSpeeds[7];

        bool haltRobot = false;
        
};