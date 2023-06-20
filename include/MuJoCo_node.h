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
#include "mujoco.h"
#include <GLFW/glfw3.h>

#include <Eigen/Dense>


using namespace Eigen;

typedef Eigen::Matrix<double, 3, 1> m_point;
typedef Eigen::Matrix<double, 4, 1> m_quat;
typedef Eigen::Matrix<double, 6, 1> m_pose;
typedef Eigen::Matrix<double, 7, 1> m_pose_quat;

#define NUM_JOINTS          7
#define PI                  3.14159265359
#define NUM_POSES_HISTORY   10
#define OPTITRACK           1

struct objectTracking{
    std::string parent_id;
    std::string target_id;
    std::string mujoco_name;
};

class MuJoCo_realRobot_ROS{
    public:
        // Constructor
        MuJoCo_realRobot_ROS(int argc, char **argv, int _numberOfObjects, std::string optitrackTopicName, std::vector<m_point> _objectPosOffsetList);
        ~MuJoCo_realRobot_ROS();

        // -----------------------------------------------------------------------------------
        // ROS subscribers
        ros::Subscriber jointStates_sub;
        void jointStates_callback(const sensor_msgs::JointState &msg);

        ros::Subscriber frankaStates_sub;
        void frankaStates_callback(const franka_msgs::FrankaState &msg);

        ros::Subscriber optiTrack_sub;
        void optiTrack_callback(const geometry_msgs::PoseStamped &msg);

        ros::Subscriber robotBase_sub;
        void robotBasePose_callback(const geometry_msgs::PoseStamped &msg);
        // -----------------------------------------------------------------------------------

        // Updates Mujoco data
        void updateMujocoData(mjModel* m, mjData* d);

        bool switchController(std::string controllerName);
        void sendTorquesToRealRobot(double torques[]);
        void sendPositionsToRealRobot(double positions[]);

        void resetTorqueControl();

        bool jointsCallBackCalled;
        bool objectCallBackCalled;

    private:

        int numberOfObjects;

        std::vector<objectTracking> objectTrackingList;
        std::vector<m_pose_quat> objectPoseList;
        std::vector<m_point> objectPosOffsetList;

        m_pose_quat robotBase;

        ros::NodeHandle *n;
        tf::TransformListener *listener;
        ros::Publisher *torque_pub;

        std::string currentController;

        // Updates the joint states of the robot
        void updateRobotState(mjModel* m, mjData* d);
        // Loops through all known objects in the scene and updates their position and rotation
        void updateScene(mjModel* m, mjData* d);
        void updateObjectHistory();
        m_pose_quat filterObjectHistory(std::vector<m_pose_quat> objectPoses);

        // Sets the qpos value for the corresponding body id to the specified value
        void set_BodyPosition(mjModel *m, mjData* d, int bodyId, m_point pos);
        void set_qPosVal(mjModel *m, mjData *d, int bodyId, bool freeJoint, int freeJntAxis, double val);
        void setBodyQuat(mjModel *m, mjData *d, int bodyId, Quaternionf q);

        double jointVals[7];
        double jointSpeeds[7];

        bool haltRobot = false;
        
};