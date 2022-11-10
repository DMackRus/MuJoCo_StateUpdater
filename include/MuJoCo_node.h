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
// Controller includes work stragith away when including ROS, nothing needed extra in the cmake
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/LoadController.h"

// MuJoCo Simulator
#include "mujoco.h"
#include "glfw3.h"

#include <Eigen/Dense>


using namespace Eigen;

typedef Eigen::Matrix<double, 3, 1> m_point;
typedef Eigen::Matrix<double, 4, 1> m_quat;
typedef Eigen::Matrix<double, 6, 1> m_pose;
typedef Eigen::Matrix<double, 7, 1> m_pose_quat;

#define NUM_JOINTS          7
#define PI                  3.14159265359
#define NUM_POSES_HISTORY   10             

struct objectTracking{
    std::string parent_id;
    std::string target_id;
    std::string mujoco_name;
};

class MuJoCo_realRobot_ROS{
    public:
        // Constructor
        MuJoCo_realRobot_ROS(int argc, char **argv, int _numberOfObjects);
        ~MuJoCo_realRobot_ROS();

        // ROS subscribers
        ros::Subscriber jointStates_sub;
        void jointStates_callback(const sensor_msgs::JointState &msg);

        // Updates Mujoco data
        void updateMujocoData(mjModel* m, mjData* d);

        bool switchController(std::string controllerName);
        void sendTorquesToRealRobot(double torques[]);

    private:

        int numberOfObjects;
        objectTracking myObject;

        std::vector<objectTracking> objectTrackingList;

        ros::NodeHandle *n;
        tf::TransformListener *listener;
        ros::Publisher *torque_pub;

        std::vector<m_pose_quat> objectPoses;

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

        float jointVals[7];
        
};