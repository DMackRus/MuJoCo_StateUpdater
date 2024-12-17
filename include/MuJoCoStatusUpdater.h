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

#include "MuJoCo_StateUpdater/Scene.h"
#include "MuJoCo_StateUpdater/Robot.h"
#include "MuJoCo_StateUpdater/RigidBody.h"

#include <boost/bind.hpp>
#include <boost/function.hpp>

#define NUM_JOINTS          7
#define PI                  3.14159265359

struct point{
    double x;
    double y;
    double z;
};

struct robot_real{
    std::string name;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
};

struct object_real{
    std::string name;
    double previous_raw_positions[3];
    double raw_positions[3];
    double positions[3];
    double linear_velocities[3];
    // x, y ,z, w
    double quaternion[4];
    double angular_velocities[3];
    float confidence;
};

struct scene_state{
    std::vector<robot_real> robots;
    std::vector<object_real> objects;
};

class MuJoCoStateUpdater{
    public:
        // Constructor
        MuJoCoStateUpdater(int argc, char **argv);
        ~MuJoCoStateUpdater();

        // -----------------------------------------------------------------------------------
        // ROS subscribers
        ros::Subscriber joint_states_sub;
        void JointStates_callback(const sensor_msgs::JointState &msg);

        std::vector<ros::Subscriber> optitrack_sub;
        void OptiTrack_callback(const geometry_msgs::PoseStamped::ConstPtr &msg, const std::string& topic_name);

        ros::Subscriber robot_base_sub;
        void RobotBasePose_callback(const geometry_msgs::PoseStamped &msg);
        // -----------------------------------------------------------------------------------

        // ROS publishers
        ros::Publisher scene_pub;   // Scene publishers (robots and objects)

        std::map<std::string, object_real> tracked_object_poses;

        std::vector<robot_real> robots;

    private:

        point robotBase;

        ros::NodeHandle *n;
};