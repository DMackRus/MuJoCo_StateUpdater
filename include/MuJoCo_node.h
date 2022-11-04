#pragma once

// General Includes
#include <cmath>
#include <string>
#include <iostream>
#include <thread>

// ROS includes
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>

// MuJoCo Simulator
#include "mujoco.h"
#include "glfw3.h"

using namespace std;

#define NUM_JOINTS  7
#define PI          3.14159265359

class MuJoCo_realRobot_ROS{
    public:
        // Constructor
        MuJoCo_realRobot_ROS(int argc, char **argv);

        // ROS subscribers
        ros::Subscriber jointStates_sub;
        void jointStates_callback(const sensor_msgs::JointState &msg);

        // Updates Mujoco data
        void updateMujocoData(mjModel* m, mjData* d);

    private:

        
        void updateRobotState(mjModel* m, mjData* d);
        void updateScene(mjModel* m, mjData* d);


        float jointVals[7];
        
};