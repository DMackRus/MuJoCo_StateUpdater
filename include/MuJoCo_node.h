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
        MuJoCo_realRobot_ROS(bool _visualise, ros::NodeHandle *n);

        // ROS subscribers
        ros::Subscriber jointStates_sub;
        void jointStates_callback(const sensor_msgs::JointState &msg);

        // Visualisation functions + variables
        bool visualise;
        void render();

        void cloneMjData(mjData* data);
        mjModel* model;
        mjData* mdata_real;

    private:

        void setupMujocoWorld();
        void updateMujocoData();


        float jointVals[7];
        
};