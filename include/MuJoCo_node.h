#pragma once

// General Includes
#include <cmath>
#include <string>
#include <iostream>
#include <thread>

// ROS includes
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>

using namespace std;

class MuJoCo_realRobot_ROS{
    public:
        MuJoCo_realRobot_ROS(bool _visualise, ros::NodeHandle *n);

        ros::Subscriber jointStates_sub;

        void jointStates_callback(const sensor_msgs::JointState &msg);



    private:

        bool visualise;

};