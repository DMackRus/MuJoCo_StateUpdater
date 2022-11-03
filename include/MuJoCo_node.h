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
        MuJoCo_realRobot_ROS(bool _visualise, ros::NodeHandle *n);

        ros::Subscriber jointStates_sub;

        void jointStates_callback(const sensor_msgs::JointState &msg);

        bool visualise;

        void render();
        

    private:

        void setupMujocoWorld();
        void updateMujocoData();
        float jointVals[7];
        

        // void scroll(GLFWwindow* window, double xoffset, double yoffset);
        // void mouse_move(GLFWwindow* window, double xpos, double ypos);
        // void mouse_button(GLFWwindow* window, int button, int act, int mods);
        // void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);

};