#include <iostream>

#include "mujoco.h"
#include <GLFW/glfw3.h>

// Ros includes
#include "ros/ros.h"
#include "MuJoCo_StateUpdater/Scene.h"
#include "MuJoCo_StateUpdater/Robot.h"
#include "MuJoCo_StateUpdater/RigidBody.h"

#define PI 3.14159265359

// -----------------------------------------------------------------------------------------
// Keyboard + mouse callbacks + variables
void scroll(GLFWwindow* window, double xoffset, double yoffset);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void windowCloseCallback(GLFWwindow * /*window*/) ;

bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;
// ------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------
// MuJoCo Visualisation variables
mjvCamera cam;                   // abstract camera
mjvScene scn;                   // abstract scene
mjvOption opt;			        // visualization options
mjrContext con;				    // custom GPU context
GLFWwindow *window;              // GLFW window
// ----------------------------------------------------------------------

mjModel *model;
mjData* mdata_real;

struct robot_real{
    std::string name;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
};

struct object_real{
    std::string name;
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

// Global variable for scene state
scene_state world;

void set_BodyPosition(mjModel *m, mjData* d, int bodyId, double position[3]);
void set_qPosVal(mjModel *m, mjData *d, int bodyId, bool freeJoint, int freeJntAxis, double val);
void setBodyQuat(mjModel *m, mjData *d, int bodyId, double quat[4]);

void setupMujocoWorld(){
    char error[1000];

    // TODO - this is hardcoded for now
    model = mj_loadXML("/home/david/catkin_ws/src/Panda_MPC/TrajOptKP/mujoco_models/Franka_panda/scene.xml", NULL, error, 1000);

    if(!model) {
        std::cout << "model xml Error" << std::endl;
        printf("%s\n", error);
    }

    // make data corresponding to model
    mdata_real = mj_makeData(model);

    // init GLFW, create window, make OpenGL context current, request v-sync
    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    window = glfwCreateWindow(1200, 900, "iLQR_Testing", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);

    // // create scene and context
    mjv_makeScene(model, &scn, 2000);
    mjr_makeContext(model, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    glfwSetWindowCloseCallback(window, windowCloseCallback);
}

void render(){

    for(auto & robot : world.robots){
        for(int j = 0; j < robot.joint_positions.size(); j++){
            mdata_real->qpos[j] = robot.joint_positions[j];     // TODO - this assume robot is first always in xml
        }
    }

    for(auto & object : world.objects){
        int bodyId = mj_name2id(model, mjOBJ_BODY, object.name.c_str());
        set_BodyPosition(model, mdata_real, bodyId, object.positions);
        setBodyQuat(model, mdata_real, bodyId, object.quaternion);
    }

    mj_forward(model, mdata_real);

    // get framebuffer viewport
    mjrRect viewport = { 0, 0, 0, 0 };
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(model, mdata_real, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

void SceneState_callback(const MuJoCo_StateUpdater::Scene &msg){

    for(int i = 0; i < msg.robots.size(); i++){

        world.robots[i].name = msg.robots[i].name;

        for (int j = 0; j < msg.robots[i].joint_positions.size(); j++){
            world.robots[i].joint_positions[j] = msg.robots[i].joint_positions[j];
            world.robots[i].joint_velocities[j] = msg.robots[i].joint_velocities[j];
        }

        // Apply joint offsets due to difference between my current panda model and real panda
        world.robots[i].joint_positions[5] -= PI / 2;
        world.robots[i].joint_positions[6] -= PI / 4;
    }

    for(int i = 0; i < msg.objects.size(); i++){

        world.objects[i].name = msg.objects[i].name;

        // Assign pose
        world.objects[i].positions[0] = msg.objects[i].pose[0];
        world.objects[i].positions[1] = msg.objects[i].pose[1];
        world.objects[i].positions[2] = msg.objects[i].pose[2];

        world.objects[i].quaternion[0] = msg.objects[i].pose[3];
        world.objects[i].quaternion[1] = msg.objects[i].pose[4];
        world.objects[i].quaternion[2] = msg.objects[i].pose[5];
        world.objects[i].quaternion[3] = msg.objects[i].pose[6];
    }
}

int main(int argc, char **argv){

    setupMujocoWorld();

    // Create a ROS node
    ros::init(argc, argv, "MuJoCo_real_world_vis");

    // Create a ROS subscriber to scene_state
    ros::NodeHandle nh;
    ros::Subscriber scene_sub = nh.subscribe("scene_state", 10, SceneState_callback);

    // TODO - Currently assume we always only have 1 robot which has 7 joints
    world.robots.push_back(robot_real());
    for(int i = 0; i < 7; i++){
        world.robots[0].joint_positions.push_back(0.0);
        world.robots[0].joint_velocities.push_back(0.0);
    }

    // Get the list of tracked objects from ROS param server
    std::vector<std::string> tracked_objects;
    if (nh.getParam("tracked_objects", tracked_objects)) {
    } else {
        ROS_WARN("Failed to get parameter 'tracked_objects'");
    }

    for(auto object_name : tracked_objects){
        world.objects.push_back(object_real());
    }

    // Main loop
    ros::Rate rate(30); // 30 Hz loop rate
    while (ros::ok()) {
        ros::spinOnce(); // Process incoming messages
        render();        // Render your scene
        rate.sleep();    // Sleep to maintain the loop rate
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------
//
//              Camera viewing callbacks using mouse and keyboard
//
//---------------------------------------------------------------------------------------------------

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods){

}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods){
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos){
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(model, action, dx / height, dy / height, &scn, &cam);

}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset){
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void windowCloseCallback(GLFWwindow * /*window*/) {
    // Use this flag if you wish not to terminate now.
    // glfwSetWindowShouldClose(window, GLFW_FALSE);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(mdata_real);
    mj_deleteModel(model);
}

void set_BodyPosition(mjModel* m, mjData* d, int bodyId, double position[3]){

    for(int i = 0; i < 3; i++){
        set_qPosVal(m, d, bodyId, true, i, position[i]);
    }

}

void set_qPosVal(mjModel *m, mjData *d, int bodyId, bool freeJoint, int freeJntAxis, double val){
    const int jointIndex = m->body_jntadr[bodyId];
    const int posIndex = m->jnt_qposadr[jointIndex];

    // free joint axis can be any number between 0 and 2 (x, y, z)
    if(freeJntAxis < 0 or freeJntAxis > 3){
        std::cout << "you have used set_qPosVal wrong!!!!!!!!!!! Freejntaxis was: " << freeJntAxis << std::endl;
    }

    if(!freeJoint){
        // automatically return 1 val
        d->qpos[posIndex] = val;

    }
    else{
        // have to add on freeJntAxis to get desired x y or z component of free joint
        d->qpos[posIndex + freeJntAxis] = val;
    }

}

void setBodyQuat(mjModel *m, mjData *d, int bodyId, double quat[4]){
    int jointIndex = m->body_jntadr[bodyId];
    int qposIndex = m->jnt_qposadr[jointIndex];

    d->qpos[qposIndex + 3] = quat[3];
    d->qpos[qposIndex + 4] = quat[0];
    d->qpos[qposIndex + 5] = quat[1];
    d->qpos[qposIndex + 6] = quat[2];
}

// TEMP stuff tobe moved over for testing
//    double robot_pos_command[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//    for(int i = 0; i < 7; i++){
//        robot_pos_command[i] = robot_joints[i];
//    }
//
//    robot_pos_command[0] += 0.3;

//        counter++;
//        if(counter < 200){
//            double torques[7] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//            mujoco_realRobot_ROS->sendTorquesToRealRobot(torques);
//        }
//mujoco_realRobot_ROS->sendPositionsToRealRobot(robot_pos_command);


