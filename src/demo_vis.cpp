#include "MuJoCo_node.h"
#include "mujoco.h"
#include <GLFW/glfw3.h>

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

MuJoCo_realRobot_ROS* mujoco_realRobot_ROS;

void set_BodyPosition(mjModel *m, mjData* d, int bodyId, double position[3]);
void set_qPosVal(mjModel *m, mjData *d, int bodyId, bool freeJoint, int freeJntAxis, double val);
void setBodyQuat(mjModel *m, mjData *d, int bodyId, double quat[4]);

void setupMujocoWorld(){
    char error[1000];

//    model = mj_loadXML("/home/davidrussell/catkin_ws/src/realRobotExperiments_TrajOpt/Franka-emika-panda-arm/V1/cylinder_pushing.xml", NULL, error, 1000);
    model = mj_loadXML("/home/davidrussell/catkin_ws/src/realRobotExperiments_TrajOpt/MuJoCo_realRobot_ROS/mujoco_models/Franka_emika_scenes_V1/cylinder_pushing_heavyClutter_realWorld.xml", NULL, error, 1000);

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

    // cam.distance = 1.485;
    // cam.azimuth = 178.7;
    // cam.elevation = -31.3;
    // cam.lookat[0] = 0.325;
    // cam.lookat[1] = -0.0179;
    // cam.lookat[2] = 0.258;

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

    sceneState world = mujoco_realRobot_ROS->returnScene();

    for(int i = 0; i < world.robots.size(); i++){
        for(int j = 0; j < world.robots[i].joint_positions.size(); j++){
            mdata_real->qpos[j] = world.robots[i].joint_positions[j];
        }
    }

    for(int i = 0; i < world.objects.size(); i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, world.objects[i].name.c_str());
        set_BodyPosition(model, mdata_real, bodyId, world.objects[i].positions);
        setBodyQuat(model, mdata_real, bodyId, world.objects[i].quaternion);
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

int main(int argc, char **argv){

    setupMujocoWorld();

    // Create an instance of 
    // MuJoCo_realRobot_ROS mujocoController(true, &n);
    std::vector<std::string> optitrack_names = {"HotChocolate", "Bistro_1", "Bistro_2", "Bistro_3", "Bistro_4", "Bistro_5", "Bistro_6", "Bistro_7"};
    mujoco_realRobot_ROS = new MuJoCo_realRobot_ROS(argc, argv, optitrack_names);

    //mujoco_realRobot_ROS->switchController("effort_group_effort_controller");

    while(ros::ok()){

//        counter++;
//        if(counter < 200){
//            double torques[7] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//            mujoco_realRobot_ROS->sendTorquesToRealRobot(torques);
//        }

        render();
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
    mj_deactivate();
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


