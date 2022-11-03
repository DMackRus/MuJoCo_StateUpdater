#include "MuJoCo_node.h"



mjModel* model;
mjData* mdata_real;
mjvCamera cam;                   // abstract camera
mjvScene scn;                   // abstract scene
mjvOption opt;			        // visualization options
mjrContext con;				    // custom GPU context
GLFWwindow *window;              // GLFW window

bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

void scroll(GLFWwindow* window, double xoffset, double yoffset);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods){
    // backspace: reset simulation
//    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
//    {
//        mj_resetData(model, mdata);
//        mj_forward(model, mdata);
//    }
//
//    float ctrlChange = 0.02;
//    if(key == 265 && act == GLFW_PRESS ){
//        // forwards arrow
//        mdata->ctrl[0] += ctrlChange;
//    }
//    else if(key == 263 && act == GLFW_PRESS ){
//        // back arrow
//        mdata->ctrl[0] -= ctrlChange;
//    }
//    else if(key == 264 && act == GLFW_PRESS ){
//        // left arrow
//        mdata->ctrl[1] += ctrlChange;
//    }
//    else if(key == 262 && act == GLFW_PRESS ){
//        // right arrow
//        mdata->ctrl[1] -= ctrlChange;
//    }
//    else if(key == 257 && act == GLFW_PRESS ){
//        //enter key
//        m_state collState;
//        //collState = modelTranslator->returnState(mdata);
//        //collState(0) += 0.00001;
//        collState << 0, 0, 0.02, 0, 0, 0, 0, 0;
//
//        modelTranslator->setState(mdata, collState);
//        mj_forward(model, mdata);
//        m_dof accels;
//        accels = modelTranslator->returnAccelerations(mdata);
//        cout << "accelerations: " << endl << accels << endl;
//        mju_copy(mdata->qacc_warmstart, mdata->qacc, model->nv);
//        for( int rep=1; rep<5; rep++ ){
//            mju_copy(mdata->qacc_warmstart, mdata->qacc, model->nv);
//            mj_forward(model, mdata);
//            //mj_forwardSkip(model, mdata, mjSTAGE_VEL, 1);
//            accels = modelTranslator->returnAccelerations(mdata);
//            cout << "accelerations: " << endl << accels << endl;
//
//        }
//
//    }

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


MuJoCo_realRobot_ROS::MuJoCo_realRobot_ROS(bool _visualise, ros::NodeHandle *n){
    jointStates_sub = n->subscribe("joint_states", 10, &MuJoCo_realRobot_ROS::jointStates_callback, this); 

    visualise = _visualise;
    
    setupMujocoWorld();

}

void MuJoCo_realRobot_ROS::jointStates_callback(const sensor_msgs::JointState &msg){
    
    // TODO - make this programatic
    for(int i = 0; i < NUM_JOINTS; i++){
        // TODO - think how to fix this or make it less epcific, mujoco model does not match
        // values returned by franka emika arm
        jointVals[i] = msg.position[i];
        // if(i == 5){
        //     jointVals[i] = msg.position[i] - (PI / 2);
        // }
        // else if(i == 6){
        //     jointVals[i] = msg.position[i] - (PI / 4);
        // }
        // else{
        //     jointVals[i] = msg.position[i];
        // }
    }
}

void MuJoCo_realRobot_ROS::setupMujocoWorld(){
    char error[1000];

    // TODO - fix this hard coded path issue
    // model = mj_loadXML("/home/davidrussell/catkin_ws/src/MuJoCo_realRobot_ROS/models/reaching.xml", NULL, error, 1000);
    model = mj_loadXML("/home/davidrussell/catkin_ws/src/MuJoCo_realRobot_ROS/models/new/franka_emika_panda/scene.xml", NULL, error, 1000);

    if( !model ) {
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

void MuJoCo_realRobot_ROS::updateMujocoData(){

    //std::cout << "joint vals: ";
    for(int i = 0; i < NUM_JOINTS; i++){
        //std::cout << i << ": " << jointVals[i];
        mdata_real->qpos[i] = jointVals[i];
    }
    //std::cout << std::endl;

    mj_forward(model, mdata_real);
}

void MuJoCo_realRobot_ROS::render(){

    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    // mjtNum simstart = mdata_real->time;
    // while (mdata_real->time - simstart < 1.0 / 60.0){
    //     mj_step(model, mdata_real);
    // }

    updateMujocoData();

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
    ros::init(argc, argv, "MuJoCo_node");
    ros::NodeHandle n;

    std::cout << "Hello, world!" << std::endl;

    // Create an instance of 
    MuJoCo_realRobot_ROS mujocoController(true, &n);

    mj_step(model, mdata_real);

    while(ros::ok()){
        if(mujocoController.visualise){
            mujocoController.render();

            ros::spinOnce();
        }
    }

    return 0;
}