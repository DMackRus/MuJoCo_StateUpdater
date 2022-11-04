#include "MuJoCo_node.h"

MuJoCo_realRobot_ROS::MuJoCo_realRobot_ROS(int argc, char **argv){
    ros::init(argc, argv, "MuJoCo_node");
    ros::NodeHandle n;

    jointStates_sub = n.subscribe("joint_states", 10, &MuJoCo_realRobot_ROS::jointStates_callback, this); 

}

void MuJoCo_realRobot_ROS::jointStates_callback(const sensor_msgs::JointState &msg){
    
    // TODO - make this programatic
    for(int i = 0; i < NUM_JOINTS; i++){
        jointVals[i] = msg.position[i];
    }
}

void MuJoCo_realRobot_ROS::updateMujocoData(mjModel* m, mjData* d){

    updateRobotState(m, d);

    updateScene(m, d);

    mj_forward(m, d);
}

void MuJoCo_realRobot_ROS::updateRobotState(mjModel* m, mjData* d){
    for(int i = 0; i < NUM_JOINTS; i++){
        d->qpos[i] = jointVals[i];
    }
}

void MuJoCo_realRobot_ROS::updateScene(mjModel* m, mjData* d){

}