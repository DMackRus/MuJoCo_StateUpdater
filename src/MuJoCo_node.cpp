#include "MuJoCo_node.h"


MuJoCo_realRobot_ROS::MuJoCo_realRobot_ROS(bool _visualise, ros::NodeHandle *n){
    jointStates_sub = n->subscribe("joint_states", 10, &MuJoCo_realRobot_ROS::jointStates_callback, this); 

    visualise = _visualise;
}

void MuJoCo_realRobot_ROS::jointStates_callback(const sensor_msgs::JointState &msg){
    
    std::cout << "joint 1: " << msg.position[1] << std::endl;


}

int main(int argc, char **argv){
    ros::init(argc, argv, "MuJoCo_node");
    ros::NodeHandle n;

    std::cout << "Hello, world!" << std::endl;

    // Create an instance of 
    MuJoCo_realRobot_ROS mujocoController(true, &n);

    ros::spin();

    return 0;
}