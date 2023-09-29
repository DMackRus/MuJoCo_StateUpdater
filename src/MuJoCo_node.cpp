#include "MuJoCo_node.h"


MuJoCo_realRobot_ROS::MuJoCo_realRobot_ROS(int argc, char **argv, std::vector<std::string> optitrack_topic_names){

    ros::init(argc, argv, "MuJoCo_node");

    n = new ros::NodeHandle();
    listener = new tf::TransformListener();

    jointStates_sub = n->subscribe("joint_states", 10, &MuJoCo_realRobot_ROS::jointStates_callback, this);
    frankaStates_sub = n->subscribe("/franka_state_controller/franka_states", 10, &MuJoCo_realRobot_ROS::frankaStates_callback, this);
    robotBase_sub = n->subscribe("/mocap/rigid_bodies/pandaRobot/pose", 10, &MuJoCo_realRobot_ROS::robotBasePose_callback, this);

    for(int i = 0; i < optitrack_topic_names.size(); i++){
        auto callback = std::bind(&MuJoCo_realRobot_ROS::optiTrack_callback, this, std::placeholders::_1, optitrack_topic_names[i]);
        std::string topic = "/mocap/rigid_bodies/" + optitrack_topic_names[i] + "/pose";
        optiTrack_sub.push_back(n->subscribe<geometry_msgs::PoseStamped>(topic, 1, callback));
    }

    numberOfObjects = optitrack_topic_names.size();
    optitrack_objects = optitrack_topic_names;

    torque_pub = new ros::Publisher(n->advertise<std_msgs::Float64MultiArray>("//effort_group_position_controller/command", 1));

    for(int i = 0; i < optitrack_topic_names.size(); i++){
        objectTrackingList.push_back(objectTracking());
        objectPoseList.push_back(m_pose_quat());
        objectPosOffsetList.push_back(m_point());

        objectTrackingList[i].parent_id = "/panda_link0";
        objectTrackingList[i].target_id = "/ar_marker_3";
        objectTrackingList[i].mujoco_name = optitrack_topic_names[i];
    }

    // TODO - when class is instantied, have it check what controllers are running and keep track of it
    currentController = "position_joint_trajectory_controller";

    jointsCallBackCalled = false;
    objectCallBackCalled = false;

}

int MuJoCo_realRobot_ROS::getObjectId(std::string itemName){
    for(int i = 0; i < optitrack_objects.size(); i++){
        if(itemName == optitrack_objects[i]){
            return i;
        }
    }
    // If object Id is not found, this will crash the program!
    return -1;
}

// order of poses is {x, y, z, w, wx, wy, wz}
void MuJoCo_realRobot_ROS::optiTrack_callback(const geometry_msgs::PoseStamped::ConstPtr &msg, const std::string& topic_name){
    m_pose objectPose;
    int objectId = getObjectId(topic_name);

    // For coordinate frames in mujoco, offset the object coordinates by the base frame of robot
    // as that is origin in mujoco
    objectPoseList[objectId](0) = msg->pose.position.x - robotBase(0);
    objectPoseList[objectId](1) = msg->pose.position.y - robotBase(1);
    objectPoseList[objectId](2) = msg->pose.position.z - robotBase(2);

    objectPoseList[objectId](3) = msg->pose.orientation.w;
    objectPoseList[objectId](4) = msg->pose.orientation.x;
    objectPoseList[objectId](5) = msg->pose.orientation.y;
    objectPoseList[objectId](6) = msg->pose.orientation.z;

//    objectCallBackCalled = true;
}

MuJoCo_realRobot_ROS::~MuJoCo_realRobot_ROS(){
    delete n;
    delete listener;
}

void MuJoCo_realRobot_ROS::jointStates_callback(const sensor_msgs::JointState &msg){
    
    // TODO - make this dependant on size of msg?
    for(int i = 0; i < NUM_JOINTS; i++){
        jointVals[i] = msg.position[i];
    }

    jointsCallBackCalled = true;
}

void MuJoCo_realRobot_ROS::frankaStates_callback(const franka_msgs::FrankaState &msg){

    for(int i = 0; i < NUM_JOINTS; i++){
        jointSpeeds[i] = msg.dq[i];
    }
}

void MuJoCo_realRobot_ROS::robotBasePose_callback(const geometry_msgs::PoseStamped &msg){
    robotBase(0) = msg.pose.position.x;
    robotBase(1) = msg.pose.position.y;
    robotBase(2) = msg.pose.position.z;

    robotBase(3) = msg.pose.orientation.w;
    robotBase(4) = msg.pose.orientation.x;
    robotBase(5) = msg.pose.orientation.y;
    robotBase(6) = msg.pose.orientation.z;
}

sceneState MuJoCo_realRobot_ROS::returnScene(){
    sceneState world;

    ros::spinOnce();
    std::vector<robot> robots = returnRobotState();
    std::vector<object> objects = returnObjectsStates();

    world.robots = robots;
    world.objects = objects;

    return world;
}

// TODO - should probably add abaility for multiple robots, which should be specified by constructo call
std::vector<robot> MuJoCo_realRobot_ROS::returnRobotState() {

    std::vector<robot> robots;
    robots.push_back(robot());
    robots[0].name = "panda";

    for(int i = 0; i < NUM_JOINTS; i++){
        if(i == 5){
            robots[0].joint_positions.push_back(jointVals[i] - PI/2);
        }
        else if(i == 6){
            robots[0].joint_positions.push_back(jointVals[i] - PI/4);
        }
        else{
            robots[0].joint_positions.push_back(jointVals[i]);
        }

    }


    return robots;
}

std::vector<object> MuJoCo_realRobot_ROS::returnObjectsStates(){
    tf::StampedTransform transform;
    std::vector<object> objects;
    if(OPTITRACK){
        for(int i = 0; i < numberOfObjects; i++){
            objects.push_back(object());
            objects[i].name = optitrack_objects[i];
            // x, y, z
            objects[i].positions[0] = objectPoseList[i](0);
            objects[i].positions[1] = -objectPoseList[i](2);
            objects[i].positions[2] = objectPoseList[i](1);

            // This seems so random, Optitrack and mujoco conversion is weird...
            // x, y, z, w
            objects[i].quaternion[0] = objectPoseList[i](4);
            objects[i].quaternion[1] = -objectPoseList[i](6);
            objects[i].quaternion[2] = objectPoseList[i](5);
            objects[i].quaternion[3] = objectPoseList[i](3);

//            int itemId = mj_name2id(m, mjOBJ_BODY, objectTrackingList[i].mujoco_name.c_str());
//            m_point bodyPoint;
//            bodyPoint(0) = objectPoseList[i](0); //  + objectPosOffsetList[i](0);    // mujoco x
//            bodyPoint(1) = -objectPoseList[i](2); // + objectPosOffsetList[0](2);    // mujoco y
//            bodyPoint(2) = objectPoseList[i](1); //  + objectPosOffsetList[0](1);    // mujoco z (up/down)
//            set_BodyPosition(m, d, itemId, bodyPoint);
//
//            Quaternionf q;
//            q.w() = objectPoseList[i](3);
//            q.x() = objectPoseList[i](4);
//            q.y() = -objectPoseList[i](6);
//            q.z() = objectPoseList[i](5);
//            setBodyQuat(m, d, itemId, q);
        }
    }
//    else {
//        for (int i = 0; i < numberOfObjects; i++) {
//            try {
//                listener->lookupTransform(objectTrackingList[i].parent_id, objectTrackingList[i].target_id,
//                                          ros::Time(0), transform);
//
//                int cheezit_id = mj_name2id(m, mjOBJ_BODY, objectTrackingList[i].mujoco_name.c_str());
//
//                m_point bodyPos;
//                bodyPos(0) = transform.getOrigin().x();
//                bodyPos(1) = transform.getOrigin().y();
//                bodyPos(2) = transform.getOrigin().z();
//                set_BodyPosition(m, d, cheezit_id, bodyPos);
//
//                float x = transform.getRotation().x();
//                float y = transform.getRotation().y();
//                float z = transform.getRotation().z();
//                float w = transform.getRotation().w();
//
//                Quaternionf q = {w, x, y, z};
//                setBodyQuat(m, d, cheezit_id, q);
//
//            }
//            catch (tf::TransformException ex) {
//                std::cout << " no ar marker 3 found" << std::endl;
//                ROS_ERROR("%s", ex.what());
//            }
//        }
//    }

    return objects;
}

// TODO - check loaded controllers, only load controller if required.
bool MuJoCo_realRobot_ROS::switchController(std::string controllerName){
    ros::ServiceClient load_controller = n->serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");

    controller_manager_msgs::LoadController load_controller_req;
    load_controller_req.request.name = controllerName;
    load_controller.call(load_controller_req);

    ros::ServiceClient switch_controller = n->serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

    std::vector<std::string> start_controller;
    start_controller.push_back(controllerName);
    std::vector<std::string> stop_controller;
    stop_controller.push_back("position_joint_trajectory_controller");
    controller_manager_msgs::SwitchController switch_controller_req;
    switch_controller_req.request.start_controllers = start_controller;
    switch_controller_req.request.stop_controllers = stop_controller;
    switch_controller_req.request.strictness = 1;
    switch_controller_req.request.start_asap = false;
    ros::service::waitForService("/controller_manager/switch_controller", ros::Duration(5));
    switch_controller.call(switch_controller_req);
    if (switch_controller_req.response.ok){
        ROS_INFO_STREAM("Controller switch correctly");
    }
    else{
        ROS_ERROR_STREAM("Error occured trying to switch controller");
        return 0;
    }

    return switch_controller_req.response.ok;
}

void MuJoCo_realRobot_ROS::sendTorquesToRealRobot(double torques[]){
    std_msgs::Float64MultiArray  desired_torques;
    double jointSpeedLimits[NUM_JOINTS] = {0.5, 0.5, 0.5, 0.5, 1, 1.5, 1.5};
    bool jointVelsSafe = true;

    if(!haltRobot){
        std::cout << "torques Sent: " << torques[0] << ", " << torques[1] << ", " << torques[2] << ", " << torques[3] << ", " << torques[4] << ", " << torques[5] << ", " << torques[6] << ", " << std::endl;
        for(int i = 0; i < NUM_JOINTS; i++){
            double safeTorque = torques[i];

            if(jointSpeeds[i] > jointSpeedLimits[i]){
                std::cout << "joint " << i <<  " speed: " << jointSpeeds[i] << std::endl;
                std::cout << "safety vel triggered" << std::endl;
                haltRobot = true;
                safeTorque = 0.0;
            }

            if(jointSpeeds[i] < -jointSpeedLimits[i]){
                std::cout << "joint " << i <<  " speed: " << jointSpeeds[i] << std::endl;
                std::cout << "safety vel triggered" << std::endl;
                haltRobot = true;
                safeTorque = 0.0;
            }

            desired_torques.data.push_back(safeTorque);
        }

        torque_pub->publish(desired_torques);
    }
    else{
        for(int i = 0; i < NUM_JOINTS; i++){
            desired_torques.data.push_back(0.0);
        }
        torque_pub->publish(desired_torques);
    }
}

void MuJoCo_realRobot_ROS::sendPositionsToRealRobot(double positions[]){
    std_msgs::Float64MultiArray  desired_positions;
    double jointSpeedLimits[NUM_JOINTS] = {0.6, 0.5, 0.8, 0.5, 1, 2, 3};
    //double torqueLimits[NUM_JOINTS] = {10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0};

    if(!haltRobot){
        //std::cout << "torques Sent: " << torques[0] << ", " << torques[1] << ", " << torques[2] << ", " << torques[3] << ", " << torques[4] << ", " << torques[5] << ", " << torques[6] << ", " << std::endl;
        for(int i = 0; i < NUM_JOINTS; i++){

            if(jointSpeeds[i] > jointSpeedLimits[i]){
                std::cout << "safety vel triggered" << std::endl;
                std::cout << "joint " << i <<  " speed: " << jointSpeeds[i] << std::endl;
                std::cout << "joints at safety trigger: " << jointVals[i] << std::endl;
                haltRobot = true;
            }

            if(jointSpeeds[i] < -jointSpeedLimits[i]){
                std::cout << "safety vel triggered" << std::endl;
                std::cout << "joint " << i <<  " speed: " << jointSpeeds[i] << std::endl;
                std::cout << "joints at safety trigger: " << jointVals[i] << std::endl;
                haltRobot = true;
            }

            desired_positions.data.push_back(positions[i]);

        }
        //std::cout << "torque 0: " << desired_torques.data[0] << std::endl;
        torque_pub->publish(desired_positions);
    }
    else{
        // dont publish anything

    }

    ros::spinOnce();
}


// void MuJoCo_realRobot_ROS::resetTorqueControl(){
//     haltRobot = false;
// }