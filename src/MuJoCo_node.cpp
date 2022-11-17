#include "MuJoCo_node.h"

MuJoCo_realRobot_ROS::MuJoCo_realRobot_ROS(int argc, char **argv, int _numberOfObjects){

    ros::init(argc, argv, "MuJoCo_node");

    n = new ros::NodeHandle();
    listener = new tf::TransformListener();

    jointStates_sub = n->subscribe("joint_states", 10, &MuJoCo_realRobot_ROS::jointStates_callback, this); 
    frankaStates_sub = n->subscribe("/franka_state_controller/franka_states", 10, &MuJoCo_realRobot_ROS::frankaStates_callback, this);

    torque_pub = new ros::Publisher(n->advertise<std_msgs::Float64MultiArray>("/effort_group_effort_controller/command", 1));

    numberOfObjects = _numberOfObjects;

    for(int i = 0; i < numberOfObjects; i++){
        objectTrackingList.push_back(objectTracking());
    }

    // objectTrackingList[0].parent_id = "/panda_link0";
    // objectTrackingList[0].target_id = "/ar_marker_3";
    // objectTrackingList[0].mujoco_name = "cheezit";

    // objectTrackingList[1].parent_id = "/panda_link0";
    // objectTrackingList[1].target_id = "/panda_hand_tcp";
    // objectTrackingList[1].mujoco_name = "EE";

    // TODO - when class is instantied, have it check what controllers are running and keep track of it
    currentController = "position_joint_trajectory_controller";

    firstCallbackCalled = false;

}

MuJoCo_realRobot_ROS::~MuJoCo_realRobot_ROS(){
    delete n;
    delete listener;
}

void MuJoCo_realRobot_ROS::jointStates_callback(const sensor_msgs::JointState &msg){
    
    // TODO - make this programatic
    for(int i = 0; i < NUM_JOINTS; i++){
        jointVals[i] = msg.position[i];
    }

    firstCallbackCalled = true;
}

void MuJoCo_realRobot_ROS::frankaStates_callback(const franka_msgs::FrankaState &msg){

    for(int i = 0; i < NUM_JOINTS; i++){
        jointSpeeds[i] = msg.dq[i];
    }
}

void MuJoCo_realRobot_ROS::updateMujocoData(mjModel* m, mjData* d){

    ros::spinOnce();

    updateRobotState(m, d);

    // updateScene(m, d);

    mj_forward(m, d);
}

void MuJoCo_realRobot_ROS::updateRobotState(mjModel* m, mjData* d){
    for(int i = 0; i < NUM_JOINTS; i++){
        if(i == 5){
            d->qpos[i] = jointVals[i] - PI/2;
        }
        else if(i == 6){
            d->qpos[i] = jointVals[i] - PI/4;
        }
        else{
            d->qpos[i] = jointVals[i];
        }
    }
}

void MuJoCo_realRobot_ROS::updateScene(mjModel* m, mjData* d){
    tf::StampedTransform transform;

   
    for(int i = 0; i < numberOfObjects; i++){

        try{
            listener->lookupTransform(objectTrackingList[i].parent_id, objectTrackingList[i].target_id, ros::Time(0), transform);

            int cheezit_id = mj_name2id(m, mjOBJ_BODY, objectTrackingList[i].mujoco_name.c_str());

            m_point bodyPos;
            bodyPos(0) = transform.getOrigin().x();
            bodyPos(1) = transform.getOrigin().y();
            bodyPos(2) = transform.getOrigin().z();
            set_BodyPosition(m, d, cheezit_id, bodyPos);
            
            float x = transform.getRotation().x();
            float y = transform.getRotation().y();
            float z = transform.getRotation().z();
            float w = transform.getRotation().w();

            Quaternionf q = {w, x, y, z};
            setBodyQuat(m, d, cheezit_id, q);

        }
        catch (tf::TransformException ex){
            std::cout << " no ar marker 3 found" << std::endl;
            ROS_ERROR("%s",ex.what());
        }
    }
}

m_pose_quat filterObjectHistory(std::vector<m_pose_quat> objectPoses){
    m_pose_quat filteredPose;

    // filteredPose = objectPoses[0];

    // for(int i = 0; i < NUM_POSES_HISTORY - 1; i++){
    //     for(int j = 0; j < 7; j++){
    //         filteredPose[i](j) += objectPoses[i](j);
    //     }
    // }

    // for(int j = 0; j < 7; j++){
    //     filteredPose[i](j) /= NUM_POSES_HISTORY;
    // }

    return filteredPose;
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
    double jointSpeedLimits[NUM_JOINTS] = {0.5, 0.5, 0.5, 0.5, 1, 1, 1};
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

// void MuJoCo_realRobot_ROS::resetTorqueControl(){
//     haltRobot = false;
// }

//----------------------------------------------------------------------------------------------------------------------
//
//                             Setting positions/rotations of objects in MuJoCo
//
//---------------------------------------------------------------------------------------------------------------------

void MuJoCo_realRobot_ROS::set_BodyPosition(mjModel* m, mjData* d, int bodyId, m_point pos){

    for(int i = 0; i < 3; i++){
        set_qPosVal(m, d, bodyId, true, i, pos(i));
    }

}

void MuJoCo_realRobot_ROS::set_qPosVal(mjModel *m, mjData *d, int bodyId, bool freeJoint, int freeJntAxis, double val){
    const int jointIndex = m->body_jntadr[bodyId];
    const int dofIndex = m->jnt_dofadr[jointIndex];
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

void MuJoCo_realRobot_ROS::setBodyQuat(mjModel *m, mjData *d, int bodyId, Quaternionf q){
    int jointIndex = m->body_jntadr[bodyId];
    int qposIndex = m->jnt_qposadr[jointIndex];

    d->qpos[qposIndex + 3] = q.w();
    d->qpos[qposIndex + 4] = q.x();
    d->qpos[qposIndex + 5] = q.y();
    d->qpos[qposIndex + 6] = q.z();
}

