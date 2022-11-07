#include "MuJoCo_node.h"

MuJoCo_realRobot_ROS::MuJoCo_realRobot_ROS(int argc, char **argv, int _numberOfObjects){

    ros::init(argc, argv, "MuJoCo_node");

    n = new ros::NodeHandle();
    listener = new tf::TransformListener();

    jointStates_sub = n->subscribe("joint_states", 10, &MuJoCo_realRobot_ROS::jointStates_callback, this); 

    numberOfObjects = _numberOfObjects;

    for(int i = 0; i < numberOfObjects; i++){
        objectTrackingList.push_back(objectTracking());
    }

    objectTrackingList[0].parent_id = "/panda_link0";
    objectTrackingList[0].target_id = "/ar_marker_3";
    objectTrackingList[0].mujoco_name = "cheezit";

    objectTrackingList[1].parent_id = "/panda_link0";
    objectTrackingList[1].target_id = "/panda_hand_tcp";
    objectTrackingList[1].mujoco_name = "EE";

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
}

void MuJoCo_realRobot_ROS::updateMujocoData(mjModel* m, mjData* d){

    ros::spinOnce();

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

