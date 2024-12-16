#include "MuJoCoStatusUpdater.h"

int main(int argc, char **argv){
    std::vector<std::string> tracked_objects = {"HotChocolate"};
    // Create an instance of MuJocoStatusUpdater
    MuJoCoStateUpdater mujoco_state_updater(argc, argv, tracked_objects);

    while(ros::ok()){
        scene_state world = mujoco_state_updater.ReturnScene();

        // create message
        MuJoCo_StateUpdater::Scene scene_msg;

        // fill in message
        for(auto robot : world.robots){
            MuJoCo_StateUpdater::Robot robot_msg;
            robot_msg.name = robot.name;

            for(int i = 0; i < NUM_JOINTS; i++){
                robot_msg.joint_positions.push_back(robot.joint_positions[i]);
                robot_msg.joint_velocities.push_back(robot.joint_velocities[i]);

            }
            scene_msg.robots.push_back(robot_msg);
        }

        for(auto object : world.objects){
            MuJoCo_StateUpdater::RigidBody object_msg;
            object_msg.name = object.name;

            // Pose is x, y, z, qx, qy, qz, w
            object_msg.pose = {object.positions[0], object.positions[1], object.positions[2],
                               object.quaternion[0], object.quaternion[1], object.quaternion[2], object.quaternion[3]};

            // Velocity is x, y, z, wx, wy, wz
            object_msg.velocity = {object.linear_velocities[0], object.linear_velocities[1], object.linear_velocities[2],
                                   object.angular_velocities[0], object.angular_velocities[1], object.angular_velocities[2]};

            scene_msg.objects.push_back(object_msg);
        }

        // Publish message
        mujoco_state_updater.scene_pub.publish(scene_msg);
    }
}

MuJoCoStateUpdater::MuJoCoStateUpdater(int argc, char **argv, std::vector<std::string> optitrack_topic_names){

    ros::init(argc, argv, "MuJoCo_node");

    n = new ros::NodeHandle();
    listener = new tf::TransformListener();

    joint_states_sub = n->subscribe("joint_states", 10, &MuJoCoStateUpdater::JointStates_callback, this);
//    franka_states_sub = n->subscribe("/franka_state_controller/franka_states", 10, &MuJoCoStateUpdater::FrankaStates_callback, this);
    robot_base_sub = n->subscribe("/mocap/rigid_bodies/pandaRobot/pose", 10, &MuJoCoStateUpdater::RobotBasePose_callback, this);

    for(int i = 0; i < optitrack_topic_names.size(); i++){
        auto callback = std::bind(&MuJoCoStateUpdater::OptiTrack_callback, this, std::placeholders::_1, optitrack_topic_names[i]);
        std::string topic = "/mocap/rigid_bodies/" + optitrack_topic_names[i] + "/pose";
        optitrack_sub.push_back(n->subscribe<geometry_msgs::PoseStamped>(topic, 1, callback));
    }

    // Create scene message publisher
    scene_pub = n->advertise<MuJoCo_StateUpdater::Scene>("scene_state", 10);

    number_of_objects = optitrack_topic_names.size();
    optitrack_objects = optitrack_topic_names;

//    torque_pub = new ros::Publisher(n->advertise<std_msgs::Float64MultiArray>("/effort_group_effort_controller/command", 1));
//    position_pub = new ros::Publisher(n->advertise<std_msgs::Float64MultiArray>("/effort_group_position_controller/command", 1));
//    velocity_pub = new ros::Publisher(n->advertise<std_msgs::Float64MultiArray>("/effort_velocity_controller/command", 1));

    for(int i = 0; i < optitrack_topic_names.size(); i++){
        objectTrackingList.push_back(object_tracking());
        objectPoseList.push_back(pose());

        objectTrackingList[i].parent_id = "/panda_link0";
        objectTrackingList[i].target_id = "/ar_marker_3";
        objectTrackingList[i].mujoco_name = optitrack_topic_names[i];
        optitrack_objects_found.push_back(false);
    }

    // TODO - when class is instantied, have it check what controllers are running and keep track of it
    current_controller = "position_joint_trajectory_controller";

    object_callback_called = false;

}

int MuJoCoStateUpdater::GetObjectID(std::string item_name){
    for(int i = 0; i < optitrack_objects.size(); i++){
        if(item_name == optitrack_objects[i]){
            return i;
        }
    }
    // If object Id is not found, this will crash the program!
    return -1;
}

// order of poses is {x, y, z, w, wx, wy, wz}
void MuJoCoStateUpdater::OptiTrack_callback(const geometry_msgs::PoseStamped::ConstPtr &msg,
                                            const std::string &topic_name) {
    pose objectPose;
    int objectId = GetObjectID(topic_name);

//    std::cout << "Robot base: " << robotBase(0) << " " << robotBase(1) << " " << robotBase(2) << "\n";

    // For coordinate frames in mujoco, offset the object coordinates by the base frame of robot
    // as that is origin in mujoco
    objectPoseList[objectId].position.x = msg->pose.position.x - robotBase.x;
    objectPoseList[objectId].position.y = msg->pose.position.y - robotBase.y;
    objectPoseList[objectId].position.z = msg->pose.position.z - robotBase.z;

    objectPoseList[objectId].quaternion[0] = msg->pose.orientation.w;
    objectPoseList[objectId].quaternion[1] = msg->pose.orientation.x;
    objectPoseList[objectId].quaternion[2] = msg->pose.orientation.y;
    objectPoseList[objectId].quaternion[3] = msg->pose.orientation.z;

    optitrack_objects_found[objectId] = true;
}

MuJoCoStateUpdater::~MuJoCoStateUpdater(){
    delete n;
    delete listener;
}

void MuJoCoStateUpdater::JointStates_callback(const sensor_msgs::JointState &msg){
    
    // TODO - make this dependant on size of msg?
    for(int i = 0; i < NUM_JOINTS; i++){
        joint_positions[i] = msg.position[i];
        joint_velocities[i] = msg.velocity[i];
    }
}

//void MuJoCoStateUpdater::FrankaStates_callback(const franka_msgs::FrankaState &msg){
//
//    for(int i = 0; i < NUM_JOINTS; i++){
//        joint_velocities[i] = msg.dq[i];
//    }
//}

void MuJoCoStateUpdater::RobotBasePose_callback(const geometry_msgs::PoseStamped &msg){
    robotBase.x = msg.pose.position.x;
    robotBase.y = msg.pose.position.y;
    robotBase.z = msg.pose.position.z;
//
//    robotBase(3) = msg.pose.orientation.w;
//    robotBase(4) = msg.pose.orientation.x;
//    robotBase(5) = msg.pose.orientation.y;
//    robotBase(6) = msg.pose.orientation.z;
}

scene_state MuJoCoStateUpdater::ReturnScene(){
    scene_state world;

    ros::spinOnce();
    std::vector<robot_real> robots = ReturnRobotState();
    std::vector<object_real> objects = ReturnObjectsState();

    world.robots = robots;
    world.objects = objects;

    // Setting up a callback flag so you can pause execution until all objects found
    if(!object_callback_called){
        bool allobjectsFound = true;
        for(int i = 0; i < optitrack_objects_found.size(); i++){
            if(!optitrack_objects_found[i]){
                allobjectsFound = false;
            }
        }
        if(allobjectsFound){
            object_callback_called = true;
        }
    }

    return world;
}

// TODO - should probably add adaptibility for multiple robots, which should be specified by constructor call
std::vector<robot_real> MuJoCoStateUpdater::ReturnRobotState() {

    std::vector<robot_real> robots;
    robots.push_back(robot_real());
    robots[0].name = "panda";

    // Here are some specific joint offsets between franka model in MuJoCo and actual joints on real robot
    for(int i = 0; i < NUM_JOINTS; i++){
        if(i == 5){
            robots[0].joint_positions.push_back(joint_positions[i] - PI / 2);
        }
        else if(i == 6){
            robots[0].joint_positions.push_back(joint_positions[i] - PI / 4);
        }
        else{
            robots[0].joint_positions.push_back(joint_positions[i]);
        }
        robots[0].joint_velocities.push_back(joint_velocities[i]);
    }

    return robots;
}

std::vector<object_real> MuJoCoStateUpdater::ReturnObjectsState(){
    tf::StampedTransform transform;
    std::vector<object_real> objects;


    for(int i = 0; i < number_of_objects; i++){
        objects.push_back(object_real());
        objects[i].name = optitrack_objects[i];
        // x, y, z (THIS LOOKS WEIRD, THERE IS A REASON FOR THIS WEIRD SWAPPING)
        // Swapping between optitrack frame and MuJoCo frame I believe....
        objects[i].positions[0] = objectPoseList[i].position.x;
        objects[i].positions[1] = -objectPoseList[i].position.z;
        objects[i].positions[2] = objectPoseList[i].position.y;

        objects[i].linear_velocities[0] = 0.0;
        objects[i].linear_velocities[1] = 0.0;
        objects[i].linear_velocities[2] = 0.0;

        // This seems so random, Optitrack and mujoco conversion is weird...
        // x, y, z, w  (THIS LOOKS WEIRD, THERE IS A REASON FOR THIS WEIRD SWAPPING)
        // Swapping between optitrack frame and MuJoCo frame I believe....
        objects[i].quaternion[0] = objectPoseList[i].quaternion[1];
        objects[i].quaternion[1] = -objectPoseList[i].quaternion[3];
        objects[i].quaternion[2] = objectPoseList[i].quaternion[2];
        objects[i].quaternion[3] = objectPoseList[i].quaternion[0];

        objects[i].angular_velocities[0] = 0.0;
        objects[i].angular_velocities[1] = 0.0;
        objects[i].angular_velocities[2] = 0.0;
    }

//        for (int i = 0; i < number_of_objects; i++) {
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
//bool MuJoCoStateUpdater::SwitchController(std::string controllerName){
//    ros::ServiceClient load_controller = n->serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
//
//    controller_manager_msgs::LoadController load_controller_req;
//    load_controller_req.request.name = controllerName;
//    load_controller.call(load_controller_req);
//
//    ros::ServiceClient switch_controller = n->serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
//
//    std::vector<std::string> start_controller;
//    start_controller.push_back(controllerName);
//    std::vector<std::string> stop_controller;
//    stop_controller.push_back("position_joint_trajectory_controller");
//    controller_manager_msgs::SwitchController switch_controller_req;
//    switch_controller_req.request.start_controllers = start_controller;
//    switch_controller_req.request.stop_controllers = stop_controller;
//    switch_controller_req.request.strictness = 1;
//    switch_controller_req.request.start_asap = false;
//    ros::service::waitForService("/controller_manager/switch_controller", ros::Duration(5));
//    switch_controller.call(switch_controller_req);
//    if (switch_controller_req.response.ok){
//        ROS_INFO_STREAM("Controller switch correctly");
//    }
//    else{
//        ROS_ERROR_STREAM("Error occured trying to switch controller");
//        return 0;
//    }
//
//    return switch_controller_req.response.ok;
//}

//void MuJoCoStateUpdater::sendTorquesToRealRobot(double torques[]){
//    std_msgs::Float64MultiArray  desired_torques;
//    double jointSpeedLimits[NUM_JOINTS] = {0.7, 0.7, 0.7, 0.7, 1.5, 1.5, 1.5};
//    bool jointVelsSafe = true;
//    std::cout << "sending torques \n";
//
//    if(!halt_robot){
//        std::cout << "torques Sent: " << torques[0] << ", " << torques[1] << ", " << torques[2] << ", " << torques[3] << ", " << torques[4] << ", " << torques[5] << ", " << torques[6] << ", " << std::endl;
//        for(int i = 0; i < NUM_JOINTS; i++){
//            double safeTorque = torques[i];
//
//            if(joint_velocities[i] > jointSpeedLimits[i]){
//                std::cout << "joint " << i << " speed: " << joint_velocities[i] << std::endl;
//                std::cout << "safety vel triggered" << std::endl;
//                halt_robot = true;
//                safeTorque = 0.0;
//            }
//
//            if(joint_velocities[i] < -jointSpeedLimits[i]){
//                std::cout << "joint " << i << " speed: " << joint_velocities[i] << std::endl;
//                std::cout << "safety vel triggered" << std::endl;
//                halt_robot = true;
//                safeTorque = 0.0;
//            }
//
//            desired_torques.data.push_back(safeTorque);
//        }
//
//        torque_pub->publish(desired_torques);
//    }
//    else{
//        for(int i = 0; i < NUM_JOINTS; i++){
//            desired_torques.data.push_back(0.0);
//        }
//        torque_pub->publish(desired_torques);
//    }
//}

//void MuJoCoStateUpdater::sendPositionsToRealRobot(double positions[]){
//    std_msgs::Float64MultiArray  desired_positions;
//    double jointSpeedLimits[NUM_JOINTS] = {0.7, 0.7, 0.7, 0.7, 1.5, 1.5, 1.5};
//    //double torqueLimits[NUM_JOINTS] = {10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0};
//
//    if(!halt_robot){
//        //std::cout << "torques Sent: " << torques[0] << ", " << torques[1] << ", " << torques[2] << ", " << torques[3] << ", " << torques[4] << ", " << torques[5] << ", " << torques[6] << ", " << std::endl;
//        for(int i = 0; i < NUM_JOINTS; i++){
//
//            if(joint_velocities[i] > jointSpeedLimits[i]){
//                std::cout << "safety vel triggered" << std::endl;
//                std::cout << "joint " << i << " speed: " << joint_velocities[i] << std::endl;
//                std::cout << "joints at safety trigger: " << joint_positions[i] << std::endl;
//                halt_robot = true;
//            }
//
//            if(joint_velocities[i] < -jointSpeedLimits[i]){
//                std::cout << "safety vel triggered" << std::endl;
//                std::cout << "joint " << i << " speed: " << joint_velocities[i] << std::endl;
//                std::cout << "joints at safety trigger: " << joint_positions[i] << std::endl;
//                halt_robot = true;
//            }
//
//            desired_positions.data.push_back(positions[i]);
//
//        }
//        position_pub->publish(desired_positions);
//    }
//    else{
//        // dont publish anything
//
//    }
//
//    ros::spinOnce();
//}

//void MuJoCoStateUpdater::sendVelocitiesToRealRobot(double velocities[]){
//    std_msgs::Float64MultiArray  desired_velocities;
//    double jointSpeedLimits[NUM_JOINTS] = {0.7, 0.7, 0.7, 0.7, 1.5, 1.5, 1.5};
//
//    if(!halt_robot){
//        //std::cout << "torques Sent: " << torques[0] << ", " << torques[1] << ", " << torques[2] << ", " << torques[3] << ", " << torques[4] << ", " << torques[5] << ", " << torques[6] << ", " << std::endl;
//        for(int i = 0; i < NUM_JOINTS; i++){
//
//            if(joint_velocities[i] > jointSpeedLimits[i]){
//                std::cout << "safety vel triggered" << std::endl;
//                std::cout << "joint " << i << " speed: " << joint_velocities[i] << std::endl;
//                std::cout << "joints at safety trigger: " << joint_positions[i] << std::endl;
//                halt_robot = true;
//            }
//
//            if(joint_velocities[i] < -jointSpeedLimits[i]){
//                std::cout << "safety vel triggered" << std::endl;
//                std::cout << "joint " << i << " speed: " << joint_velocities[i] << std::endl;
//                std::cout << "joints at safety trigger: " << joint_positions[i] << std::endl;
//                halt_robot = true;
//            }
//
//            desired_velocities.data.push_back(velocities[i]);
//
//        }
//        velocity_pub->publish(desired_velocities);
//    }
//    else{
//        // dont publish anything
//
//    }
//
//    ros::spinOnce();
//}

// void MuJoCoStateUpdater::resetTorqueControl(){
//     halt_robot = false;
// }
