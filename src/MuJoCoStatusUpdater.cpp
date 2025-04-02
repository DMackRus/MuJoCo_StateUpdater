#include "MuJoCoStatusUpdater.h"

int main(int argc, char **argv){

    // Create an instance of MuJocoStatusUpdater
    MuJoCoStateUpdater mujoco_state_updater(argc, argv);

    ros::Rate rate(10000); // Loop rate in Hz
    while(ros::ok()){
        ros::spinOnce();
        // create message
        MuJoCo_StateUpdater::Scene scene_msg;

        // fill in message
        for(auto robot : mujoco_state_updater.robots){
            MuJoCo_StateUpdater::Robot robot_msg;
            robot_msg.name = robot.name;

            for(int i = 0; i < NUM_JOINTS; i++){
                robot_msg.joint_positions.push_back(robot.joint_positions[i]);
                robot_msg.joint_velocities.push_back(robot.joint_velocities[i]);
                robot_msg.joint_efforts.push_back(robot.joint_efforts[i]);
            }
            scene_msg.robots.push_back(robot_msg);
        }

        // Iterate through dictionary keys for tracked objects
        for (const auto& [key, value] : mujoco_state_updater.tracked_object_poses) {
            MuJoCo_StateUpdater::RigidBody object_msg;

            object_msg.name = value.name;

            if(value.confidence < 0.05){
                // Pose is x, y, z, qx, qy, qz, w
                object_msg.pose = {0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0};

                // Velocity is x, y, z, wx, wy, wz
                object_msg.velocity = {0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0};

                object_msg.confidence = value.confidence;

            }
            else{
                // Pose is x, y, z, qx, qy, qz, w
                object_msg.pose = {value.positions[0], value.positions[1], value.positions[2],
                                   value.quaternion[0], value.quaternion[1], value.quaternion[2], value.quaternion[3]};

                // Velocity is x, y, z, wx, wy, wz
                object_msg.velocity = {value.linear_velocities[0], value.linear_velocities[1], value.linear_velocities[2],
                                       value.angular_velocities[0], value.angular_velocities[1], value.angular_velocities[2]};

                object_msg.confidence = value.confidence;
            }

            scene_msg.objects.push_back(object_msg);
        }

        // Publish scene message
        mujoco_state_updater.scene_pub.publish(scene_msg);

        ros::spinOnce(); // Process callbacks
        rate.sleep();    // Maintain loop rate
    }
}

MuJoCoStateUpdater::MuJoCoStateUpdater(int argc, char **argv){

    ros::init(argc, argv, "MuJoCo_node");

    n = new ros::NodeHandle();

    // Get the list of objects to track from ROS param server
    std::vector<std::string> tracked_objects;
    if (n->getParam("tracked_objects", tracked_objects)) {
        ROS_INFO("Loaded tracked objects vector:");
        for (const auto& str : tracked_objects) {
            ROS_INFO("  - %s", str.c_str());
        }
    } else {
        ROS_WARN("Failed to get parameter 'tracked_objects'");
    }

    joint_states_sub = n->subscribe("joint_states", 10, &MuJoCoStateUpdater::JointStates_callback, this);
    robot_base_sub = n->subscribe("/mocap/rigid_bodies/pandaRobot/pose", 10, &MuJoCoStateUpdater::RobotBasePose_callback, this);

    for(int i = 0; i < tracked_objects.size(); i++){
        auto callback = std::bind(&MuJoCoStateUpdater::OptiTrack_callback, this, std::placeholders::_1, tracked_objects[i]);
        std::string topic = "/mocap/rigid_bodies/" + tracked_objects[i] + "/pose";
        optitrack_sub.push_back(n->subscribe<geometry_msgs::PoseStamped>(topic, 1, callback));
    }

    // Curently assume we only have one real robot - with 7 joints (AKA Panda)
    robots.push_back(robot_real());
    for(int i = 0; i < NUM_JOINTS; i++){
        robots[0].joint_positions.push_back(0.0);
        robots[0].joint_velocities.push_back(0.0);
        robots[0].joint_efforts.push_back(0.0);
    }

    // Create scene message publisher
    scene_pub = n->advertise<MuJoCo_StateUpdater::Scene>("scene_state", 10);

    // Populate the dictionary
    for(auto & tracked_object : tracked_objects){

        tracked_object_poses[tracked_object] = object_real
                {tracked_object, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
                 {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
                 {0.0, 0.0, 0.0, 0.0},
                 {0.0, 0.0, 0.0}, 0.0};

    }
}

// order of poses is {x, y, z, w, wx, wy, wz}
void MuJoCoStateUpdater::OptiTrack_callback(const geometry_msgs::PoseStamped::ConstPtr &msg,
                                            const std::string &topic_name) {

    tracked_object_poses[topic_name].name = topic_name;

    // Store previous raw positions
    tracked_object_poses[topic_name].previous_raw_positions[0] = tracked_object_poses[topic_name].raw_positions[0];
    tracked_object_poses[topic_name].previous_raw_positions[1] = tracked_object_poses[topic_name].raw_positions[1];
    tracked_object_poses[topic_name].previous_raw_positions[2] = tracked_object_poses[topic_name].raw_positions[2];

    tracked_object_poses[topic_name].raw_positions[0] = msg->pose.position.x;
    tracked_object_poses[topic_name].raw_positions[1] = msg->pose.position.y;
    tracked_object_poses[topic_name].raw_positions[2] = msg->pose.position.z;

    // For coordinate frames in MuJoCo, offset the object coordinates by the base frame of robot
    // as that is origin in MuJoCo
    // NOTE - THIS LOOKS WEIRD AND THAT IS BECAUSE IT IS. Trying to swap between real world optitrack frame
    // and MuJoCo frame. Possibly this should not be done here and be done on the user side instead????
    tracked_object_poses[topic_name].positions[0] = msg->pose.position.x - robotBase.x;
    tracked_object_poses[topic_name].positions[1] = -(msg->pose.position.z - robotBase.z);
    tracked_object_poses[topic_name].positions[2] = msg->pose.position.y - robotBase.y;

    tracked_object_poses[topic_name].quaternion[0] = msg->pose.orientation.x;
    tracked_object_poses[topic_name].quaternion[1] = -msg->pose.orientation.z;
    tracked_object_poses[topic_name].quaternion[2] = msg->pose.orientation.y;
    tracked_object_poses[topic_name].quaternion[3] = msg->pose.orientation.w;

    // TODO - add velocity tracking
    tracked_object_poses[topic_name].linear_velocities[0] = 0.0;
    tracked_object_poses[topic_name].linear_velocities[1] = 0.0;
    tracked_object_poses[topic_name].linear_velocities[2] = 0.0;

    tracked_object_poses[topic_name].angular_velocities[0] = 0.0;
    tracked_object_poses[topic_name].angular_velocities[1] = 0.0;
    tracked_object_poses[topic_name].angular_velocities[2] = 0.0;

    // Do a custom check specific to optitrack to check how confident we are
    // Basically, when optitrack is working well, the data should be "flickering" on the lower
    // bits of precisions (0.0001). If it has lost the object the data will remain exactly the same.
    double diff = abs(tracked_object_poses[topic_name].raw_positions[0] -
            tracked_object_poses[topic_name].previous_raw_positions[0]);

    if (diff < 0.000001){
        tracked_object_poses[topic_name].confidence = 0.0;
    }
    else{
        tracked_object_poses[topic_name].confidence = 1.0;
    }
}

MuJoCoStateUpdater::~MuJoCoStateUpdater(){
    delete n;
}

void MuJoCoStateUpdater::JointStates_callback(const sensor_msgs::JointState &msg){

    robots[0].name = "panda";
    for(int i = 0; i < NUM_JOINTS; i++){
        robots[0].joint_positions[i] = msg.position[i];
        robots[0].joint_velocities[i] = msg.velocity[i];
        robots[0].joint_efforts[i] = msg.effort[i];
    }
}

void MuJoCoStateUpdater::RobotBasePose_callback(const geometry_msgs::PoseStamped &msg){
    robotBase.x = msg.pose.position.x;
    robotBase.y = msg.pose.position.y;
    robotBase.z = msg.pose.position.z;
}