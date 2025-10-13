#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

// Global variables for ROS publishers and services
ros::Publisher scene_publisher;
ros::ServiceClient scene_client;
ros::ServiceClient attach_client;
ros::ServiceClient detach_client;
ros::Publisher pub;
ros::Subscriber robot_state_sub;
std::vector<moveit_msgs::Constraints> groups_constraints;

// Placeholder function for robot state callback
void robotStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // This function will be used if necessary
}

// Function to close the gripper
void close_gripper(moveit::planning_interface::MoveGroupInterface& group) {
    std::vector<double> joint_values = {-0.035, 0.035};  // Adjust as needed
    group.setJointValueTarget(joint_values);

    if (group.move()) {
        ROS_INFO("Gripper closed successfully!");
    } else {
        ROS_ERROR("Failed to close the gripper!");
    }
}

int main(int argc, char **argv) { 
    ros::init(argc, argv, "close_gripper_node");
    ros::NodeHandle nh;

    // Initialize the Planning Scene
    moveit::planning_interface::PlanningSceneInterface planning_scene;

    // Advertise and subscribe to necessary topics
    scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);    
    scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
    attach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/attach");
    detach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/detach");
    pub = nh.advertise<std_msgs::Int32>("action_completed", 1);
    robot_state_sub = nh.subscribe("/kr20_kr120/joint_states", 1000, robotStateCallback);
    ros::Rate rate(1); 

    // Start MoveIt spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(2.0);

    // Set constraints (ensure this function is defined)
    // If undefined, you can comment this line
    // set_constraints(groups_constraints);

    // Initialize MoveIt Planning Groups
    moveit::planning_interface::MoveGroupInterface gripper_group("kr120_gripper");

    // Wait for a valid robot state
    while (ros::ok() && gripper_group.getCurrentJointValues().empty()) {
        ROS_WARN("Waiting for valid robot state...");
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("Received valid robot state!");

    // Close the gripper
    close_gripper(gripper_group);

    ros::shutdown();
    return 0;
}



// #include "exec/pick.h"
// #include <ros/ros.h>
// #include <moveit/planning_interface/MoveGroupInterface.h>
// #include <moveit/planning_interface/PlanningSceneInterface.h>
// #include <geometry_msgs/Pose.h>

// /**
//  * @brief Test node to call the `pick` function.
//  */
// int main(int argc, char** argv) {
//     // Initialize ROS node
//     ros::init(argc, argv, "test_pick_node");
//     ros::NodeHandle nh;

//     // MoveIt interfaces
//     moveit::planning_interface::MoveGroupInterface group("manipulator");
//     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
//     // Define a test pick pose
//     geometry_msgs::Pose pick_pose;
//     pick_pose.position.x = 0.4;
//     pick_pose.position.y = 0.0;
//     pick_pose.position.z = 0.5;
//     pick_pose.orientation.w = 1.0;

//     // Collision objects (empty for now)
//     std::vector<moveit_msgs::CollisionObject> tray_components;

//     // Service client to get planning scene
//     ros::ServiceClient get_scene = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

//     // Publisher for updating the scene
//     ros::Publisher scene_pub = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

//     // Call the pick function
//     ROS_INFO("Calling pick function...");
//     pick("test_tray", tray_components, 0, group, pick_pose, get_scene, scene_pub);

//     ROS_INFO("Pick function executed.");
//     return 0;
// }
