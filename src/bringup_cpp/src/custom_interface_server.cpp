#include <ros/ros.h>
#include <custom_interfaces/BringTray.h>
#include <custom_interfaces/PickPlace.h>
#include <custom_interfaces/Screw.h>
#include <custom_interfaces/MoveTray.h>

// Function prototypes (implementations should be in corresponding `.cpp` files)
bool bring_tray_function(const std::string& tray_name, const std::string& location);
bool pick_place_function(const std::string& object_name, const geometry_msgs::Pose& target_pose);
bool screw_function(const std::string& screw_name, const geometry_msgs::Pose& screw_pose);
bool move_tray_function(const std::string& tray_name, const std::string& next_screw_position);

// Callback for BringTray service
bool bring_tray(custom_interfaces::BringTray::Request &req,
                custom_interfaces::BringTray::Response &res) {
    ROS_INFO("Bringing tray %s to %s", req.tray_name.c_str(), req.location.c_str());
    res.success = bring_tray_function(req.tray_name, req.location);
    return true;
}

// Callback for PickPlace service
bool pick_place(custom_interfaces::PickPlace::Request &req,
                custom_interfaces::PickPlace::Response &res) {
    ROS_INFO("Picking %s and placing at target pose", req.object_name.c_str());
    res.success = pick_place_function(req.object_name, req.target_pose.pose);
    return true;
}

// Callback for Screw service
bool screw(custom_interfaces::Screw::Request &req,
           custom_interfaces::Screw::Response &res) {
    ROS_INFO("Screwing %s at given pose", req.screw_name.c_str());
    res.success = screw_function(req.screw_name, req.screw_pose.pose);
    return true;
}

// Callback for MoveTray service
bool move_tray(custom_interfaces::MoveTray::Request &req,
               custom_interfaces::MoveTray::Response &res) {
    ROS_INFO("Moving tray %s to next position %s", req.tray_name.c_str(), req.next_screw_position.c_str());
    res.success = move_tray_function(req.tray_name, req.next_screw_position);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_control_server");
    ros::NodeHandle nh;

    // Advertise services
    ros::ServiceServer bring_tray_srv = nh.advertiseService("bring_tray", bring_tray);
    ros::ServiceServer pick_place_srv = nh.advertiseService("pick_place", pick_place);
    ros::ServiceServer screw_srv = nh.advertiseService("screw", screw);
    ros::ServiceServer move_tray_srv = nh.advertiseService("move_tray", move_tray);

    ROS_INFO("Robot Control Server is running...");
    ros::spin(); // Keeps node running

    return 0;
}
