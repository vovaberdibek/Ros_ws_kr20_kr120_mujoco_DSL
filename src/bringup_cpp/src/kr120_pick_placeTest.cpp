#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometric_shapes/shape_operations.h"
#include <fstream>
#include <std_msgs/Int32.h>
#include <cmath>
#include <gazebo_ros_link_attacher/Attach.h>
#include <actionlib/server/simple_action_server.h>
#include "custom_interfaces/ExecuteSequenceAction.h"
#include "custom_interfaces/PositionTray.h"
#include "custom_interfaces/AddTray2.h"
#include "custom_interfaces/PickTray.h"
#include "custom_interfaces/OperatorPositionTray.h"
#include "custom_interfaces/RechargeSequence.h"
#include "custom_interfaces/InternalScrewingSequence.h"
#include "custom_interfaces/ScrewingUnit.h"
#include "custom_interfaces/ExternalScrewingSequence.h"
#include "custom_interfaces/PlaceTray.h"
#include "custom_interfaces/InitParameters.h"
#include <custom_interfaces/InternalScrewByNum.h>
#include <sstream>
#include <gazebo_msgs/SpawnModel.h>
#include <map>
#include <vector>
#include <string>
#include "ros/ros.h"
#include <iostream>




const double pi=3.141592653589793;
int start_index=0; // Initial sequence step
int n_screw;        //testing screwing func for single screw
double tray_angle1; // angle wrt z axis for tray positioning
double tray_angle2; // angle wrt z axis for tray positioning
double height_tray; //aoc eps tray height
double height_dhc; //dhc tray height 
double height_payload; //payload tray height 
double orig_bott_dist_aoc; //distance between origin of RF and the bottom of the tray
double orig_bott_dist_dhc=orig_bott_dist_aoc+height_tray;  //distance between origin of RF and the bottom of the tray
double orig_bott_dist_eps=orig_bott_dist_dhc+height_dhc; //distance between origin of RF and the bottom of the tray
double orig_bott_dist_payload=orig_bott_dist_eps+height_tray; //distance between origin of RF and the bottom of the tray
double displ_aoc=2*orig_bott_dist_aoc+height_tray; //distance between the positions of RF origin for tray up and down
double displ_eps=2*orig_bott_dist_aoc+2*height_tray+height_dhc; //distance between the positions of RF origin for tray up and down

std::string file_pose;
std::string file_order;
std::vector< geometry_msgs::Pose > target_poses;   
std::unordered_map<int,geometry_msgs::Pose> hole_pose_map;

std::vector<std::vector<int>> target_order;

std::vector<std::string> tray_unit_names;
std::vector<int> tray_unit_pose_indices;
std::vector<int> screw_quantities;
std::vector<std::string> screw_types;


moveit::planning_interface::MoveGroupInterface::Plan plan;
std::vector<moveit_msgs::Constraints> groups_constraints;

ros::ServiceClient spawn_client;
ros::Publisher scene_publisher;
ros::ServiceClient scene_client;
ros::ServiceClient attach_client;
ros::ServiceClient detach_client;
ros::Publisher pub;
ros::Subscriber robot_state_sub;

std::string tray_name;
std::string object_name;
std::vector<moveit_msgs::CollisionObject> tray_components_aoc;
std::vector<moveit_msgs::CollisionObject> tray_components_eps;
std::vector<moveit_msgs::CollisionObject> tray_dhc;
std::vector<moveit_msgs::CollisionObject> tray_payload;
std::vector<moveit_msgs::CollisionObject> bottom_panel;
std::vector<moveit_msgs::CollisionObject> collision_objects;

std::map<std::string,std::vector<moveit_msgs::CollisionObject>> tray_map;

geometry_msgs::Pose approach_pose;
geometry_msgs::Pose screw_pose;
geometry_msgs::Pose table_pose;
geometry_msgs::Pose tray_init_pose;
geometry_msgs::Pose recharge_pose;
geometry_msgs::Pose rest_pose;
geometry_msgs::Pose pick_pose;
geometry_msgs::Pose pre_pick_pose;
geometry_msgs::Pose post_pick_pose;
geometry_msgs::Pose end_pose;
geometry_msgs::Pose pre_end_pose;
geometry_msgs::Pose bottom_pick_pose;
geometry_msgs::Pose tray_pose;
geometry_msgs::Pose pre_recharge;
geometry_msgs::Pose bottom_init_pose;
std::array<double, 6> ext_screw_tray_pose;
std::array<double, 6> tray_final_pose; 
std::array<double, 6> tray_pose_operator;
std::array<std::array<double, 6>, 16> value_tray_pose;
std::array<double, 3> tip_position;

std::vector<int> new_tray_indeces; // First index of a tray
std::vector<int> operator_indeces; // When the tray need to be placed to add other dummies
std::vector<int> horiz_rotation_indeces;
std::vector<int> change_pose_indeces; // When the tray need to be moved to another pose 
std::vector<int> tray_down_indeces; 

std::vector<std::string> gyro_joints={"kr120_gyro_right_joint","kr120_gyro_left_joint"};
std::vector<std::string> plier_joints={"kr120_pinza_right_joint","kr120_pinza_left_joint"};
    
std_msgs::Int32 spawn_msg;
int recharge_index;
geometry_msgs::Pose kr120_target_pose;
geometry_msgs::Pose kr20_target_pose;
int target_index;
int target_order_index;
int count_components;
std::array<double,3> tray_flange_dist_pos;
std::array<double,3> tray_flange_dist_op;
double gyro_target;
double tray_flange_angle_pos;
double tray_flange_angle_op;
gazebo_ros_link_attacher::Attach attach_srv;
std::string new_dummy;

// A single screw entry:
struct Screw {
  std::string mode;         // if you still use it elsewhere
  int         index;
  geometry_msgs::Pose pose;
  std::string unit_name;    // <— new member!
};


// using json = nlohmann::json;

// using ScrewConfig = std::map<std::string, std::vector<Screw>>;

// ScrewConfig g_screw_cfg;  

// assume this used to take a filename:
// std::map<std::string, std::vector<Screw>> loadScrewConfigFromFile(const std::string& path);


geometry_msgs::Quaternion rpy_to_quatExtra(double roll, double pitch, double yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion q_msg;
    tf2::convert(q, q_msg);
    return q_msg;
}

// Load the JSON file into memory
// std::map<std::string, std::vector<Screw>> loadScrewConfigFromString(const std::string& txt) {
//     auto j = json::parse(txt);
//     ScrewConfig cfg;
//     for (auto& [tray, arr] : j.items()) {
//         for (auto& el : arr) {
//             Screw s;
//             s.unit_name = tray;
//             s.index     = el.at("index").get<int>();
//             s.mode      = el.at("mode").get<std::string>();
//             auto& p     = el.at("pose");
//             // convert mm → m
//             s.pose.position.x = p.at("x").get<double>() / 1000.0;
//             s.pose.position.y = p.at("y").get<double>() / 1000.0;
//             s.pose.position.z = p.at("z").get<double>() / 1000.0;
//             // roll/pitch/yaw → quaternion
//             s.pose.orientation = rpy_to_quatExtra(
//                 p.at("roll").get<double>(),
//                 p.at("pitch").get<double>(),
//                 p.at("yaw").get<double>()
//             );
//             cfg[tray].push_back(s);
//         }
//     }
//     return cfg;
// }

// ScrewConfig loadScrewConfig(const std::string& path) {
//   std::ifstream in(path);
//   if (!in) throw std::runtime_error("Cannot open "+path);
//   json j; in >> j;

//   ScrewConfig cfg;
//   for (auto& [objName, arr] : j.items()) {
//     for (auto& el : arr) {
//       Screw s;
//       s.unit_name = objName;    
//       s.index = el.at("index").get<int>();
//       s.mode  = el.at("mode").get<std::string>();
//       auto& p = el.at("pose");
//       // convert mm→m
//       s.pose.position.x = p.at("x").get<double>() / 1000.0;
//       s.pose.position.y = p.at("y").get<double>() / 1000.0;
//       s.pose.position.z = p.at("z").get<double>() / 1000.0;
//       // roll/pitch/yaw → quaternion
//       s.pose.orientation = rpy_to_quatExtra(
//         p.at("roll").get<double>(),
//         p.at("pitch").get<double>(),
//         p.at("yaw").get<double>()
//       );
//       cfg[objName].push_back(s);
//     }
//   }
//   return cfg;
// }

// hole_pose_map.clear();
// for (auto & [tray, screws] : g_screw_cfg) {
//   for (auto & s : screws) {
//     hole_pose_map[s.index] = s.pose;    // key = the "index" from JSON
//   }
// }



struct Unit {
    std::string name;
    int pose_index;
};

// std::vector<Unit> current_tray_units = {
//     {"MTQ12", 0},
//     {"MTQ3_MAG", 1},
//     {"CMG2", 2},
//     {"CMG1", 4},
//     {"CMG34", 6}
// };

// std::vector<std::string> tray_down_steps = {"MTQ12", "MTQ3_MAG", "CMG2", "CMG1", "PCDU"};

// Global tray info received from Python DSL
std::vector<Unit> current_tray_units;
std::vector<std::string> tray_down_steps;



int get_pose_index_from_unit_name(const std::vector<Unit>& units, const std::string& name) {
    for (const auto& unit : units) {
        if (unit.name == name)
            return unit.pose_index;
    }
    return -1;  // not found
}


// geometry_msgs::Quaternion rpy_to_quatExtra(double roll, double pitch, double yaw) {
//     tf2::Quaternion q;
//     q.setRPY(roll, pitch, yaw);
//     q.normalize();  
//     geometry_msgs::Quaternion q_msg;
//     tf2::convert(q, q_msg);
//     return q_msg;
// }

geometry_msgs::Quaternion make_quat_pitch_90deg() {
    geometry_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 0.707107;
    q.z = 0.0;
    q.w = 0.707107;
    return q;
}



geometry_msgs::Pose pose_from_array(const boost::array<double, 6> &arr) {
    geometry_msgs::Pose pose;
    pose.position.x = arr[0];
    pose.position.y = arr[1];
    pose.position.z = arr[2];
    pose.orientation = rpy_to_quatExtra(arr[3], arr[4], arr[5]);
    return pose;
}

std::string to_string(const std::vector<double>& vec) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        oss << vec[i];
        if (i != vec.size() - 1) oss << ", ";
    }
    oss << "]";
    return oss.str();
}



// Check if an array contains a certain value
bool contains(std::vector<int> vec, int value) {
    for (int i = 0; i < vec.size(); i++) {
        if (vec[i] == value) {
            return true;
        }
    }
    return false;
}

// Check if a string exists in a vector<string>
bool contains2(const std::vector<std::string>& vec, const std::string& value) {
    return std::find(vec.begin(), vec.end(), value) != vec.end();
}


// Check if an array contains a certain string
bool contains_string(std::vector< std::string > vec, std::string value) {
    for (int i = 0; i < vec.size(); i++) {
        if (vec[i] == value) {
            return true;
        }
    }
    return false;
}

// Get the index of a string in a vector
int get_index(std::vector<std::string> v, std::string s){
    int index=-1;

    for (int i = 0; i < v.size(); i++){
        if(v[i]==s) index=i;
    }
    return index;
}

void spawnTrayInGazebo(const std::string &tray_name) {
    std::string tray_file = "/home/user/cella_ws/src/kr20_kr120/urdf/" + tray_name + ".sdf";
    ROS_INFO("🔄 Spawning tray '%s' from file '%s'", tray_name.c_str(), tray_file.c_str());

    std::ifstream file(tray_file);
    if (!file.is_open()) {
        ROS_ERROR("❌ Cannot open SDF file: %s", tray_file.c_str());
        return;
    }

    std::stringstream xml;
    xml << file.rdbuf();
    file.close();

    gazebo_msgs::SpawnModel spawn_srv;
    spawn_srv.request.model_name = tray_name;
    spawn_srv.request.model_xml = xml.str();

    if(spawn_client.call(spawn_srv)) {
        ROS_INFO("✅ Spawned: %s", spawn_srv.response.status_message.c_str());
    } else {
        ROS_ERROR("❌ Failed to spawn tray '%s'", tray_name.c_str());
    }
}


// Convert an orientation from Roll-Pitch-Yaw representation to quaternion representation
geometry_msgs::Quaternion rpy_to_quat(double roll, double pitch, double yaw)
{
  tf2::Quaternion quaternion_tf2;
  quaternion_tf2.setRPY(roll, pitch, yaw);
  geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
  return quaternion;
}

// Convert an orientation from quaternion representation to matrix representation
std::array<std::array<double, 3>, 3> quat_to_rotationMatrix(geometry_msgs::Quaternion quaternion) {
    std::array<std::array<double, 3>, 3> R;
    double q0=quaternion.w;
    double q1=quaternion.x;
    double q2=quaternion.y;
    double q3=quaternion.z;

    R[0][0] = 1 - 2 * (q2 * q2 + q3 * q3);
    R[0][1] = 2 * (q1 * q2 - q0 * q3);
    R[0][2] = 2 * (q1 * q3 + q0 * q2);

    R[1][0] = 2 * (q1 * q2 + q0 * q3);
    R[1][1] = 1 - 2 * (q1 * q1 + q3 * q3);
    R[1][2] = 2 * (q2 * q3 - q0 * q1);

    R[2][0] = 2 * (q1 * q3 - q0 * q2);
    R[2][1] = 2 * (q2 * q3 + q0 * q1);
    R[2][2] = 1 - 2 * (q1 * q1 + q2 * q2);

    return R;
}

// Convert an orientation from matrix representation to quaternion representation
geometry_msgs::Quaternion rotationMatrix_to_quat(const std::array<std::array<double, 3>, 3>& R) {
    geometry_msgs::Quaternion quaternion;

    double trace = R[0][0] + R[1][1] + R[2][2]; 

    if (trace > 0) {
        double s = 0.5 / sqrt(trace + 1.0);
        quaternion.w = 0.25 / s;
        quaternion.x = (R[2][1] - R[1][2]) * s;
        quaternion.y = (R[0][2] - R[2][0]) * s;
        quaternion.z = (R[1][0] - R[0][1]) * s;
    } else {
        if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
            double s = 2.0 * sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
            quaternion.w = (R[2][1] - R[1][2]) / s;
            quaternion.x = 0.25 * s;
            quaternion.y = (R[0][1] + R[1][0]) / s;
            quaternion.z = (R[0][2] + R[2][0]) / s;
        } else if (R[1][1] > R[2][2]) {
            double s = 2.0 * sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
            quaternion.w = (R[0][2] - R[2][0]) / s;
            quaternion.x = (R[0][1] + R[1][0]) / s;
            quaternion.y = 0.25 * s;
            quaternion.z = (R[1][2] + R[2][1]) / s;
        } else {
            double s = 2.0 * sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
            quaternion.w = (R[1][0] - R[0][1]) / s;
            quaternion.x = (R[0][2] + R[2][0]) / s;
            quaternion.y = (R[1][2] + R[2][1]) / s;
            quaternion.z = 0.25 * s;
        }
    }

    return quaternion;
}

// Extract position and orientation of all the holes from a text file 
std::vector< geometry_msgs::Pose > extract_poses_from_file(std::string file_name ){
    std::vector< geometry_msgs::Pose > target_poses;

    std::ifstream file(file_name);

    if (!file.is_open()) {
        std::cerr << "File is not open!" << std::endl;
        return target_poses;
    }

    std::string line;
    geometry_msgs::Pose pose;

    // Read on line at time
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;

        if(ss.str().size()>4){// To not consider dummy names
            // Divide the line using comma as token
            while (std::getline(ss, token, ',')) {
                try {
                    double value = std::stod(token);
                    values.push_back(value);
                } catch (const std::invalid_argument& e) {
                    std::cerr << "Conversion error: " << token << " number is not valid." << std::endl;
                    continue;
                } catch (const std::out_of_range& e) {
                    std::cerr << "Conversion error: " << token << " number out of double range." << std::endl;
                    continue;
                }
            }

            pose.orientation = rpy_to_quat(values[0],values[1],values[2]);
            pose.position.x = values[3]*1e-3;// Cause in the file positions are in millimeters
            pose.position.y = values[4]*1e-3;
            pose.position.z = values[5]*1e-3;
            target_poses.push_back(pose);
        }
    }

    file.close();
    return target_poses;
}

// Extract screwing sequence from a text file
std::vector<std::vector<int>> extract_order_from_file(std::string file_name ){
    std::vector<std::vector<int>> targets_order;
    std::ifstream file(file_name); 

    if (!file.is_open()) {
        std::cerr << "File is not open!" << std::endl;
        return targets_order;
    }

    std::string line;

    // Read on line at time
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        if(ss.str()=="MAN"){
            std::getline(file, line);
        }else{
            if(ss.str()=="AUTO"){
                std::getline(file, line);
            }

            std::stringstream str(line);
            std::string token;
            std::vector<int> values;

            // Divide the line using comma as token
            while (std::getline(str, token, ',')) {
                try {
                    int value = std::stod(token);
                    values.push_back(value);
                } catch (const std::invalid_argument& e) {
                    std::cerr << "Conversion error: " << token << " number is not valid." << std::endl;
                    continue;
                } catch (const std::out_of_range& e) {
                    std::cerr << "Conversion error: " << token << " number out of double range." << std::endl;
                    continue;
                }
            }
            targets_order.push_back(values);
        }
    }

    file.close();
    return targets_order;
}

// Takes as input the pose of the hole wrt the tray RF and gives as output the pose of the flange wrt the world RF
// -----------------------------------------------------------------------------
// Takes as input the index of the hole (the JSON “index”) and the tray pose
// [r,p,y, x,y,z] and returns the target flange pose in the world frame.
// -----------------------------------------------------------------------------
// new:
geometry_msgs::Pose kr20_flange_target_pose2(
    const geometry_msgs::Pose&               hole_in_tray,
    const std::array<double,6>&              tray_rpy_xyz)
{
  ROS_INFO("🔥 FLANGE_DEBUG start");

  // 1) Build tray_in_world from RPY+XYZ
  geometry_msgs::Pose tray_in_world;
  tray_in_world.orientation = rpy_to_quat(
      tray_rpy_xyz[0], tray_rpy_xyz[1], tray_rpy_xyz[2]);
  tray_in_world.position.x = tray_rpy_xyz[3];
  tray_in_world.position.y = tray_rpy_xyz[4];
  tray_in_world.position.z = tray_rpy_xyz[5];

  // 2) Rotation matrices
  auto R_tray    = quat_to_rotationMatrix(tray_in_world.orientation);
  auto R_target  = quat_to_rotationMatrix(hole_in_tray.orientation);

  // 3) Use identity for tool rotation (so flange Z == hole Z)
  std::array<std::array<double,3>,3> R_tool {{
    std::array<double,3>{{1.0, 0.0, 0.0}},
    std::array<double,3>{{0.0, 1.0, 0.0}},
    std::array<double,3>{{0.0, 0.0, 1.0}}
  }};

  // 4) R_tw = R_tray * R_target
  std::array<std::array<double,3>,3> R_tw{};
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      for(int k=0;k<3;k++){
        R_tw[i][j] += R_tray[i][k] * R_target[k][j];
      }
    }
  }

  // 5) R_fw = R_tw * R_tool  (but R_tool is identity, so R_fw == R_tw)
  std::array<std::array<double,3>,3> R_fw{};
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      for(int k=0;k<3;k++){
        R_fw[i][j] += R_tw[i][k] * R_tool[k][j];
      }
    }
  }

  // 6) Compute hole position in world
  std::array<double,3> p_hole_world {{
    tray_in_world.position.x,
    tray_in_world.position.y,
    tray_in_world.position.z
  }};
  for(int i=0;i<3;i++){
    p_hole_world[i] +=
      R_tray[i][0]*hole_in_tray.position.x +
      R_tray[i][1]*hole_in_tray.position.y +
      R_tray[i][2]*hole_in_tray.position.z;
  }

  // 7) Add your known tip offset along the flange Z
  extern std::array<double,3> tip_position;  // your [x,y,z] in tool frame
  std::array<double,3> p_flange_world = p_hole_world;
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      p_flange_world[i] += R_fw[i][j] * tip_position[j];
    }
  }

  // 8) Pack into Pose
  geometry_msgs::Pose flange;
  flange.position.x = p_flange_world[0];
  flange.position.y = p_flange_world[1];
  flange.position.z = p_flange_world[2];
  flange.orientation = rotationMatrix_to_quat(R_fw);

  // 9) Debug axes
  // hole Z-axis in world = column 2 of R_tw
  ROS_INFO_STREAM("🔍 hole Z-axis in world = ["
    << R_tw[0][2] << ", "
    << R_tw[1][2] << ", "
    << R_tw[2][2] << "]");
  // tool Z-axis in world = column 2 of R_fw
  ROS_INFO_STREAM("🔍 tool Z-axis in world = ["
    << R_fw[0][2] << ", "
    << R_fw[1][2] << ", "
    << R_fw[2][2] << "]");

  return flange;
}


geometry_msgs::Pose kr20_flange_target_pose(int index,std::array<double, 6> tray_pose){
    
    geometry_msgs::Quaternion quat_tray=rpy_to_quat(tray_pose[0],tray_pose[1],tray_pose[2]);; // Orientation of the tray wrt the world RF

    geometry_msgs::Quaternion quat_target=target_poses[index].orientation; // Orientation of the hole wrt the tray RF

    geometry_msgs::Quaternion quat_tool=rpy_to_quat(pi,pi/2,0); // Screwing orientation wrt the tool RF

    std::array<std::array<double, 3>, 3> matrix_tray=quat_to_rotationMatrix(quat_tray);
    std::array<std::array<double, 3>, 3> matrix_target=quat_to_rotationMatrix(quat_target);
    std::array<std::array<double, 3>, 3> matrix_tool=quat_to_rotationMatrix(quat_tool);

    std::array<double, 3> target_position = {target_poses[index].position.x,target_poses[index].position.y,target_poses[index].position.z};
    std::array<double, 3> flange_position = {tray_pose[3],tray_pose[4],tray_pose[5]};

    std::array<double, 3> debug = {tray_pose[3],tray_pose[4],tray_pose[5]};

    std::array<std::array<double, 3>, 3> matrix_target_world = {{{0.0, 0.0, 0.0},{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}}}; // Orientation of the target wrt the world RF
    std::array<std::array<double, 3>, 3> matrix_flange = {{{0.0, 0.0, 0.0},{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}}}; // Orientation of the flange wrt the world RF

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                matrix_target_world[i][j] += matrix_tray[i][k] * matrix_target[k][j];
            }
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                matrix_flange[i][j] +=  matrix_target_world[i][k] *matrix_tool[k][j];
            }
        }
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            debug[i] += (matrix_tray[i][j] * target_position[j]);
            flange_position[i] += (matrix_tray[i][j] * target_position[j]+ matrix_flange[i][j]*tip_position[j]);
        }
    }

    geometry_msgs::Quaternion quat_flange=rotationMatrix_to_quat(matrix_flange);

    geometry_msgs::Pose flange_target_pose;

    flange_target_pose.position.x=flange_position[0];
    flange_target_pose.position.y=flange_position[1];
    flange_target_pose.position.z=flange_position[2];
    flange_target_pose.orientation=quat_flange;

    return flange_target_pose;
}


// geometry_msgs::Pose kr20_flange_target_pose(int index,std::array<double, 6> tray_pose){
//     geometry_msgs::Quaternion tray_quat = rpy_to_quat(tray_pose[0], tray_pose[1], tray_pose[2]);

//     geometry_msgs::Pose tray_pose_dbg;
//     tray_pose_dbg.position.x = tray_pose[3];
//     tray_pose_dbg.position.y = tray_pose[4];
//     tray_pose_dbg.position.z = tray_pose[5];
//     tray_pose_dbg.orientation = tray_quat;

//     ROS_INFO_STREAM("🧱 tray_pose = ["
//         << tray_pose_dbg.position.x << ", "
//         << tray_pose_dbg.position.y << ", "
//         << tray_pose_dbg.position.z << ", "
//         << tray_pose_dbg.orientation.x << ", "
//         << tray_pose_dbg.orientation.y << ", "
//         << tray_pose_dbg.orientation.z << ", "
//         << tray_pose_dbg.orientation.w << "]");


    
//     geometry_msgs::Quaternion quat_tray=rpy_to_quat(tray_pose[0],tray_pose[1],tray_pose[2]);; // Orientation of the tray wrt the world RF

//     geometry_msgs::Quaternion quat_target=target_poses[index].orientation; // Orientation of the hole wrt the tray RF

//     geometry_msgs::Quaternion quat_tool=rpy_to_quat(pi,pi/2,0); // Screwing orientation wrt the tool RF

//     std::array<std::array<double, 3>, 3> matrix_tray=quat_to_rotationMatrix(quat_tray);
//     std::array<std::array<double, 3>, 3> matrix_target=quat_to_rotationMatrix(quat_target);
//     std::array<std::array<double, 3>, 3> matrix_tool=quat_to_rotationMatrix(quat_tool);

//     std::array<double, 3> target_position = {target_poses[index].position.x,target_poses[index].position.y,target_poses[index].position.z};
//     std::array<double, 3> flange_position = {tray_pose[3],tray_pose[4],tray_pose[5]};

//     std::array<double, 3> debug = {tray_pose[3],tray_pose[4],tray_pose[5]};

//     std::array<std::array<double, 3>, 3> matrix_target_world = {{{0.0, 0.0, 0.0},{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}}}; // Orientation of the target wrt the world RF
//     std::array<std::array<double, 3>, 3> matrix_flange = {{{0.0, 0.0, 0.0},{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}}}; // Orientation of the flange wrt the world RF

//     for (int i = 0; i < 3; i++) {
//         for (int j = 0; j < 3; j++) {
//             for (int k = 0; k < 3; k++) {
//                 matrix_target_world[i][j] += matrix_tray[i][k] * matrix_target[k][j];
//             }
//         }
//     }

//     for (int i = 0; i < 3; i++) {
//         for (int j = 0; j < 3; j++) {
//             for (int k = 0; k < 3; k++) {
//                 matrix_flange[i][j] +=  matrix_target_world[i][k] *matrix_tool[k][j];
//             }
//         }
//     }

//     for (int i = 0; i < 3; ++i) {
//         for (int j = 0; j < 3; ++j) {
//             debug[i] += (matrix_tray[i][j] * target_position[j]);
//             flange_position[i] += (matrix_tray[i][j] * target_position[j]+ matrix_flange[i][j]*tip_position[j]);
//         }
//     }

//     geometry_msgs::Quaternion quat_flange=rotationMatrix_to_quat(matrix_flange);

//     geometry_msgs::Pose flange_target_pose;

//     flange_target_pose.position.x=flange_position[0];
//     flange_target_pose.position.y=flange_position[1];
//     flange_target_pose.position.z=flange_position[2];
//     flange_target_pose.orientation=quat_flange;

    
//     ROS_INFO_STREAM("📌 flange_target_pose = ["
//         << flange_target_pose.position.x << ", "
//         << flange_target_pose.position.y << ", "
//         << flange_target_pose.position.z << "]");
//     ROS_INFO_STREAM("🎯 orientation = ["
//         << flange_target_pose.orientation.x << ", "
//         << flange_target_pose.orientation.y << ", "
//         << flange_target_pose.orientation.z << ", "
//         << flange_target_pose.orientation.w << "]");


//     return flange_target_pose;
// }

// Takes as input the pose of the tray wrt the world RF and gives as output the pose of the flange wrt the world RF
geometry_msgs::Pose kr120_flange_target_pose(std::array<double, 6> tray_pose,std::array<double,3> tray_flange_position,double tray_flange_angle){
    geometry_msgs::Pose target_pose; 

    geometry_msgs::Quaternion quat_tray=rpy_to_quat(tray_pose[0],tray_pose[1],tray_pose[2]); // Orientation of the tray wrt the world RF
    
    geometry_msgs::Quaternion quat_flange=rpy_to_quat(0,pi/2+tray_flange_angle,-pi/2); // Orientation of the flange wrt the tray RF
    
    std::array<std::array<double, 3>, 3> matrix_tray=quat_to_rotationMatrix(quat_tray);
    std::array<std::array<double, 3>, 3> matrix_flange=quat_to_rotationMatrix(quat_flange);

    std::array<std::array<double, 3>, 3> matrix_flange_world = {{{0.0, 0.0, 0.0},{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}}}; // Orientation of the flange wrt the world RF
    std::array<double, 3> flange_position = {tray_pose[3],tray_pose[4],tray_pose[5]};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                matrix_flange_world[i][j] += matrix_tray[i][k] * matrix_flange[k][j];
            }
        }
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            flange_position[i] += matrix_tray[i][j] * tray_flange_position[j];
        }
    }

    target_pose.orientation=rotationMatrix_to_quat(matrix_flange_world);
    target_pose.position.x=flange_position[0];
    target_pose.position.y=flange_position[1];
    target_pose.position.z=flange_position[2];

    return target_pose;
}

// Compute the target pose of the flange wrt the world RF which consider the distance from the hole necessary to move the slide and screw
void compute_screw_pose2(
    geometry_msgs::Pose& screw_pose,
    const geometry_msgs::Pose& flange_target,       // this gives you the tool-flange pose in world
    const geometry_msgs::Pose& hole_in_tray)        // this is the JSON hole in *tray* frame
{
  // 1) Reconstruct R_tray and tray_world_pos from flange_target *before* the tip offset
  //    (if you factor your flange function to also spit out the pure hole pose, even better)
  auto R_fw = quat_to_rotationMatrix(flange_target.orientation);
  // tip_position is your known [x,y,z] in tool frame that you added there originally
  extern std::array<double,3> tip_position;

  // 2) Compute the *world* location of the hole by subtracting the tip offset:
  std::array<double,3> p_flange = {
    flange_target.position.x,
    flange_target.position.y,
    flange_target.position.z
  };
  std::array<double,3> world_hole = p_flange;
  for(int i=0; i<3; ++i){
    world_hole[i] -=
      R_fw[i][0]*tip_position[0] +
      R_fw[i][1]*tip_position[1] +
      R_fw[i][2]*tip_position[2];
  }

  ROS_INFO_STREAM("🌐 world_hole = [" 
    << world_hole[0] << ", "
    << world_hole[1] << ", "
    << world_hole[2] << "]");

  // 3) Now apply your small offset along the *tool’s Z* to get the screw start pose:
  double offset = 0.018;  // as before
  std::array<double,3> screw_p = world_hole;
  for(int i=0;i<3;i++){
    screw_p[i] += R_fw[i][2] * offset;
  }

  screw_pose.position.x = screw_p[0];
  screw_pose.position.y = screw_p[1];
  screw_pose.position.z = screw_p[2];
  screw_pose.orientation   = flange_target.orientation;  

  ROS_INFO_STREAM("🔩 final screw_pose = ["
    << screw_pose.position.x << ", "
    << screw_pose.position.y << ", "
    << screw_pose.position.z << "]");

  ROS_INFO_STREAM("💠 hole_in_tray.orientation = ["
    << hole_in_tray.orientation.x << ", "
    << hole_in_tray.orientation.y << ", "
    << hole_in_tray.orientation.z << ", "
    << hole_in_tray.orientation.w << "]");
  ROS_INFO_STREAM("💠 flange_target.orientation = ["
    << flange_target.orientation.x << ", "
    << flange_target.orientation.y << ", "
    << flange_target.orientation.z << ", "
    << flange_target.orientation.w << "]");
  ROS_INFO_STREAM("💠 final screw_pose.orientation = ["
    << screw_pose.orientation.x << ", "
    << screw_pose.orientation.y << ", "
    << screw_pose.orientation.z << ", "
    << screw_pose.orientation.w << "]");

}

void compute_screw_pose(geometry_msgs::Pose& screw_pose,geometry_msgs::Pose target_pose,int index){
    
    geometry_msgs::Quaternion quat_target;
    
    quat_target=target_pose.orientation;

    std::array<std::array<double, 3>, 3> matrix_target=quat_to_rotationMatrix(quat_target);

    double offset; // Distance to have along the screwing direction
    if(index==-1) offset = 0.018;
    else if((index>=0 && index<=87)||index>=96) offset = 0.018; // internal MTQ and CMG: 16mm screw
    else offset = 0.042; // external MTQ and MAG: 40mm screw

    std::array<double, 3> screw_position ={target_pose.position.x,target_pose.position.y,target_pose.position.z};

    for (int i = 0; i < 3; ++i) {
        screw_position[i]+=matrix_target[i][2]*offset;
    }

    screw_pose.orientation=quat_target;
    screw_pose.position.x=screw_position[0];
    screw_pose.position.y=screw_position[1];
    screw_pose.position.z=screw_position[2];
}


// Path plan using RRT and execute the motion
void execute_rrt(geometry_msgs::Pose target_pose, moveit::planning_interface::MoveGroupInterface& group){
    group.setStartStateToCurrentState(); // Start to plan always from the current configuration
    group.setPoseTarget(target_pose);

    moveit_msgs::MoveItErrorCodes result;

    int count=0;
    do{
        count++;
        result=group.plan(plan);
    }while(count<= 10 && result.val!=moveit_msgs::MoveItErrorCodes::SUCCESS); // Try to get a plan untill success
    group.execute(plan);
}

// Execute slide motion to screw
void execute_screw(moveit::planning_interface::MoveGroupInterface& group,int index){
    std::vector< double > j_val;
    if(index==-1) j_val = {-0.016};
    else if((index>=0 && index<=87)||index>=96) j_val = {-0.016}; // Consistent with offset
    else j_val = {-0.040};
    
    group.setStartStateToCurrentState();
    group.setJointValueTarget(j_val);
    group.plan(plan);
    group.execute(plan);

    j_val[0]=0;
    group.setStartStateToCurrentState();
    group.setJointValueTarget(j_val);
    group.plan(plan);
    group.execute(plan);
}

// Linear cartesian path plan and execute the motion
void execute_lin_path(geometry_msgs::Pose pose, moveit::planning_interface::MoveGroupInterface& group){
    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(pose);
    group.setStartStateToCurrentState();

    moveit_msgs::RobotTrajectory trajectory;
    const double eef_step = 0.09;
    double fraction = group.computeCartesianPath(waypoints, eef_step, trajectory,false);
    group.execute(trajectory); 
}

// Set desired values for some joints and execute the motion
void set_joints(moveit::planning_interface::MoveGroupInterface& group,std::vector<std::string> joints, std::vector<double> values){
    for (int i = 0; i < joints.size(); i++){
        group.setJointValueTarget(joints[i], values[i]);
    }
    group.move();
}

// Attach the tray to the given link in gazebo
void gazebo_attach(std::string model_1, std::string link_1, std::string model_2, std::string link_2){
    attach_srv.request.model_name_1 = model_1;
    attach_srv.request.link_name_1 = link_1;
    attach_srv.request.model_name_2 = model_2;
    attach_srv.request.link_name_2 = link_2;
    attach_client.call(attach_srv);
    sleep(1.0);
}

// Detach the tray to the given link in gazebo
void gazebo_detach(std::string model_1, std::string link_1, std::string model_2, std::string link_2){
    attach_srv.request.model_name_1 = model_1;
    attach_srv.request.link_name_1 = link_1;
    attach_srv.request.model_name_2 = model_2;
    attach_srv.request.link_name_2 = link_2;
    detach_client.call(attach_srv);
    sleep(1.0);
}

// Close the gripper and attach the tray to the robot
void close_gripper(moveit::planning_interface::MoveGroupInterface& group,std::vector<moveit_msgs::CollisionObject>& tray_components,moveit::planning_interface::PlanningSceneInterface& planning_scene){
    set_joints(group,plier_joints,{0,0});

    // gazebo_attach("kr20_kr120", "kr120_gyro_left", tray_name, "link_0");

    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "kr120_gyro_left";

    if(tray_components.size()>5){
        int coll_obj_size=tray_components.size();
        for (int i = 0; i < coll_obj_size-1; i++){
            tray_components[i].pose=tray_components[coll_obj_size-1].pose;
        }
    }

    for (int i = 0; i < tray_components.size(); i++){
        tray_components[i].operation = tray_components[i].ADD;
        attached_object.object = tray_components[i];
        planning_scene.applyAttachedCollisionObject(attached_object);
    }
}

// Compute a new pose by rotating the given one and execute the motion to it
void rotate_flange(moveit::planning_interface::MoveGroupInterface& group,geometry_msgs::Pose& pose,double angle){
    tf2::Quaternion rotation;
    rotation.setRPY(angle,0, 0);

    tf2::Quaternion current_orientation;
    tf2::fromMsg(pose.orientation,current_orientation);

    tf2::Quaternion new_orientation;
    new_orientation = current_orientation*rotation; 
    new_orientation.normalize();

    geometry_msgs::Pose target_pose=pose;
    target_pose.orientation=tf2::toMsg(new_orientation);
    execute_rrt(target_pose,group);
}

// Reach the pick pose and disable collision check for the tray with the gripper and the table 
void pick(moveit::planning_interface::MoveGroupInterface& group, std::vector<moveit_msgs::CollisionObject>& tray_components, int count_components, geometry_msgs::Pose& pick_pose){
    execute_lin_path(pick_pose,group);

    moveit_msgs::PlanningScene currentScene;
    moveit_msgs::PlanningScene newSceneDiff;
    moveit_msgs::GetPlanningScene scene_srv;

    scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;
    std::vector<std::string> names;
    if(!scene_client.call(scene_srv)){
        ROS_WARN("Failed to call service /get_planning_scene");
    }else{
        currentScene = scene_srv.response.scene;

        moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;

        if(!contains_string(currentACM.entry_names,"tavolo")){
            currentACM.entry_names.push_back("tavolo");     
            moveit_msgs::AllowedCollisionEntry entry_table;
            entry_table.enabled.resize(currentACM.entry_names.size());

            currentACM.entry_values.push_back(entry_table);

            for (int i = 0; i < currentACM.entry_names.size()-1; i++){
                currentACM.entry_values[i].enabled.push_back(false);
            }
        }
        
        int table_index= get_index(names,"tavolo");

        for(int i = 0; i < names.size(); i++){
            if (i==table_index){
                currentACM.entry_values[table_index].enabled[i] = true;
                currentACM.entry_values[i].enabled[table_index] = true;
            }else{
                currentACM.entry_values[table_index].enabled[i] = false;
                currentACM.entry_values[i].enabled[table_index] = false;
            }
        }

        if(!contains_string(currentACM.entry_names,tray_name)){
            int entry_size=currentACM.entry_names.size()+tray_components.size();
            for (int i = 0; i < tray_components.size(); i++){
                moveit_msgs::AllowedCollisionEntry entry;
                entry.enabled.resize(entry_size);
                currentACM.entry_names.push_back(tray_components[i].id);
                currentACM.entry_values.push_back(entry);
            }
            
            for (int i = 0; i < currentACM.entry_names.size()-tray_components.size(); i++){
                for (int j = 0; j < tray_components.size(); j++){
                    currentACM.entry_values[i].enabled.push_back(false);
                }    
            }
        }else if(tray_name=="tray_aoc"){
            int entry_size=currentACM.entry_names.size()+1;
            moveit_msgs::AllowedCollisionEntry entry;
            entry.enabled.resize(entry_size);
            currentACM.entry_names.push_back("bottom_panel");
            currentACM.entry_values.push_back(entry);
            
            for (int i = 0; i < currentACM.entry_names.size()-1; i++){
                currentACM.entry_values[i].enabled.push_back(false);
            }
        }

        names=currentACM.entry_names;

        int tray_index= get_index(names,tray_name);
        int gyro_left_index= get_index(names,"kr120_gyro_left");
        int gyro_right_index= get_index(names,"kr120_gyro_right");
        int slide_index= get_index(names,"kr20_slide");
        int fixed_index= get_index(names,"kr20_fixed");
        int slider_index=get_index(names,"rl_slider");
        int plate_index=get_index(names,"rl_plate");
        int bottom_index=get_index(names,"bottom_panel");

        for(int i = 0; i < names.size(); i++){
            if (i==table_index || i==gyro_left_index || i==gyro_right_index){
                currentACM.entry_values[tray_index].enabled[i] = true;
                currentACM.entry_values[i].enabled[tray_index] = true;
            }else{
                currentACM.entry_values[tray_index].enabled[i] = false;
                currentACM.entry_values[i].enabled[tray_index] = false;
            }

            if(i>=tray_index){
                for (int j = 0; j < names.size(); j++){
                    if (j>=tray_index){
                        currentACM.entry_values[i].enabled[j]=true;
                        currentACM.entry_values[j].enabled[i]=true;
                    }
                    if(i>=tray_index+1+count_components && (j==slide_index || j==fixed_index)){
                        currentACM.entry_values[i].enabled[j]=true;
                        currentACM.entry_values[j].enabled[i]=true;
                    }
                }
                currentACM.entry_values[i].enabled[slider_index]=true;
                currentACM.entry_values[slider_index].enabled[i]=true;
            }
        }

        if(bottom_index>0){
            currentACM.entry_values[bottom_index].enabled[slider_index]=true;
            currentACM.entry_values[slider_index].enabled[bottom_index]=true;
            currentACM.entry_values[bottom_index].enabled[plate_index]=true;
            currentACM.entry_values[plate_index].enabled[bottom_index]=true;
        }
        
        newSceneDiff.is_diff = true;
        newSceneDiff.allowed_collision_matrix = currentACM;
        scene_publisher.publish(newSceneDiff);
    }

    if(!scene_client.call(scene_srv)){
        ROS_WARN("Failed to call service /get_planning_scene");
    }else{
        currentScene = scene_srv.response.scene;

        moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix; 
    }

    sleep(1.0);
}

// enable collision checking for the component which is just added
void enable_component_collision(const std::string& object_name){
    moveit_msgs::PlanningScene currentScene;
    moveit_msgs::PlanningScene newSceneDiff;
    moveit_msgs::GetPlanningScene scene_srv;

    scene_srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

    if (!scene_client.call(scene_srv)) {
        ROS_WARN("Failed to call service /get_planning_scene");
        return;
    }

    currentScene = scene_srv.response.scene;
    moveit_msgs::AllowedCollisionMatrix& currentACM = currentScene.allowed_collision_matrix;

    std::vector<std::string>& names = currentACM.entry_names;

    int component_index = get_index(names, object_name);
    int slide_index = get_index(names, "kr20_slide");
    int fixed_index = get_index(names, "kr20_fixed");

    if (component_index == -1 || slide_index == -1 || fixed_index == -1) {
        ROS_ERROR("⚠️ One or more required objects not found in the collision matrix!");
        return;
    }

    // Disable collision between component and the two robot parts
    currentACM.entry_values[slide_index].enabled[component_index] = false;
    currentACM.entry_values[component_index].enabled[slide_index] = false;
    currentACM.entry_values[fixed_index].enabled[component_index] = false;
    currentACM.entry_values[component_index].enabled[fixed_index] = false;

    newSceneDiff.is_diff = true;
    newSceneDiff.allowed_collision_matrix = currentACM;
    scene_publisher.publish(newSceneDiff);
}

    // moveit_msgs::PlanningScene currentScene;
    // moveit_msgs::PlanningScene newSceneDiff;
    // moveit_msgs::GetPlanningScene scene_srv;

    // scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;

    // if(!scene_client.call(scene_srv)){
    //     ROS_WARN("Failed to call service /get_planning_scene");
    // }else{
    //     currentScene = scene_srv.response.scene;

    //     moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;

    //     std::vector<std::string> names=currentACM.entry_names;

    //     int slide_index= get_index(names,"kr20_slide");
    //     int fixed_index= get_index(names,"kr20_fixed");
    //     int component_index;

    //     if(tray_name=="tray_aoc"){
    //         switch (count){
    //         case 1:
    //             component_index = get_index(names,"MTQ_MAG");
    //             break;
    //         case 2:
    //             component_index = get_index(names,"CMG2");
    //             break;
    //         case 3:
    //             component_index = get_index(names,"CMG1");
    //             break;
    //         case 4:
    //             component_index = get_index(names,"CMG34");
    //             break;
    //         default:
    //             component_index = -1;
    //             break;
    //         }
    //     }else if(tray_name=="tray_eps"){
    //         switch (count){
    //         case 1:
    //             component_index = get_index(names,"BAT1");
    //             break;
    //         case 2:
    //             component_index = get_index(names,"BAT2");
    //             break;
    //         case 3:
    //             component_index = get_index(names,"PCDU");
    //             break;
    //         default:
    //             component_index = -1;
    //             break;
    //         }
    //     }

    //     currentACM.entry_values[slide_index].enabled[component_index]=false;
    //     currentACM.entry_values[component_index].enabled[slide_index]=false;
    //     currentACM.entry_values[fixed_index].enabled[component_index]=false;
    //     currentACM.entry_values[component_index].enabled[fixed_index]=false;
     
    //     newSceneDiff.is_diff = true;
    //     newSceneDiff.allowed_collision_matrix = currentACM;
    //     scene_publisher.publish(newSceneDiff);

    // }

    // if(!scene_client.call(scene_srv)){
    //     ROS_WARN("Failed to call service /get_planning_scene");
    // }else{
    //     currentScene = scene_srv.response.scene;

    //     moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;

    // }
// }

// Reach the place pose, open the gipper and enable again collision check for the tray
void place(moveit::planning_interface::MoveGroupInterface& arm_group,moveit::planning_interface::MoveGroupInterface& gripper_group, geometry_msgs::Pose& place_pose,std::vector<moveit_msgs::CollisionObject>& tray_components,std::string attach_name,moveit::planning_interface::PlanningSceneInterface& planning_scene){
    execute_lin_path(place_pose,arm_group);
    sleep(2);

    // gazebo_detach("kr20_kr120", "kr120_gyro_left", tray_name, "link_0");

    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "kr120_gyro_left";

    // gazebo_detach("kr20_kr120", "kr120_gyro_left", tray_name, "link_0");


    for (int i = 0; i < tray_components.size(); i++){
        tray_components[i].operation = tray_components[i].REMOVE;
        attached_object.object = tray_components[i];
        planning_scene.applyAttachedCollisionObject(attached_object);
    }

    sleep(1.0);

    gazebo_ros_link_attacher::Attach attach_srv;
    attach_srv.request.model_name_1 = tray_name;
    attach_srv.request.link_name_1="link_0";
    attach_srv.request.model_name_2 = attach_name;
    attach_srv.request.link_name_2=(attach_name=="kr20_kr120") ? "rl_plate" : "link_0";
    attach_client.call(attach_srv);
    std::cout << "openinigGripper" << std::endl;
    set_joints(gripper_group,plier_joints,{-0.035,0.035});

}

// Add the table .stl for collision checking
void add_table(moveit::planning_interface::PlanningSceneInterface& planning_scene){
    // Add the table
    moveit_msgs::CollisionObject table;
    table.id = "tavolo";
    table.header.frame_id = "world";

    static const Eigen::Vector3d scale(0.001, 0.001, 0.001);
    shapes::Mesh* m1 = shapes::createMeshFromResource("package://kr20_kr120/collision/tavolo.stl",scale);
    shapes::ShapeMsg table_mesh_msg;
    shapes::constructMsgFromShape(m1,table_mesh_msg);
    shape_msgs::Mesh table_mesh = boost::get<shape_msgs::Mesh>(table_mesh_msg);
    table.meshes.resize(1);
    table.meshes[0] = table_mesh;
    table.mesh_poses.resize(1);
    
    geometry_msgs::Pose table_pose;
    table_pose.position.x = 1.26;
    table_pose.position.y = -0.125;
    table_pose.position.z = 0.0;  // match Gazebo
    // 90 degrees = π/2 radians
    double angle = -M_PI / 2.0;
    table_pose.orientation.x = 0.0;
    table_pose.orientation.y = 0.0;
    table_pose.orientation.z = sin(angle / 2.0);  // = sin(π/4) = √2/2
    table_pose.orientation.w = cos(angle / 2.0);  // = cos(π/4) = √2/2

    table.mesh_poses[0]=table_pose;

    table.meshes.push_back(table_mesh);
    table.mesh_poses.push_back(table.mesh_poses[0]);
    table.operation = table.ADD;

    collision_objects.push_back(table);
    planning_scene.applyCollisionObjects(collision_objects);

    moveit_msgs::PlanningScene planning_scene_add;
    planning_scene_add.world.collision_objects.push_back(table);
    planning_scene_add.is_diff = true;
    scene_publisher.publish(planning_scene_add);

    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components =  moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;
}

// Add the tray .stl for collision checking 
void add_tray(std::string tray_name, 
    std::vector<moveit_msgs::CollisionObject>& tray_components,
    moveit::planning_interface::PlanningSceneInterface& planning_scene)
{  
    static const Eigen::Vector3d scale(0.001, 0.001, 0.001);

    tray_components.clear();  // remove any old data first

    // First element is always the tray base itself
    moveit_msgs::CollisionObject tray;
    tray.id = tray_name;
    tray_components.push_back(tray);

    // Add components depending on the tray
    if (tray_name == "tray_aoc") {
    std::vector<std::string> component_ids = {
    "MTQ12", "MTQ3_MAG", "CMG2", "CMG1", "CMG34"
    };
    for (const auto& id : component_ids) {
    moveit_msgs::CollisionObject comp;
    comp.id = id;
    tray_components.push_back(comp);
    }
    } else if (tray_name == "tray_eps") {
    std::vector<std::string> component_ids = {
    "BAT1", "BAT2", "PCDU"
    };
    for (const auto& id : component_ids) {
    moveit_msgs::CollisionObject comp;
    comp.id = id;
    tray_components.push_back(comp);
    }
    } else if (tray_name == "bottom_panel") {
    // Explicitly handle bottom_panel here (NEW LOGIC)
    tray_components.resize(1);
    tray_components[0].id = "bottom_panel";
    } else if (tray_name == "tray_dhc") {
    std::cout << "inside trayname = tray_dhc" << std::endl;
    tray_components.resize(1);
    std::cout << "inside trayname = tray_dhc 2 line" << std::endl;
    tray_components[0].id = "tray_dhc";
    std::cout << "inside trayname = tray_dhc 3 line" << std::endl;
    } else {
    // This handles any other trays (optional safety net)
    tray_components.resize(1);
    tray_components[0].id = tray_name;
    }

    int n_prev_components = 1;
    if (collision_objects.back().id == "bottom_panel") 
    n_prev_components = 6;
    else if (collision_objects.back().id == "PCDU") 
    n_prev_components = 4;

    int coll_obj_size = collision_objects.size();
    for (int i = coll_obj_size - n_prev_components; i < coll_obj_size; i++) {
    collision_objects[i].pose.orientation = rpy_to_quat(tray_final_pose[3], tray_final_pose[4], tray_final_pose[5]);
    collision_objects[i].pose.position.x = tray_final_pose[0];
    collision_objects[i].pose.position.y = tray_final_pose[1];
    collision_objects[i].pose.position.z = tray_final_pose[2];
    }

    for (size_t i = 0; i < tray_components.size(); i++) {
    tray_components[i].header.frame_id = "world";
    shapes::Mesh* m = shapes::createMeshFromResource("package://kr20_kr120/collision/" + tray_components[i].id + ".stl", scale);
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    tray_components[i].meshes.resize(1);
    tray_components[i].meshes[0] = mesh;
    tray_components[i].meshes.push_back(mesh);

    tray_components[i].operation = tray_components[i].ADD;
    tray_components[i].pose = (tray_name != "bottom_panel") ? tray_init_pose : bottom_init_pose;
    }

    moveit_msgs::PlanningScene planning_scene_add;

    for (size_t i = 0; i < tray_components.size(); i++) {
    collision_objects.push_back(tray_components[i]);
    planning_scene.applyCollisionObject(tray_components[i]);
    planning_scene_add.world.collision_objects.push_back(tray_components[i]);
    }

    planning_scene_add.is_diff = true;
    scene_publisher.publish(planning_scene_add);

    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;
}

// Set cartesian boundary for planning
void set_constraints(std::vector<moveit_msgs::Constraints>& groups_constraints){
    moveit_msgs::OrientationConstraint orientation_constr;
    orientation_constr.header.frame_id = "world";
    orientation_constr.orientation=rpy_to_quat(0,0,0);
    orientation_constr.link_name="kr20_flange";
    orientation_constr.absolute_x_axis_tolerance=0.5;
    orientation_constr.absolute_y_axis_tolerance=0.5;
    orientation_constr.absolute_z_axis_tolerance=pi;
    orientation_constr.weight=1.0;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 4.0;
    primitive.dimensions[1] = 4.5;
    primitive.dimensions[2] = 2.5;

    geometry_msgs::Pose pose;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    pose.position.x = 1.0;
    pose.position.y = -0.75;
    pose.position.z = 1.25;

    moveit_msgs::BoundingVolume bounding_volume;
    bounding_volume.primitives.push_back(primitive);
    bounding_volume.primitive_poses.push_back(pose);

    moveit_msgs::PositionConstraint kr20_position_constr;
    kr20_position_constr.header.frame_id = "world";
    kr20_position_constr.link_name="kr20_flange";
    kr20_position_constr.weight=1.0;
    kr20_position_constr.constraint_region=bounding_volume;

    moveit_msgs::PositionConstraint kr120_position_constr;
    kr120_position_constr.header.frame_id = "kr120_base_link";
    kr120_position_constr.link_name="kr120_flange";
    kr120_position_constr.weight=1.0;
    kr120_position_constr.constraint_region=bounding_volume;

    moveit_msgs::Constraints kr20_constraints;
    kr20_constraints.orientation_constraints.push_back(orientation_constr);
    kr20_constraints.position_constraints.push_back(kr20_position_constr);

    moveit_msgs::Constraints kr120_constraints;
    kr120_constraints.position_constraints.push_back(kr120_position_constr);

    groups_constraints.push_back(kr20_constraints);
    groups_constraints.push_back(kr120_constraints);
}

// Attach the tray to the given link
void attach_tray(const std::string& tray_id, const std::string& parent_link, moveit::planning_interface::PlanningSceneInterface& planning_scene) {
    std::vector<moveit_msgs::CollisionObject> tray_components;

    // Extract only the components that belong to this tray
    for (const auto& obj : collision_objects) {
        if (obj.id.find(tray_id) != std::string::npos) {
            tray_components.push_back(obj);
        }
    }

    // Set correct pose for all components
    for (auto& obj : tray_components) {
        obj.pose.orientation = rpy_to_quat(tray_final_pose[3], tray_final_pose[4], tray_final_pose[5]);
        obj.pose.position.x = tray_final_pose[0];
        obj.pose.position.y = tray_final_pose[1];
        obj.pose.position.z = tray_final_pose[2];

        // Mark as attached
        obj.operation = obj.ADD;

        moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = parent_link;
        attached_object.object = obj;

        planning_scene.applyAttachedCollisionObject(attached_object);
    }

    ROS_INFO_STREAM("✅ Attached tray '" << tray_id << "' to link '" << parent_link << "' with " << tray_components.size() << " components.");
    sleep(1);
}


// Detach the tray to the given link
void detach_tray(std::string link,moveit::planning_interface::PlanningSceneInterface& planning_scene){
    int n_prev_components=1;
    if(collision_objects.back().id=="CMG34") n_prev_components=5;
    else if(collision_objects.back().id=="PCDU") n_prev_components=4;
    int coll_obj_size=collision_objects.size();
    for (int i = coll_obj_size-n_prev_components; i < coll_obj_size; i++){
        collision_objects[i].pose.orientation=rpy_to_quat(tray_final_pose[3],tray_final_pose[4],tray_final_pose[5]);
        collision_objects[i].pose.position.x = tray_final_pose[0];
        collision_objects[i].pose.position.y = tray_final_pose[1];
        collision_objects[i].pose.position.z=tray_final_pose[2];
    }
    
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = link;

    for (int i = 1; i < collision_objects.size(); i++){
        collision_objects[i].operation = collision_objects[i].REMOVE;
        attached_object.object = collision_objects[i];
        planning_scene.applyAttachedCollisionObject(attached_object);
    }
    for (int i = 1; i < collision_objects.size(); i++){
        collision_objects[i].operation = collision_objects[i].ADD;
        planning_scene.applyCollisionObject(collision_objects[i]);
    }
}

// Perform complete sequence for tray picking
void pick_sequence(moveit::planning_interface::MoveGroupInterface& arm_group,moveit::planning_interface::MoveGroupInterface& gripper_group, moveit::planning_interface::PlanningSceneInterface& planning_scene, int i){
    if(tray_name=="tray_aoc") count_components=i/2;
    else if(tray_name=="tray_eps") count_components=ceil(i/2)-4;

    set_joints(gripper_group,gyro_joints,{0,0});

    std::cout << "=== pre_pick_pose ===" << std::endl;
    std::cout << "Position: [" << pre_pick_pose.position.x << ", "
            << pre_pick_pose.position.y << ", "
            << pre_pick_pose.position.z << "]" << std::endl;
    std::cout << "Orientation: [" << pre_pick_pose.orientation.x << ", "
            << pre_pick_pose.orientation.y << ", "
            << pre_pick_pose.orientation.z << ", "
            << pre_pick_pose.orientation.w << "]" << std::endl;

    if(i==0) execute_rrt(pre_pick_pose,arm_group); // Move kr120 to the pre-pick pose
    
    set_joints(gripper_group,plier_joints,{-0.035,0.035}); // Open kr120 gripper

    if(tray_name=="tray_payload") gazebo_attach("kr20_kr120","table","tray_payload","link_0");

    std::cout << "=== pick_pose ===" << std::endl;
    std::cout << "Position: [" << pick_pose.position.x << ", "
            << pick_pose.position.y << ", "
            << pick_pose.position.z << "]" << std::endl;
    std::cout << "Orientation: [" << pick_pose.orientation.x << ", "
            << pick_pose.orientation.y << ", "
            << pick_pose.orientation.z << ", "
            << pick_pose.orientation.w << "]" << std::endl;
    
    pick(arm_group,tray_map[tray_name],count_components,pick_pose); // Move kr120 to the pick pose
    
    close_gripper(gripper_group,tray_map[tray_name],planning_scene); // Close kr120 gripper
    
    if(tray_name=="tray_payload") gazebo_detach("kr20_kr120","table","tray_payload","link_0");
    
    execute_lin_path(post_pick_pose,arm_group); // Move kr120 to the post-pick pose
}

// Perform complete sequence to put the tray in the position where the human operator works
void operator_sequence(moveit::planning_interface::MoveGroupInterface& kr20_arm_group, moveit::planning_interface::MoveGroupInterface& kr120_arm_group,moveit::planning_interface::MoveGroupInterface& kr120_gripper_group, int i){
    count_components++;

    if(!contains(new_tray_indeces, i)) execute_rrt(rest_pose,kr20_arm_group); // Move kr20 to the rest pose

    if(i>1){
        tray_pose_operator[0]=value_tray_pose[i][0];
        tray_pose_operator[1]=value_tray_pose[i][1];
    }else{
        tray_pose_operator[0]=value_tray_pose[1][0];
        tray_pose_operator[1]=value_tray_pose[1][1];
    }
    ROS_INFO("➡️ tray_pose_operator: [%f, %f, %f, %f, %f, %f]",
        tray_pose_operator[0], tray_pose_operator[1], tray_pose_operator[2],
        tray_pose_operator[3], tray_pose_operator[4], tray_pose_operator[5]);

    ROS_INFO("➡️ tray_flange_dist_op: [%f, %f, %f]",
            tray_flange_dist_op[0], tray_flange_dist_op[1], tray_flange_dist_op[2]);

    ROS_INFO("➡️ tray_flange_angle_op: %f", tray_flange_angle_op);

    kr120_target_pose=kr120_flange_target_pose(tray_pose_operator,tray_flange_dist_op,tray_flange_angle_op);
    
    ROS_INFO("⬅️ kr120_target_pose.position:  [%.3f, %.3f, %.3f]",
        kr120_target_pose.position.x, kr120_target_pose.position.y, kr120_target_pose.position.z);

    ROS_INFO("⬅️ kr120_target_pose.orientation: [%.3f, %.3f, %.3f, %.3f]",
            kr120_target_pose.orientation.x, kr120_target_pose.orientation.y,
            kr120_target_pose.orientation.z, kr120_target_pose.orientation.w);

    execute_rrt(kr120_target_pose,kr120_arm_group);

    sleep(5.0);

    if(object_name == "MTQ3_MAG"){

        spawnTrayInGazebo(object_name);
        gazebo_attach(tray_name, "link_0", object_name, "link_0");

    } else if(object_name == "MTQ12"){
        rotate_flange(kr120_arm_group, kr120_target_pose, -M_PI / 2);
        spawnTrayInGazebo(object_name);
        gazebo_attach(tray_name, "link_0", object_name, "link_0");

    }else if(object_name == "CMG2"){
            rotate_flange(kr120_arm_group, kr120_target_pose, M_PI);
            spawnTrayInGazebo(object_name);
            gazebo_attach(tray_name, "link_0", object_name, "link_0");

    }else if(object_name == "CMG1"){
        spawnTrayInGazebo(object_name);
        gazebo_attach(tray_name, "link_0", object_name, "link_0");

    }else if(object_name == "CMG34"){
        rotate_flange(kr120_arm_group, kr120_target_pose, M_PI / 2);
        spawnTrayInGazebo(object_name);
        gazebo_attach(tray_name, "link_0", object_name, "link_0");
    }
    enable_component_collision(object_name);
}

// Perform complete sequence to pick another screw
void recharge_sequence(moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& slide_group){
    ROS_INFO("Moving to pre_recharge pose: x=%.3f y=%.3f z=%.3f",
        pre_recharge.position.x,
        pre_recharge.position.y,
        pre_recharge.position.z);

    execute_lin_path(pre_recharge, arm_group); // Move kr20 to the recharge pose
    execute_screw(slide_group,-1); // Recharge

    // To get the position of the next screw on the table
    recharge_index=recharge_index+1;
    if(recharge_index>20){
        recharge_index=0;
        pre_recharge.position.y=table_pose.position.y-0.17248;
        pre_recharge.position.x+=0.02;
    }else{
        pre_recharge.position.y-=0.02;
    }
}

// Perform complete sequence to put the tray in the position where the screwing robot works
void positioning_sequence(moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& gripper_group, int i){
    // Tray positioning
    if(contains(new_tray_indeces,i+1)) rotate_flange(arm_group,kr120_target_pose,pi);
    set_joints(gripper_group,gyro_joints,{pi-gyro_target,-(pi-gyro_target)});
    
    kr120_target_pose=kr120_flange_target_pose(value_tray_pose[i],tray_flange_dist_pos,tray_flange_angle_pos);
    execute_rrt(kr120_target_pose,arm_group);
}

// -----------------------------------------------------------------------------
// SCREWING SEQUENCE (single JSON-driven version)
// -----------------------------------------------------------------------------
// void screwing_sequence2(
//     moveit::planning_interface::MoveGroupInterface& arm_group,
//     moveit::planning_interface::MoveGroupInterface& slide_group,
//     const std::vector<Screw>& screws,
//     const std::string& tray_name)
// {
//   // build tray_units → ext_screw_tray_pose as before…

//   int auto_counter = 0;
//   for (const auto& s : screws) {
//     if (s.mode != "AUTO") continue;
//     const int idx = s.index;

//     // use the two-arg version:
//     auto flange_target =
//       kr20_flange_target_pose2(s.pose,
//                               value_tray_pose[auto_counter]);

//     geometry_msgs::Pose screw_target;
//     compute_screw_pose2(screw_target,
//                        flange_target,
//                        s.pose);

//     // build approach exactly like original
//     geometry_msgs::Pose approach = screw_target;
//     approach.orientation = screw_target.orientation;
//     if (contains(horiz_rotation_indeces, auto_counter)) {
//       approach.position.x -= 0.35 * cos(tray_angle2);
//       approach.position.y += 0.35 * sin(tray_angle2);
//     } else {
//       approach.position.x -= 0.35 * cos(tray_angle1);
//       approach.position.y += 0.35 * sin(tray_angle1);
//     }
//     approach.position.z = screw_target.position.z;

//     ROS_INFO_STREAM("🔧 AUTO["<<auto_counter<<"] idx="<<idx
//       <<" | flange=("
//       <<flange_target.position.x<<","
//       <<flange_target.position.y<<","
//       <<flange_target.position.z<<")"
//       <<" screw=("
//       <<screw_target.position.x<<","
//       <<screw_target.position.y<<","
//       <<screw_target.position.z<<")"
//     );

//     execute_lin_path(approach, arm_group);
//     execute_rrt(screw_target, arm_group);
//     execute_screw(slide_group, idx);
//     execute_rrt(approach, arm_group);

//     if (auto_counter+1 < 
//         std::count_if(screws.begin(), screws.end(),
//                       [](auto& x){ return x.mode=="AUTO"; }))
//     {
//       recharge_sequence(arm_group, slide_group);
//     }
//     ++auto_counter;
//   }
// }

// Perform complete sequence for screwing
void screwing_sequence(moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& slide_group, int begin, int end, int i){
    for (int k = begin; k < end; k++){
        if(begin>0){
            target_index = k;
            kr20_target_pose=kr20_flange_target_pose(target_index,ext_screw_tray_pose);
        }else{
            target_index = target_order[target_order_index][k];
            ROS_INFO_STREAM("🧪 value_tray_pose[" << i << "] = ["
                << value_tray_pose[i][0] << ", " << value_tray_pose[i][1] << ", " << value_tray_pose[i][2] << ", "
                << value_tray_pose[i][3] << ", " << value_tray_pose[i][4] << ", " << value_tray_pose[i][5] << "]");

            kr20_target_pose=kr20_flange_target_pose(target_index,value_tray_pose[i]);
        }
        // Tray pose used
        ROS_INFO_STREAM("🧱 tray_pose = ["
            << tray_pose.position.x << ", "
            << tray_pose.position.y << ", "
            << tray_pose.position.z << ", "
            << tray_pose.orientation.x << ", "
            << tray_pose.orientation.y << ", "
            << tray_pose.orientation.z << "]");


        // Local screw pose
        geometry_msgs::Pose local_target_pose = target_poses[target_index];
        ROS_INFO_STREAM("🔘 target_pose[" << target_index << "] = ["
            << local_target_pose.position.x << ", "
            << local_target_pose.position.y << ", "
            << local_target_pose.position.z << "]");



        // Final flange pose
        ROS_INFO_STREAM("📌 flange pose = ["
            << kr20_target_pose.position.x << ", "
            << kr20_target_pose.position.y << ", "
            << kr20_target_pose.position.z << "]");

        
        compute_screw_pose(screw_pose,kr20_target_pose,target_index);

        ROS_INFO_STREAM("🔧 i = " << i);
        ROS_INFO_STREAM("📐 tray_angle1 = " << tray_angle1 << ", tray_angle2 = " << tray_angle2);
        ROS_INFO_STREAM("🎯 target_index = " << target_index);
        ROS_INFO_STREAM("📍 kr20_target_pose = [" << kr20_target_pose.position.x << ", "
                                                << kr20_target_pose.position.y << ", "
                                                << kr20_target_pose.position.z << "]");
        ROS_INFO_STREAM("🔩 screw_pose = [" << screw_pose.position.x << ", "
                                            << screw_pose.position.y << ", "
                                            << screw_pose.position.z << "]");
        ROS_INFO_STREAM("🚀 approach_pose = [" << approach_pose.position.x << ", "
                                            << approach_pose.position.y << ", "
                                            << approach_pose.position.z << "]");

        ROS_INFO_STREAM("🧪 screw_pose.position = [" 
            << screw_pose.position.x << ", " 
            << screw_pose.position.y << ", " 
            << screw_pose.position.z << "]");
        ROS_INFO_STREAM("🧪 tray_angle1 = " << tray_angle1 << ", tray_angle2 = " << tray_angle2);
        ROS_INFO_STREAM("🧪 horiz_rotation_indeces contains i = " << contains(horiz_rotation_indeces, i));
        

        
        approach_pose.orientation = screw_pose.orientation;
        if(begin>0){
            approach_pose.position.x = screw_pose.position.x;
            approach_pose.position.y = screw_pose.position.y+0.30;
        }else if(contains(horiz_rotation_indeces, i)){
            approach_pose.position.x = screw_pose.position.x-0.35*cos(tray_angle2);
            approach_pose.position.y = screw_pose.position.y+0.35*sin(tray_angle2);
        }else{
            approach_pose.position.x = screw_pose.position.x-0.35*cos(tray_angle1);
            approach_pose.position.y = screw_pose.position.y+0.35*sin(tray_angle1);
        }  
        approach_pose.position.z = screw_pose.position.z;
        
        execute_lin_path(approach_pose,arm_group); 
        
        ROS_INFO_STREAM("vite n. "<< target_index);
        
        execute_rrt(screw_pose,arm_group); 
        
        execute_screw(slide_group,target_index); 
        
        execute_rrt(approach_pose,arm_group); 
        
        if(k<end-1 || k==begin){
            recharge_sequence(arm_group, slide_group);
        }
    }
}

// Perform sequence via user numerical screwing
void screw_num_screwing_sequence(moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& slide_group, int n_screw, int i){
    // for (int k = begin; k < end; k++){
    //     if(begin>0){
    //         target_index = k;
            // kr20_target_pose=kr20_flange_target_pose(target_index,ext_screw_tray_pose);
    //     }else{
    //         target_index = target_order[target_order_index][k];
    //         ROS_INFO_STREAM("🧪 value_tray_pose[" << i << "] = ["
    //             << value_tray_pose[i][0] << ", " << value_tray_pose[i][1] << ", " << value_tray_pose[i][2] << ", "
    //             << value_tray_pose[i][3] << ", " << value_tray_pose[i][4] << ", " << value_tray_pose[i][5] << "]");

            kr20_target_pose=kr20_flange_target_pose(n_screw,value_tray_pose[i]);
    //     }
        // Tray pose used
        ROS_INFO_STREAM("🧱 tray_pose = ["
            << tray_pose.position.x << ", "
            << tray_pose.position.y << ", "
            << tray_pose.position.z << ", "
            << tray_pose.orientation.x << ", "
            << tray_pose.orientation.y << ", "
            << tray_pose.orientation.z << "]");


        // Local screw pose
        geometry_msgs::Pose local_target_pose = target_poses[target_index];
        ROS_INFO_STREAM("🔘 target_pose[" << target_index << "] = ["
            << local_target_pose.position.x << ", "
            << local_target_pose.position.y << ", "
            << local_target_pose.position.z << "]");



        // Final flange pose
        ROS_INFO_STREAM("📌 flange pose = ["
            << kr20_target_pose.position.x << ", "
            << kr20_target_pose.position.y << ", "
            << kr20_target_pose.position.z << "]");

        
        compute_screw_pose(screw_pose,kr20_target_pose,n_screw);

        ROS_INFO_STREAM("🔧 i = " << i);
        ROS_INFO_STREAM("📐 tray_angle1 = " << tray_angle1 << ", tray_angle2 = " << tray_angle2);
        ROS_INFO_STREAM("🎯 target_index = " << target_index);
        ROS_INFO_STREAM("📍 kr20_target_pose = [" << kr20_target_pose.position.x << ", "
                                                << kr20_target_pose.position.y << ", "
                                                << kr20_target_pose.position.z << "]");
        ROS_INFO_STREAM("🔩 screw_pose = [" << screw_pose.position.x << ", "
                                            << screw_pose.position.y << ", "
                                            << screw_pose.position.z << "]");
        ROS_INFO_STREAM("🚀 approach_pose = [" << approach_pose.position.x << ", "
                                            << approach_pose.position.y << ", "
                                            << approach_pose.position.z << "]");

        ROS_INFO_STREAM("🧪 screw_pose.position = [" 
            << screw_pose.position.x << ", " 
            << screw_pose.position.y << ", " 
            << screw_pose.position.z << "]");
        ROS_INFO_STREAM("🧪 tray_angle1 = " << tray_angle1 << ", tray_angle2 = " << tray_angle2);
        ROS_INFO_STREAM("🧪 horiz_rotation_indeces contains i = " << contains(horiz_rotation_indeces, i));
        

        
        approach_pose.orientation = screw_pose.orientation;
        // if(begin>0){
        // approach_pose.position.x = screw_pose.position.x;
        // approach_pose.position.y = screw_pose.position.y+0.30;
        // }else if(contains(horiz_rotation_indeces, i)){
        //     approach_pose.position.x = screw_pose.position.x-0.35*cos(tray_angle2);
        //     approach_pose.position.y = screw_pose.position.y+0.35*sin(tray_angle2);
        // }else{
        approach_pose.position.x = screw_pose.position.x-0.35*cos(tray_angle1);
        approach_pose.position.y = screw_pose.position.y+0.35*sin(tray_angle1);
        // }  
        approach_pose.position.z = screw_pose.position.z;
        
        execute_lin_path(approach_pose,arm_group); 
        
        ROS_INFO_STREAM("vite n. "<< target_index);
        
        execute_rrt(screw_pose,arm_group); 
        
        execute_screw(slide_group,n_screw); 
        
        execute_rrt(approach_pose,arm_group); 
        
        // if(k<end-1 || k==begin){
            recharge_sequence(arm_group, slide_group);
        // }
    // }
}

// // Perform complete sequence for the internal screws
void internal_screwing_by_num(moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& slide_group,int n_screw , int i){


    screw_num_screwing_sequence(arm_group, slide_group,n_screw,i);
}
// Perform complete sequence for the internal screws
void internal_screwing_sequence(moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& slide_group, int i){
    target_order_index++;

    screwing_sequence(arm_group, slide_group, 0, target_order[target_order_index].size(),i);
}

// Perform complete sequence for tray placing on the roller
void place_sequence(moveit::planning_interface::MoveGroupInterface& kr20_arm_group, moveit::planning_interface::MoveGroupInterface& kr120_arm_group, moveit::planning_interface::MoveGroupInterface& kr120_gripper_group, moveit::planning_interface::PlanningSceneInterface& planning_scene){
    execute_lin_path(recharge_pose,kr20_arm_group);
             
    execute_rrt(post_pick_pose,kr120_arm_group);

    std::cout << "📏 tray_final_pose[2] (Z): " << tray_final_pose[2] << std::endl;

    std::string parent_model = "kr20_kr120";
    if (tray_name == "tray_dhc") {
        parent_model = "tray_aoc";
    } else if (tray_name == "tray_eps") {
        parent_model = "tray_dhc";
    } else if (tray_name == "tray_payload") {
        parent_model = "tray_eps";
    }

    
    if(tray_name=="tray_aoc"){
        add_tray("bottom_panel", tray_map["bottom_panel"], planning_scene);

        bottom_panel = tray_map["bottom_panel"];

        spawnTrayInGazebo("bottom_panel");
                
        gazebo_attach("kr20_kr120", "table", "bottom_panel", "link_0");

        ROS_WARN_STREAM("🟡 bottom_pick_pose: " << bottom_pick_pose.position.x << ", "
            << bottom_pick_pose.position.y << ", "
            << bottom_pick_pose.position.z);

        place(kr120_arm_group,kr120_gripper_group, bottom_pick_pose,tray_map[tray_name],"bottom_panel",planning_scene); // Place the tray
        
        execute_rrt(post_pick_pose,kr120_arm_group);

        std::cout << "after execute_rrt" << std::endl;
        
        set_joints(kr120_gripper_group,plier_joints,{-0.035,0.035}); // Open kr120 gripper

        std::cout << "after set_joints" << std::endl;
        
        gazebo_detach("kr20_kr120", "table", "bottom_panel", "link_0"); 
                
        pick(kr120_arm_group,tray_map[tray_name],count_components,bottom_pick_pose); // Move kr120 to the pick pose

        std::cout << "after pick" << std::endl;

        tray_map[tray_name].push_back(bottom_panel[0]);

        std::cout << "after trayMap" << std::endl;

        close_gripper(kr120_gripper_group,tray_map[tray_name],planning_scene); // Close kr120 gripper

        std::cout << "printing_post_pick_pose" << std::endl;
        std::cout << post_pick_pose << std::endl;


        execute_lin_path(post_pick_pose,kr120_arm_group);
    }
    

    std::cout << "printing_pre_end_pose" << std::endl;
    std::cout << pre_end_pose << std::endl;

    
    execute_lin_path(pre_end_pose,kr120_arm_group); // Move kr120 to the pre_end pose
    
    sleep(2.0);

    std::cout << "printing_end_pose" << std::endl;
    std::cout << end_pose << std::endl;

    std::cout << "📍 placing to end_pose: " << end_pose << std::endl;

    std::cout << parent_model << std::endl;
    std::cout << tray_name << std::endl;

    place(kr120_arm_group, kr120_gripper_group, end_pose, tray_map[tray_name], parent_model, planning_scene);

    sleep(1.0);
    attach_tray(tray_name, parent_model, planning_scene);  // attach in MoveIt 

    std::cout << "print trayComponents: " << std::endl;
    // std::cout << tray_dhc << std::endl;

    execute_lin_path(pre_end_pose,kr120_arm_group); // Move kr120 to the pre_end pose

    std::cout << "After_second_pre_end_pose" << std::endl;

    set_joints(kr120_gripper_group,plier_joints,{-0.035,0.035}); // Open kr120 gripper
    
    execute_lin_path(post_pick_pose,kr120_arm_group);
}

// Perform complete sequence for the external screws
void external_screwing_sequence(moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& slide_group, moveit::planning_interface::MoveGroupInterface& rl_plate_group, moveit::planning_interface::PlanningSceneInterface& planning_scene){
    attach_tray("tray_aoc", "rl_plate", planning_scene);
    
    recharge_sequence(arm_group, slide_group);
    
    for(int w=-1; w<3; w++){
        set_joints(rl_plate_group, {"rl_slider__plate"}, {w * pi / 2 + pi / 2});
                    
        screwing_sequence(arm_group,slide_group,140,141,-1);
    }
                
    for(int w=2; w>-2; w--){
        set_joints(rl_plate_group, {"rl_slider__plate"}, {w * pi / 2 + pi / 2});
                    
        screwing_sequence(arm_group,slide_group,141,146,-1);
    }

    execute_rrt(rest_pose,arm_group);
    
    detach_tray("rl_plate",planning_scene);
}

// Perform complete sequence for roller motion
void slide_sequence(moveit::planning_interface::MoveGroupInterface& rl_lifter_group, moveit::planning_interface::MoveGroupInterface& rl_slider_group,double lift_target_1, double slide_target_1, double lift_target_2, double slide_target_2, moveit::planning_interface::PlanningSceneInterface& planning_scene){
    attach_tray("tray_slider", "rl_slider", planning_scene);
    set_joints(rl_lifter_group,{"rl_base__lifter"},{lift_target_1});
    sleep(1.0);
    set_joints(rl_slider_group,{"rl_base__slider"},{slide_target_1});
    sleep(1.0);
    set_joints(rl_slider_group,{"rl_base__slider"},{slide_target_2});
    sleep(1.0);
    set_joints(rl_lifter_group,{"rl_base__lifter"},{lift_target_2});
}

// Set parameters depending on the tray
void tray_setup(const std::string& tray_name, const std::string& current_object = "") {
    ::tray_name = tray_name;  // global var update

    float flange_offset_z = 0.4795;
    float base_z = table_pose.position.z + 0.82025;

    if (tray_name == "tray_aoc") {
        float height = height_tray;
        pick_pose.position.z = base_z + height + flange_offset_z - 0.0015;
        end_pose.position.z = tray_final_pose[2] + orig_bott_dist_aoc + height + flange_offset_z + 0.001;
        tray_init_pose.position.z = base_z - orig_bott_dist_aoc;
        ext_screw_tray_pose[2] = tray_final_pose[2];

        int unit_index = get_pose_index_from_unit_name(current_tray_units, current_object);
        bool is_down = contains2(tray_down_steps, current_object);

        if (is_down) {
            tray_flange_dist_pos = {0, 0, flange_offset_z + height + orig_bott_dist_aoc - 0.002};
            tray_flange_angle_pos = 0;
            tray_flange_dist_op = tray_flange_dist_pos;
            tray_flange_angle_op = 0;
            gyro_target = pi;
        } else {
            tray_flange_dist_pos = {0, 0, -(flange_offset_z - orig_bott_dist_aoc - 0.014)};
            tray_flange_dist_op = {0, 0, flange_offset_z + height + orig_bott_dist_aoc};
            tray_flange_angle_pos = pi;
            tray_flange_angle_op = 0;
            gyro_target = 0;
        }

        new_dummy = current_object;

    } else if (tray_name == "tray_dhc") {
        float height = height_dhc;
        pick_pose.position.z = base_z + height + flange_offset_z - 0.0015;
        end_pose.position.z = tray_final_pose[2] + orig_bott_dist_aoc + height_tray + height + flange_offset_z + 0.005;
        tray_init_pose.position.z = base_z - orig_bott_dist_dhc;
        ext_screw_tray_pose[2] = tray_final_pose[2] + height_tray;

    } else if (tray_name == "tray_eps") {
        float height = height_tray;
        pick_pose.position.z = base_z + height + flange_offset_z - 0.0015;
        end_pose.position.z = tray_final_pose[2] + orig_bott_dist_aoc + 2 * height_tray + height_dhc + flange_offset_z + 0.015;
        tray_init_pose.position.z = base_z - orig_bott_dist_eps;
        ext_screw_tray_pose[2] = tray_final_pose[2] + height_tray + height_dhc;

        bool is_down = contains2(tray_down_steps, current_object);
        if (is_down) {
            tray_flange_dist_pos = {0, 0, flange_offset_z + height + orig_bott_dist_eps - 0.002};
            tray_flange_angle_pos = 0;
            tray_flange_dist_op = {0, 0, flange_offset_z - orig_bott_dist_aoc};
            tray_flange_angle_op = pi;
            gyro_target = pi;
        } else {
            tray_flange_dist_pos = {0, 0, -(flange_offset_z - orig_bott_dist_eps - 0.014)};
            tray_flange_dist_op = {0, 0, flange_offset_z + height + orig_bott_dist_aoc};
            tray_flange_angle_pos = pi;
            tray_flange_angle_op = 0;
            gyro_target = 0;
        }

        new_dummy = current_object;

    } else if (tray_name == "tray_payload") {
        float height = height_payload;
        pick_pose.position.z = base_z + height + flange_offset_z - 0.0015 + 0.05;
        end_pose.position.z = tray_final_pose[2] + orig_bott_dist_aoc + 2 * height_tray + height_dhc + height + flange_offset_z + 0.02;
        tray_init_pose.position.z = base_z - orig_bott_dist_payload;
        ext_screw_tray_pose[2] = tray_final_pose[2] + 2 * height_tray + height_dhc;
    }
    

    ROS_INFO("🧠 Tray setup done for tray [%s] with object [%s]", tray_name.c_str(), current_object.c_str());
}


// Log joint's positions over time
void robotStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    std::ofstream outfile_kr20("/home/user/Desktop/robot_state_kr20", std::ios::app);
    std::ofstream outfile_kr120("/home/user/Desktop/robot_state_kr120", std::ios::app);
    if (outfile_kr20.is_open()) {
        for (int i = 10; i < 17; i++){
            if(i!=11 ) outfile_kr20 << msg->position[i] << ",";
        }
        outfile_kr20 << msg->position[11];
        outfile_kr20 << "\n";
        outfile_kr20.close();
    }
    if (outfile_kr120.is_open()) {
        for (int i = 0; i < 8; i++){
            if(i!=1 && i!=2) outfile_kr120 << msg->position[i] << ",";
        }
        outfile_kr120 << msg->position[9]<< ",";
        outfile_kr120 << msg->position[8]<< ",";
        outfile_kr120 << msg->position[2]<< ",";
        outfile_kr120 << msg->position[1];
        outfile_kr120 << "\n";
        outfile_kr120.close();
    }
    
}





// servers srv for functions trigger 

bool handleAddTray(custom_interfaces::AddTray2::Request &req, custom_interfaces::AddTray2::Response &res, moveit::planning_interface::PlanningSceneInterface &planning_scene) {
    ROS_INFO("🧩 Adding Tray: %s with object: %s", req.tray_name.c_str(), req.object_name.c_str());
    tray_name = req.tray_name; 
    object_name = req.object_name;
    tray_setup(req.tray_name, req.object_name);  // tray_name and optional object
    add_tray(req.tray_name, tray_map[req.tray_name], planning_scene);
    spawnTrayInGazebo(req.tray_name);  
    res.success = true;
    return true;
}

bool handlePickTray(custom_interfaces::PickTray::Request &req, custom_interfaces::PickTray::Response &res, moveit::planning_interface::MoveGroupInterface &kr120_arm_group, moveit::planning_interface::MoveGroupInterface &kr120_gripper_group, moveit::planning_interface::PlanningSceneInterface &planning_scene) {
    ROS_INFO("Picking Tray at index: %d", req.index);
    pick_sequence(kr120_arm_group, kr120_gripper_group, planning_scene, req.index);
    res.success = true;
    return true;
}

bool handlePositionTray(custom_interfaces::PositionTray::Request &req, custom_interfaces::PositionTray::Response &res, moveit::planning_interface::MoveGroupInterface &kr120_arm_group, moveit::planning_interface::MoveGroupInterface &kr120_gripper_group) {
    ROS_INFO("Positioning Tray at index: %d", req.index);
    positioning_sequence(kr120_arm_group, kr120_gripper_group, req.index);
    res.success = true;
    return true;
}

bool handleOperatorPositionTray(custom_interfaces::OperatorPositionTray::Request &req, custom_interfaces::OperatorPositionTray::Response &res, moveit::planning_interface::MoveGroupInterface &kr120_arm_group, moveit::planning_interface::MoveGroupInterface &kr120_gripper_group, moveit::planning_interface::MoveGroupInterface &kr20_arm_group) {
    ROS_INFO("Handling Operator Position Tray at index: %d", req.index);
    operator_sequence(kr20_arm_group, kr120_arm_group, kr120_gripper_group, req.index);
    res.success = true;
    return true;
}

bool handleRechargeSequence(custom_interfaces::RechargeSequence::Request &req, custom_interfaces::RechargeSequence::Response &res, moveit::planning_interface::MoveGroupInterface &kr20_arm_group, moveit::planning_interface::MoveGroupInterface &kr20_slide_group) {
    ROS_INFO("Recharging at index: %d", req.index);
    recharge_sequence(kr20_arm_group, kr20_slide_group);
    res.success = true;
    return true;
}

bool handleInternalScrewingSequence(custom_interfaces::InternalScrewingSequence::Request &req, custom_interfaces::InternalScrewingSequence::Response &res, moveit::planning_interface::MoveGroupInterface &kr20_arm_group, moveit::planning_interface::MoveGroupInterface &kr20_slide_group) {
    ROS_INFO("Executing Internal Screwing at index: %d", req.index);
    internal_screwing_sequence(kr20_arm_group, kr20_slide_group, req.index);
    // internal_screwing_by_num(kr20_arm_group, kr20_slide_group, n_screw, req.index);
    res.success = true;
    return true;
}

bool handleInternalScrewByNum(
    custom_interfaces::InternalScrewByNum::Request  &req,
    custom_interfaces::InternalScrewByNum::Response &res,
    moveit::planning_interface::MoveGroupInterface   &kr20_arm_group,
    moveit::planning_interface::MoveGroupInterface   &kr20_slide_group)
{
    ROS_INFO("🔩 InternalScrewByNum: index=%d, ScrewNum=%d",
             req.index, req.ScrewNum);

    // Pass both into your screwing routine:
    internal_screwing_by_num(
        kr20_arm_group,
        kr20_slide_group,
        req.ScrewNum,   // number of screws
        req.index       // tray unit index
    );

    res.success = true;
    res.message = "Internal screw sequence done";
    return true;
}


// bool handleScrewingUnit(
//     custom_interfaces::ScrewingUnit::Request  & req,
//     custom_interfaces::ScrewingUnit::Response & res,
//     moveit::planning_interface::MoveGroupInterface & kr20_arm_group,
//     moveit::planning_interface::MoveGroupInterface & kr20_slide_group)
// {
//   // Pick your key off the request:
//   const std::string& key = req.unit_name;

//   ROS_INFO("🔧 Internal Screwing: unit \"%s\"", key.c_str());

//   // 1) Lookup screws by unit_name
//   auto it = g_screw_cfg.find(key);
//   if (it == g_screw_cfg.end()) {
//     ROS_ERROR("❌ No screw config for unit \"%s\"!", key.c_str());
//     res.success = false;
//     return true;
//   }

//   // 2) Filter (you can skip this if you know all in that vector are already that unit)
//   std::vector<Screw> to_screw;
//   for (auto &s : it->second) {
//     if (s.unit_name == key)  // now s.unit_name actually has a value
//       to_screw.push_back(s);
//   }

//   if (to_screw.empty()) {
//     ROS_WARN("⚠️  No screws for unit \"%s\"", key.c_str());
//     res.success = false;
//   } else {
//     screwing_sequence2(kr20_arm_group,
//                       kr20_slide_group,
//                       to_screw,
//                       key /* tray_name is ignored here */);
//     res.success = true;
//   }
//   return true;
// }






bool handleExternalScrewingSequence(custom_interfaces::ExternalScrewingSequence::Request &req, custom_interfaces::ExternalScrewingSequence::Response &res, moveit::planning_interface::MoveGroupInterface &kr20_arm_group, moveit::planning_interface::MoveGroupInterface &kr20_slide_group, moveit::planning_interface::MoveGroupInterface &rl_plate_group, moveit::planning_interface::PlanningSceneInterface &planning_scene) {
    ROS_INFO("Executing External Screwing at index: %d", req.index);
    external_screwing_sequence(kr20_arm_group, kr20_slide_group, rl_plate_group, planning_scene);
    res.success = true;
    return true;
}

bool handlePlaceTray(custom_interfaces::PlaceTray::Request &req, custom_interfaces::PlaceTray::Response &res, moveit::planning_interface::MoveGroupInterface &kr120_arm_group, moveit::planning_interface::MoveGroupInterface &kr120_gripper_group, moveit::planning_interface::MoveGroupInterface &kr20_arm_group, moveit::planning_interface::PlanningSceneInterface &planning_scene) {
    ROS_INFO("Placing Tray at index: %d", req.index);
    place_sequence(kr20_arm_group, kr120_arm_group, kr120_gripper_group, planning_scene);
    res.success = true;
    return true;
}

template <typename Container>
    void print_array(const std::string& name, const Container& arr) {
        std::ostringstream oss;
        oss << name << ": [";
        for (size_t i = 0; i < arr.size(); ++i) {
            oss << arr[i];
            if (i < arr.size() - 1) oss << ", ";
        }
        oss << "]";
        ROS_INFO_STREAM(oss.str());
    }

bool handleInitParameters(
    const custom_interfaces::InitParameters::Request &req,
    custom_interfaces::InitParameters::Response &res,
    moveit::planning_interface::PlanningSceneInterface &planning_scene)
{
    ROS_INFO("✅ Received InitParameters from Python");

    // === DEBUG: Print received values ===
    ROS_INFO("📦 Received tray_step_poses (%lu values)", req.tray_step_poses.size());
    for (size_t i = 0; i < req.tray_step_poses.size(); ++i) {
        ROS_INFO("  tray_step_poses[%lu] = %f", i, req.tray_step_poses[i]);
    }


    print_array("tray_final_pose", req.tray_final_pose);
    print_array("tray_pose_operator", req.tray_pose_operator);
    print_array("table_pose", req.table_pose);
    print_array("tray_init_pose", req.tray_init_pose);
    print_array("bottom_init_pose", req.bottom_init_pose);
    print_array("recharge_pose", req.recharge_pose);
    print_array("rest_pose", req.rest_pose);
    print_array("pick_pose", req.pick_pose);
    print_array("pre_pick_pose", req.pre_pick_pose);
    print_array("post_pick_pose", req.post_pick_pose);
    print_array("bottom_pick_pose", req.bottom_pick_pose);
    print_array("end_pose", req.end_pose);
    print_array("pre_end_pose", req.pre_end_pose);
    print_array("tip_offset", req.tip_offset);
    print_array("tray_heights", req.tray_heights);
    print_array("origin_to_bottom", req.origin_to_bottom);
    print_array("tray_init_offset", req.tray_init_offset);

    // Angles
    ROS_INFO("tray_angle1: %f", req.tray_angle1);
    ROS_INFO("tray_angle2: %f", req.tray_angle2);

    // Indices
    for (size_t i = 0; i < req.tray_down_steps.size(); ++i)
        ROS_INFO("tray_down_steps[%lu] = %d", i, req.tray_down_steps[i]);

    for (size_t i = 0; i < req.operator_steps.size(); ++i)
        ROS_INFO("operator_steps[%lu] = %d", i, req.operator_steps[i]);

    for (size_t i = 0; i < req.rotation_steps.size(); ++i)
        ROS_INFO("rotation_steps[%lu] = %d", i, req.rotation_steps[i]);

    for (size_t i = 0; i < req.new_tray_steps.size(); ++i)
        ROS_INFO("new_tray_steps[%lu] = %d", i, req.new_tray_steps[i]);

    // Dummy name
    ROS_INFO("new_dummy: %s", req.new_dummy.c_str());

    // current_tray_units.clear();
    // for (size_t i = 0; i < req.tray_unit_names.size(); ++i) {
    //     Unit u;
    //     u.name = req.tray_unit_names[i];
    //     u.pose_index = (i < req.tray_unit_pose_indices.size()) ? req.tray_unit_pose_indices[i] : -1;
    //     current_tray_units.push_back(u);
    // }



    // --------- Tray Step Poses -----------
    if (req.tray_step_poses.size() != 16 * 6) {
        ROS_ERROR("Expected 96 tray_step_poses, got %ld", req.tray_step_poses.size());
        res.success = false;
        return true;
    }
    for (size_t i = 0; i < 16; ++i) {
        for (size_t j = 0; j < 6; ++j) {
            value_tray_pose[i][j] = req.tray_step_poses[i * 6 + j];
        }
    }
    //this prints the tray step poses //
    ROS_INFO("=== Step-poses received from Python (index → [x y z rx ry rz]) ===");
    for (size_t i = 0; i < 16; ++i)
    {
        std::ostringstream line;
        line << std::fixed << std::setprecision(4)
            << std::setw(2) << i << ": [";
        for (size_t j = 0; j < 6; ++j)
        {
            line << value_tray_pose[i][j];
            if (j < 5) line << ", ";
        }
        line << "]";
        ROS_INFO_STREAM(line.str());
    }
    ROS_INFO("================================================================");

    // for debug only one unit//

    // int idx = poseIndexFromName("CMG1");   // returns 4
    // ROS_INFO_STREAM("CMG1 uses pose-index "<<idx<<" → "
    //                 << vectorToString(value_tray_pose[idx]));


    table_pose = pose_from_array(req.table_pose);
    tray_init_pose = pose_from_array(req.tray_init_pose);
    bottom_init_pose = pose_from_array(req.bottom_init_pose);
    recharge_pose = pose_from_array(req.recharge_pose);
    rest_pose = pose_from_array(req.rest_pose);
    pick_pose = pose_from_array(req.pick_pose);
    pre_pick_pose = pose_from_array(req.pre_pick_pose);
    post_pick_pose = pose_from_array(req.post_pick_pose);
    bottom_pick_pose = pose_from_array(req.bottom_pick_pose);
    end_pose = pose_from_array(req.end_pose);
    pre_end_pose = pose_from_array(req.pre_end_pose);

    // --------- Poses and Offsets -----------
    std::copy(req.tray_final_pose.begin(), req.tray_final_pose.end(), tray_final_pose.begin());
    std::copy(req.tray_pose_operator.begin(), req.tray_pose_operator.end(), tray_pose_operator.begin());
    std::copy(req.tip_offset.begin(), req.tip_offset.end(), tip_position.begin());

    // --------- Angles -----------
    tray_angle1 = req.tray_angle1;
    tray_angle2 = req.tray_angle2;

    // --------- Tray Steps -----------
    tray_down_indeces = req.tray_down_steps;
    operator_indeces = req.operator_steps;
    horiz_rotation_indeces = req.rotation_steps;
    new_tray_indeces = req.new_tray_steps;

    // --------- Dummy Tray Name -----------
    new_dummy = req.new_dummy;

    
    //--------------------------------------

    geometry_msgs::Pose dummy_hole;
    dummy_hole.orientation = rpy_to_quat(0,0,0);
    dummy_hole.position.x = dummy_hole.position.y = dummy_hole.position.z = 0;
    compute_screw_pose2(pre_recharge, recharge_pose, dummy_hole);
    compute_screw_pose(pre_recharge, recharge_pose, -1);
    recharge_pose.position.z += 0.02;
    ext_screw_tray_pose=tray_final_pose;

    // --------- Tray Heights and Origins -----------

    ROS_INFO("tray_heights received (size=%ld):", req.tray_heights.size());
    for (size_t i = 0; i < req.tray_heights.size(); ++i) {
        ROS_INFO("  tray_heights[%ld] = %f", i, req.tray_heights[i]);
    }

    ROS_INFO("origin_to_bottom received (size=%ld):", req.origin_to_bottom.size());
    for (size_t i = 0; i < req.origin_to_bottom.size(); ++i) {
        ROS_INFO("  origin_to_bottom[%ld] = %f", i, req.origin_to_bottom[i]);
    }

    ROS_INFO_STREAM("end orientation: "
        << "x=" << end_pose.orientation.x
        << ", y=" << end_pose.orientation.y
        << ", z=" << end_pose.orientation.z
        << ", w=" << end_pose.orientation.w);
    
    ROS_INFO_STREAM("====[ C++ Side: tray_step_poses RECEIVED FROM PYTHON ]====");
    ROS_INFO_STREAM("Count received: " << req.tray_step_poses.size());

    std::stringstream ss;
    ss << "[";

    for (size_t i = 0; i < req.tray_step_poses.size(); ++i) {
        ss << req.tray_step_poses[i];
        if (i < req.tray_step_poses.size() - 1)
            ss << ", ";
    }

    ss << "]";

    ROS_INFO_STREAM(ss.str());


    if (req.tray_heights.size() >= 3 && req.origin_to_bottom.size() >= 1) {
        height_tray = req.tray_heights[0];
        height_dhc = req.tray_heights[1];
        height_payload = req.tray_heights[2];

        orig_bott_dist_aoc = req.origin_to_bottom[0];
        orig_bott_dist_dhc = orig_bott_dist_aoc + height_tray;
        orig_bott_dist_eps = orig_bott_dist_dhc + height_dhc;
        orig_bott_dist_payload = orig_bott_dist_eps + height_tray;

        displ_aoc = 2 * orig_bott_dist_aoc + height_tray;
        displ_eps = 2 * orig_bott_dist_aoc + 2 * height_tray + height_dhc;
    } else {
        ROS_ERROR("Invalid tray_heights or origin_to_bottom size.");
        res.success = false;
        return true;
    }

    // --------- Tray Init Pose Offset from Table -----------
    if (req.tray_init_offset.size() >= 2) {
        tray_init_pose.position.x = table_pose.position.x + req.tray_init_offset[0];
        tray_init_pose.position.y = table_pose.position.y + req.tray_init_offset[1];
    } else {
        ROS_ERROR("tray_init_offset must have at least 2 values.");
        res.success = false;
        return true;
    }
    // --------- Tray Units and Screws -----------
    ROS_INFO("Tray units received (count=%lu):", req.tray_unit_names.size());
    for (size_t i = 0; i < req.tray_unit_names.size(); ++i) {
        ROS_INFO("  Unit %lu: name = %s, pose_index = %d",
            i,
            req.tray_unit_names[i].c_str(),
            (i < req.tray_unit_pose_indices.size() ? req.tray_unit_pose_indices[i] : -1)
        );
    }

    ROS_INFO("Screws received (count=%lu):", req.screw_quantities.size());
    for (size_t i = 0; i < req.screw_quantities.size(); ++i) {
        ROS_INFO("  Screw %lu: quantity = %d, type = %s",
            i,
            req.screw_quantities[i],
            (i < req.screw_types.size() ? req.screw_types[i].c_str() : "UNKNOWN")
        );
    }
   // ─── Rebuild our runtime vectors ───────────────────────────────
    {
    // 1) current_tray_units from parallel name/index arrays
    current_tray_units.clear();
    current_tray_units.reserve(req.tray_unit_names.size());
    for (size_t i = 0; i < req.tray_unit_names.size(); ++i)
    {
        int poseIdx = (i < req.tray_unit_pose_indices.size())
                        ? req.tray_unit_pose_indices[i]
                        : -1;
        current_tray_units.push_back( Unit{ req.tray_unit_names[i], poseIdx } );
    }
    ROS_INFO("✅ current_tray_units rebuilt:");
    for (auto &u : current_tray_units)
        ROS_INFO("   • %-10s → pose_index %d", u.name.c_str(), u.pose_index);

    // 2) tray_down_steps: map incoming int indices → unit names
    tray_down_steps.clear();
    tray_down_steps.reserve(req.tray_down_steps.size());
    for (int idx : req.tray_down_steps)
    {
        auto it = std::find(
        req.tray_unit_pose_indices.begin(),
        req.tray_unit_pose_indices.end(),
        idx);

        if (it != req.tray_unit_pose_indices.end())
        {
        size_t unit_i = std::distance(
            req.tray_unit_pose_indices.begin(), it);
        tray_down_steps.push_back(
            req.tray_unit_names[unit_i]);
        }
        else
        {
        ROS_WARN("tray_down_steps contains unknown pose_index %d", idx);
        }
    }
    ROS_INFO("✅ tray_down_steps rebuilt:");
    for (auto &n : tray_down_steps)
        ROS_INFO("   • %s", n.c_str());
    }
// ────────────────────────────────────────────────────────────────


    /// Store raw arrays for any other code that still uses them
    tray_unit_names        = req.tray_unit_names;
    tray_unit_pose_indices = req.tray_unit_pose_indices;
    screw_quantities       = req.screw_quantities;
    screw_types            = req.screw_types;
    // ✅ Store for use in other parts of the program
    tray_unit_names = req.tray_unit_names;
    tray_unit_pose_indices = req.tray_unit_pose_indices;
    screw_quantities = req.screw_quantities;
    screw_types = req.screw_types;


    res.success = true;
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
    
    // if (argc == 1) {
    //     // read entire stdin as JSON
    //     std::ostringstream buf;
    //     buf << std::cin.rdbuf();
    //     auto raw = buf.str();

    //     try {
    //         g_screw_cfg = loadScrewConfigFromString(raw);
    //     } catch (const std::exception &e) {
    //         std::cerr << "❌ Failed to parse JSON on stdin: " << e.what() << "\n";
    //         return 1;
    //     }

    //     // dump what we found
    //     for (auto &kv : g_screw_cfg) {
    //         std::cerr << "🔩 Loaded " << kv.second.size()
    //                   << " screws for tray: " << kv.first << "\n";
    //     }
    //     // exit immediately
    //     return 0;
    // }
    // 1) JSON-only entrypoint
    // --------------------------------------------------------------------------
    // if (argc != 2) {
    //     std::cerr << "Usage: " << argv[0] << " <screw_config.json>\n";
    //     return 1;
    // }
    // try {
    //     g_screw_cfg = loadScrewConfig(argv[1]);
    // } catch (std::exception &e) {
    //     std::cerr << "❌ Failed to load screw_config.json: " << e.what() << "\n";
    //     return 1;
    // }
    // // dump what we found, so you can verify your tray names
    // for (auto &kv : g_screw_cfg) {
    //     std::cerr << "🔩 Loaded " << kv.second.size()
    //             << " screws for tray: " << kv.first << "\n";
    // }
    // // dump every screw we just loaded
    // for (const auto &[tray, screws] : g_screw_cfg) {
    // ROS_INFO("🔩 Tray \"%s\" has %zu screws:", tray.c_str(), screws.size());
    // for (const auto &s : screws) {
    //     const auto &p = s.pose.position;
    //     ROS_INFO("    index=%3d   mode=%s   pos=[%.3f, %.3f, %.3f]",
    //             s.index,
    //             s.mode.c_str(),
    //             p.x, p.y, p.z);
    // }
    // }



    // Extract file name from command arguments
    if (argc != 3) { 
        std::cerr << "Utilizzo: " << argv[0] << " <nome_file>" << " <nome_file>" << std::endl;
        return 1;
    }
    file_pose = argv[1];
    file_order = argv[2];

    target_poses =extract_poses_from_file(file_pose);   
    target_order =extract_order_from_file(file_order);   

    ros::init(argc, argv, "integration");
    ros::NodeHandle nh;

    moveit::planning_interface::PlanningSceneInterface planning_scene;

    spawn_client = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");

    scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);    
    scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
    attach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/attach");
    detach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/detach");
    pub = nh.advertise<std_msgs::Int32>("action_completed", 1);
    robot_state_sub = nh.subscribe("/kr20_kr120/joint_states", 1000, robotStateCallback);
    ros::Rate rate(1); 

    sleep(2.0);

    set_constraints(groups_constraints);

    moveit::planning_interface::MoveGroupInterface kr20_arm_group("kr20_arm");
    moveit::planning_interface::MoveGroupInterface kr20_slide_group("kr20_slide");
    moveit::planning_interface::MoveGroupInterface kr120_arm_group("kr120_arm");
    moveit::planning_interface::MoveGroupInterface kr120_gripper_group("kr120_gripper");
    moveit::planning_interface::MoveGroupInterface rl_lifter_group("rl_lifter");
    moveit::planning_interface::MoveGroupInterface rl_slider_group("rl_slider");
    moveit::planning_interface::MoveGroupInterface rl_plate_group("rl_plate");
    
    kr20_arm_group.setNumPlanningAttempts(3);
    kr20_arm_group.setPathConstraints(groups_constraints[0]);
    kr20_slide_group.setNumPlanningAttempts(3);
    kr120_arm_group.setNumPlanningAttempts(3);
    kr120_arm_group.setPathConstraints(groups_constraints[1]);
    kr120_gripper_group.setNumPlanningAttempts(3);

    add_table(planning_scene);

    tray_map= { {"tray_aoc", tray_components_aoc}, {"tray_dhc", tray_dhc}, {"tray_eps", tray_components_eps},  
                {"tray_payload", tray_payload} }; 

    target_order_index=(start_index<8) ? start_index-1 : start_index-2;

    // auto cb = boost::bind(
    // &handleScrewingUnit,
    // _1, _2,
    // boost::ref(kr20_arm_group),
    // boost::ref(kr20_slide_group)
    // );
    //  lambda function to pass objects to the service
    ros::ServiceServer service_init_parameters = nh.advertiseService<custom_interfaces::InitParameters::Request, custom_interfaces::InitParameters::Response>(
        "init_parameters", [&](auto &req, auto &res) { return handleInitParameters(req, res, planning_scene); });   

    ros::ServiceServer service_add_tray = nh.advertiseService<custom_interfaces::AddTray2::Request, custom_interfaces::AddTray2::Response>(
        "add_tray2", [&](auto &req, auto &res) { return handleAddTray(req, res, planning_scene); });
    
    ros::ServiceServer service_pick_tray = nh.advertiseService<custom_interfaces::PickTray::Request, custom_interfaces::PickTray::Response>(
        "pick_tray", [&](auto &req, auto &res) { return handlePickTray(req, res, kr120_arm_group, kr120_gripper_group, planning_scene); });
    
    ros::ServiceServer service_position_tray = nh.advertiseService<custom_interfaces::PositionTray::Request, custom_interfaces::PositionTray::Response>(
        "position_tray", [&](auto &req, auto &res) { return handlePositionTray(req, res, kr120_arm_group, kr120_gripper_group); });
    
    ros::ServiceServer service_operator_position_tray = nh.advertiseService<custom_interfaces::OperatorPositionTray::Request, custom_interfaces::OperatorPositionTray::Response>(
        "operator_position_tray", [&](auto &req, auto &res) { return handleOperatorPositionTray(req, res, kr120_arm_group, kr120_gripper_group, kr20_arm_group); });
    
    ros::ServiceServer service_recharge = nh.advertiseService<custom_interfaces::RechargeSequence::Request, custom_interfaces::RechargeSequence::Response>(
        "recharge_sequence", [&](auto &req, auto &res) { return handleRechargeSequence(req, res, kr20_arm_group, kr20_slide_group); });
   
    ros::ServiceServer service_internal_screwing = nh.advertiseService<custom_interfaces::InternalScrewingSequence::Request, custom_interfaces::InternalScrewingSequence::Response>(
        "internal_screwing_sequence", [&](auto &req, auto &res) { return handleInternalScrewingSequence(req, res, kr20_arm_group, kr20_slide_group); });
    
    ros::ServiceServer service_internal_screw = nh.advertiseService<custom_interfaces::InternalScrewByNum::Request, custom_interfaces::InternalScrewByNum::Response>(
        "internal_screw_by_num", [&](auto &req, auto &res) {return handleInternalScrewByNum(req, res, kr20_arm_group, kr20_slide_group);});

    // ros::ServiceServer service_screwing_unit = nh.advertiseService<custom_interfaces::ScrewingUnit::Request, custom_interfaces::ScrewingUnit::Response>(
    //     "screwing_unit", [&](auto &req, auto &res) { return handleScrewingUnit(req, res, kr20_arm_group, kr20_slide_group); });
    
    ros::ServiceServer service_external_screwing = nh.advertiseService<custom_interfaces::ExternalScrewingSequence::Request, custom_interfaces::ExternalScrewingSequence::Response>(
        "external_screwing_sequence",
        [&](auto &req, auto &res) { return handleExternalScrewingSequence(req, res, kr20_arm_group, kr20_slide_group, rl_plate_group, planning_scene); });
    
    ros::ServiceServer service_place_tray = nh.advertiseService<custom_interfaces::PlaceTray::Request, custom_interfaces::PlaceTray::Response>(
        "place_tray",
        [&](auto &req, auto &res) { return handlePlaceTray(req, res, kr120_arm_group, kr120_gripper_group, kr20_arm_group, planning_scene); });
    
    ROS_INFO("All services are ready to receive requests.");

    // Start an asynchronous spinner with a suitable number of threads
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // Do not call ros::spin() here because spinner.start() already spins the queue.
    ros::waitForShutdown(); // This will block until shutdown

    return 0;

}










