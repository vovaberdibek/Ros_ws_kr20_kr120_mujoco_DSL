#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometric_shapes/shape_operations.h"
#include <fstream>
#include <std_msgs/Int32.h>
#include <cmath>
#include <gazebo_ros_link_attacher/Attach.h>
#include <actionlib/server/simple_action_server.h>
#include "custom_interfaces/ExecuteSequenceAction.h"
#include "custom_interfaces/PositionTray.h"
#include "custom_interfaces/AddTray.h"
#include "custom_interfaces/PickTray.h"
#include "custom_interfaces/OperatorPositionTray.h"
#include "custom_interfaces/RechargeSequence.h"
#include "custom_interfaces/InternalScrewingSequence.h"
#include "custom_interfaces/ExternalScrewingSequence.h"
#include "custom_interfaces/PlaceTray.h"


const double pi=3.141592653589793;
int start_index=0; // Initial sequence step

double tray_angle1=pi/10; // angle wrt z axis for tray positioning
double tray_angle2=pi/4; // angle wrt z axis for tray positioning
double height_tray=0.25; //aoc eps tray height
double height_dhc=0.2515; //dhc tray height 
double height_payload=0.2; //payload tray height 
double orig_bott_dist_aoc=0.056; //distance between origin of RF and the bottom of the tray
double orig_bott_dist_dhc=orig_bott_dist_aoc+height_tray;  //distance between origin of RF and the bottom of the tray
double orig_bott_dist_eps=orig_bott_dist_dhc+height_dhc; //distance between origin of RF and the bottom of the tray
double orig_bott_dist_payload=orig_bott_dist_eps+height_tray; //distance between origin of RF and the bottom of the tray
double displ_aoc=2*orig_bott_dist_aoc+height_tray; //distance between the positions of RF origin for tray up and down
double displ_eps=2*orig_bott_dist_aoc+2*height_tray+height_dhc; //distance between the positions of RF origin for tray up and down

std::string file_pose;
std::string file_order;
std::vector< geometry_msgs::Pose > target_poses;   
std::vector<std::vector<int>> target_order;

moveit::planning_interface::MoveGroupInterface::Plan plan;
std::vector<moveit_msgs::Constraints> groups_constraints;

ros::Publisher scene_publisher;
ros::ServiceClient scene_client;
ros::ServiceClient attach_client;
ros::ServiceClient detach_client;
ros::Publisher pub;
ros::Subscriber robot_state_sub;

std::string tray_name;
std::vector<moveit_msgs::CollisionObject> tray_components;
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


// Helper to print geometry_msgs::Pose
void print_pose(const std::string& name, const geometry_msgs::Pose& pose) {
    ROS_INFO("%s: position=[%f, %f, %f], orientation=[%f, %f, %f, %f]",
        name.c_str(),
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

// Helper to convert std::vector to a string for easy printing
template<typename T>
std::string vector_to_string(const std::vector<T>& vec) {
    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        ss << vec[i];
        if (i < vec.size() - 1) ss << ", ";
    }
    ss << "]";
    return ss.str();
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

        std::vector<std::string> names=currentACM.entry_names;

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
void enable_component_collision(int count){
    moveit_msgs::PlanningScene currentScene;
    moveit_msgs::PlanningScene newSceneDiff;
    moveit_msgs::GetPlanningScene scene_srv;

    scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;

    if(!scene_client.call(scene_srv)){
        ROS_WARN("Failed to call service /get_planning_scene");
    }else{
        currentScene = scene_srv.response.scene;

        moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;

        std::vector<std::string> names=currentACM.entry_names;

        int slide_index= get_index(names,"kr20_slide");
        int fixed_index= get_index(names,"kr20_fixed");
        int component_index;

        if(tray_name=="tray_aoc"){
            switch (count){
            case 1:
                component_index = get_index(names,"MTQ12");
                break;
            case 2:
                component_index = get_index(names,"MTQ3_MAG");
                break;
            case 3:
                component_index = get_index(names,"CMG2");
                break;
            case 4:
                component_index = get_index(names,"CMG1");
                break;
            case 5:
            component_index = get_index(names,"CMG34");
            break;
            default:
                component_index = -1;
                break;
            }
        }else if(tray_name=="tray_eps"){
            switch (count){
            case 1:
                component_index = get_index(names,"BAT1");
                break;
            case 2:
                component_index = get_index(names,"BAT2");
                break;
            case 3:
                component_index = get_index(names,"PCDU");
                break;
            default:
                component_index = -1;
                break;
            }
        }

        currentACM.entry_values[slide_index].enabled[component_index]=false;
        currentACM.entry_values[component_index].enabled[slide_index]=false;
        currentACM.entry_values[fixed_index].enabled[component_index]=false;
        currentACM.entry_values[component_index].enabled[fixed_index]=false;
     
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
}

// Reach the place pose, open the gipper and enable again collision check for the tray
void place(moveit::planning_interface::MoveGroupInterface& arm_group,moveit::planning_interface::MoveGroupInterface& gripper_group, geometry_msgs::Pose& place_pose,std::vector<moveit_msgs::CollisionObject>& tray_components,std::string attach_name,moveit::planning_interface::PlanningSceneInterface& planning_scene){
    execute_lin_path(place_pose,arm_group);

    // gazebo_detach("kr20_kr120", "kr120_gyro_left", tray_name, "link_0");

    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "kr120_gyro_left";

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
void add_tray(std::string tray_name, std::vector<moveit_msgs::CollisionObject>& tray_components,moveit::planning_interface::PlanningSceneInterface& planning_scene){  
    // Add the tray
    static const Eigen::Vector3d scale(0.001, 0.001, 0.001);


    // if(tray_name=="tray_aoc"){
    //     tray_components.resize(5);
    //     tray_components[1].id="MTQ_MAG";
    //     tray_components[2].id="CMG2";
    //     tray_components[3].id="CMG1";
    //     tray_components[4].id="CMG34";
    // }else if(tray_name=="tray_eps"){
    //     tray_components.resize(4);
    //     tray_components[1].id="BAT1";
    //     tray_components[2].id="BAT2";
    //     tray_components[3].id="PCDU";
    // }else{
    //     tray_components.resize(1);
    // }
    // tray_components[0].id = tray_name;
    if(tray_name=="tray_aoc"){
        tray_components.resize(6);
        tray_components[1].id="MTQ12";
        tray_components[2].id="MTQ3_MAG";
        tray_components[3].id="CMG2";
        tray_components[4].id="CMG1";
        tray_components[5].id="CMG34";
    }else if(tray_name=="tray_eps"){
        tray_components.resize(4);
        tray_components[1].id="BAT1";
        tray_components[2].id="BAT2";
        tray_components[3].id="PCDU";
    }else{
        tray_components.resize(1);
    }
    tray_components[0].id = tray_name;

    
    int n_prev_components=1;
    if(collision_objects.back().id=="bottom_panel") n_prev_components=6;
    else if(collision_objects.back().id=="PCDU") n_prev_components=4;
    int coll_obj_size=collision_objects.size();
    for (int i = coll_obj_size-n_prev_components; i < coll_obj_size; i++){
        collision_objects[i].pose.orientation=rpy_to_quat(tray_final_pose[0],tray_final_pose[1],tray_final_pose[2]);//tray_init_pose.orientation;
        collision_objects[i].pose.position.x = tray_final_pose[3];//0.0;
        collision_objects[i].pose.position.y = tray_final_pose[4];//-1.430;
        collision_objects[i].pose.position.z=tray_final_pose[5];//0.700675-orig_bott_dist_aoc;
    }

    for (int i = 0; i < tray_components.size(); i++){
        tray_components[i].header.frame_id = "world";
        shapes::Mesh* m = shapes::createMeshFromResource("package://kr20_kr120/collision/"+ tray_components[i].id + ".stl",scale);
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(m,mesh_msg);
        shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
        tray_components[i].meshes.resize(1);
        tray_components[i].meshes[0] = mesh;

        tray_components[i].meshes.push_back(mesh);
        tray_components[i].operation = tray_components[i].ADD;
        tray_components[i].pose=(tray_name!="bottom_panel") ? tray_init_pose : bottom_init_pose;
    }

    moveit_msgs::PlanningScene planning_scene_add;
    
    for (int i = 0; i < tray_components.size(); i++){
        collision_objects.push_back(tray_components[i]);
        planning_scene.applyCollisionObject(tray_components[i]);
        planning_scene_add.world.collision_objects.push_back(tray_components[i]);
    }

    planning_scene_add.is_diff = true;
    scene_publisher.publish(planning_scene_add);

    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components =  moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;
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
void attach_tray(std::string link,moveit::planning_interface::PlanningSceneInterface& planning_scene){
    
    int n_prev_components=1;
    if(collision_objects.back().id=="bottom_panel") n_prev_components=6;
    else if(collision_objects.back().id=="PCDU") n_prev_components=4;
    int coll_obj_size=collision_objects.size();
    for (int i = coll_obj_size-n_prev_components; i < coll_obj_size; i++){
        collision_objects[i].pose.orientation=rpy_to_quat(tray_final_pose[0],tray_final_pose[1],tray_final_pose[2]);
        collision_objects[i].pose.position.x = tray_final_pose[3];
        collision_objects[i].pose.position.y = tray_final_pose[4];
        collision_objects[i].pose.position.z=tray_final_pose[5];
    }
    
    
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = link;

    for (int i = 1; i < collision_objects.size(); i++){
        collision_objects[i].operation = collision_objects[i].ADD;
        attached_object.object = collision_objects[i];
        planning_scene.applyAttachedCollisionObject(attached_object);
    }
    sleep(1);
}

// Detach the tray to the given link
void detach_tray(std::string link,moveit::planning_interface::PlanningSceneInterface& planning_scene){
    int n_prev_components=1;
    if(collision_objects.back().id=="CMG34") n_prev_components=5;
    else if(collision_objects.back().id=="PCDU") n_prev_components=4;
    int coll_obj_size=collision_objects.size();
    for (int i = coll_obj_size-n_prev_components; i < coll_obj_size; i++){
        collision_objects[i].pose.orientation=rpy_to_quat(tray_final_pose[0],tray_final_pose[1],tray_final_pose[2]);
        collision_objects[i].pose.position.x = tray_final_pose[3];
        collision_objects[i].pose.position.y = tray_final_pose[4];
        collision_objects[i].pose.position.z=tray_final_pose[5];
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

// Send the message to spawn the tray in gazebo
void spawn_message(int index){
    spawn_msg.data = index;
    pub.publish(spawn_msg);
    sleep(1.0);
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

    kr120_target_pose=kr120_flange_target_pose(tray_pose_operator,tray_flange_dist_op,tray_flange_angle_op);
    execute_rrt(kr120_target_pose,kr120_arm_group);

    sleep(5.0);

    spawn_message(20+i);

    gazebo_attach(tray_name, "link_0", new_dummy, "link_0");
    if(i>1){
        if(contains(horiz_rotation_indeces, i)) rotate_flange(kr120_arm_group,kr120_target_pose,pi);

        set_joints(kr120_gripper_group,gyro_joints,{(pi-gyro_target),-(pi-gyro_target)});

        
        // gazebo_attach(tray_name, "link_0", tray_components[i].id, "link_0");

    }else{    
        tray_pose_operator[0]=value_tray_pose[0][0];
        tray_pose_operator[1]=value_tray_pose[0][1];

        kr120_target_pose=kr120_flange_target_pose(tray_pose_operator,tray_flange_dist_op,tray_flange_angle_op);
        execute_rrt(kr120_target_pose,kr120_arm_group);

        sleep(3.0);

        spawn_message(22+i+1);

        gazebo_attach(tray_name, "link_0", "MTQ12", "link_0");
    }

    enable_component_collision(count_components);
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

// Perform complete sequence for screwing
void screwing_sequence(moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& slide_group, int begin, int end, int i){
    for (int k = begin; k < end; k++){
        if(begin>0){
            target_index = k;
            kr20_target_pose=kr20_flange_target_pose(target_index,ext_screw_tray_pose);
        }else{
            target_index = target_order[target_order_index][k];
            kr20_target_pose=kr20_flange_target_pose(target_index,value_tray_pose[i]);
        }
        
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

// Perform complete sequence for the internal screws
void internal_screwing_sequence(moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& slide_group, int i){
    target_order_index++;

    screwing_sequence(arm_group, slide_group, 0, target_order[target_order_index].size(),i);
}

// Perform complete sequence for tray placing on the roller
void place_sequence(moveit::planning_interface::MoveGroupInterface& kr20_arm_group, moveit::planning_interface::MoveGroupInterface& kr120_arm_group, moveit::planning_interface::MoveGroupInterface& kr120_gripper_group, moveit::planning_interface::PlanningSceneInterface& planning_scene){
    execute_lin_path(recharge_pose,kr20_arm_group);
             
    execute_rrt(post_pick_pose,kr120_arm_group);
    
    if(tray_name=="tray_aoc"){
        add_tray("bottom_panel",bottom_panel,planning_scene);
        
        spawn_message(50);
        
        gazebo_attach("kr20_kr120", "table", "bottom_panel", "link_0");
        
        place(kr120_arm_group,kr120_gripper_group, bottom_pick_pose,tray_map[tray_name],"bottom_panel",planning_scene); // Place the tray
        
        execute_rrt(post_pick_pose,kr120_arm_group);
        
        set_joints(kr120_gripper_group,plier_joints,{-0.035,0.035}); // Open kr120 gripper
        
        gazebo_detach("kr20_kr120", "table", "bottom_panel", "link_0"); 
                
        pick(kr120_arm_group,tray_map[tray_name],count_components,bottom_pick_pose); // Move kr120 to the pick pose

        std::cout << bottom_panel[0] << std::endl;
        
        tray_map[tray_name].push_back(bottom_panel[0]);
        
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
    
    place(kr120_arm_group,kr120_gripper_group, end_pose,tray_map[tray_name],"kr20_kr120",planning_scene); // Place the tray
    
    execute_lin_path(pre_end_pose,kr120_arm_group); // Move kr120 to the pre_end pose

    set_joints(kr120_gripper_group,plier_joints,{-0.035,0.035}); // Open kr120 gripper
    
    execute_lin_path(post_pick_pose,kr120_arm_group);
}

// Perform complete sequence for the external screws
void external_screwing_sequence(moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& slide_group, moveit::planning_interface::MoveGroupInterface& rl_plate_group, moveit::planning_interface::PlanningSceneInterface& planning_scene){
    attach_tray("rl_plate",planning_scene);
    
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
    attach_tray("rl_slider",planning_scene);
    set_joints(rl_lifter_group,{"rl_base__lifter"},{lift_target_1});
    sleep(1.0);
    set_joints(rl_slider_group,{"rl_base__slider"},{slide_target_1});
    sleep(1.0);
    set_joints(rl_slider_group,{"rl_base__slider"},{slide_target_2});
    sleep(1.0);
    set_joints(rl_lifter_group,{"rl_base__lifter"},{lift_target_2});
}

// Set parameters depending on the tray
void tray_setup(int i){
    if(i<=7){
        tray_name="tray_aoc";
        pick_pose.position.z = table_pose.position.z+0.82025+height_tray+0.4795-0.0015;
        end_pose.position.z = tray_final_pose[5]+orig_bott_dist_aoc+height_tray+0.4795+0.001;
        tray_init_pose.position.z = table_pose.position.z+0.82025-orig_bott_dist_aoc;
        ext_screw_tray_pose[5] = tray_final_pose[5];
        if(contains(tray_down_indeces, i)){
            tray_flange_dist_pos={0,0,0.4795+height_tray+orig_bott_dist_aoc-0.002};
            tray_flange_angle_pos=0;
            if(i==0 || i==1){ 
                tray_flange_dist_op=tray_flange_dist_pos;
                tray_flange_angle_op=0;
            }else{ 
                tray_flange_dist_op = {0,0,0.4795 - orig_bott_dist_aoc};
                tray_flange_angle_op=pi;
            }
            gyro_target=pi;
        }else{ 
            tray_flange_dist_pos ={0,0,-( 0.4795 - orig_bott_dist_aoc-0.014)};
            tray_flange_dist_op={0,0,0.4795+height_tray+orig_bott_dist_aoc};
            tray_flange_angle_pos=pi;
            tray_flange_angle_op=0;
            gyro_target=0;
        }
        switch (i){
            case 0:
                new_dummy="MTQ12";
                // new_dummy="MTQ3_MAG";
                break;
            case 2:
                new_dummy="CMG2";
                break;
            case 4:
                new_dummy="CMG1";
                break;
            case 6:
                new_dummy="CMG34";
                break;    
            default:
                break;
        }
    }else if(i==8){
        tray_name="tray_dhc";
        pick_pose.position.z = table_pose.position.z+0.82025+height_dhc+0.4795-0.0015;
        end_pose.position.z = tray_final_pose[5]+orig_bott_dist_aoc+height_tray+height_dhc+0.4795+0.005;
        tray_init_pose.position.z = table_pose.position.z+0.82025-orig_bott_dist_dhc;
        ext_screw_tray_pose[5] = tray_final_pose[5]+height_tray;
    }else if(i<=14){  
        tray_name="tray_eps";
        pick_pose.position.z = table_pose.position.z+0.82025+height_tray+0.4795-0.0015;
        end_pose.position.z = tray_final_pose[5]+orig_bott_dist_aoc+2*height_tray+height_dhc+0.4795+0.015;
        tray_init_pose.position.z = table_pose.position.z+0.82025-orig_bott_dist_eps;
        ext_screw_tray_pose[5] = tray_final_pose[5]+height_tray+height_dhc;
        if(contains(tray_down_indeces, i)){
            tray_flange_dist_pos={0,0,0.4795+height_tray+orig_bott_dist_eps-0.002};
            tray_flange_angle_pos=0;
            tray_flange_dist_op ={0,0, 0.4795 - orig_bott_dist_aoc};
            tray_flange_angle_op=pi;
            gyro_target=pi;
        }else{ 
            tray_flange_dist_pos = {0,0,-( 0.4795 - orig_bott_dist_eps-0.014)};
            tray_flange_dist_op={0,0,0.4795+height_tray+orig_bott_dist_aoc};
            tray_flange_angle_pos=pi;
            tray_flange_angle_op=0;
            gyro_target=0;
        }
        switch (i){
            case 9:
                new_dummy="BAT1";
                break;
            case 11:
                new_dummy="BAT2";
                break;
            case 13:
                new_dummy="PCDU";
                break;
            default:
                break;
        }
    }else{
        tray_name="tray_payload";
        pick_pose.position.z = table_pose.position.z+0.82025+height_payload+0.4795-0.0015+0.05;
        end_pose.position.z = tray_final_pose[5]+orig_bott_dist_aoc+2*height_tray+height_dhc+height_payload+0.4795+0.02;
        tray_init_pose.position.z = table_pose.position.z+0.82025-orig_bott_dist_payload;
        ext_screw_tray_pose[5] = tray_final_pose[5]+2*height_tray+height_dhc;
    }
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

bool handleAddTray(custom_interfaces::AddTray::Request &req, custom_interfaces::AddTray::Response &res, moveit::planning_interface::PlanningSceneInterface &planning_scene) {
    ROS_INFO("Adding Tray at index: %d", req.index);
    tray_setup(req.index);

    if(contains(new_tray_indeces, req.index)){
        add_tray(tray_name,tray_map[tray_name],planning_scene);
        spawn_message(req.index);
    }

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
    res.success = true;
    return true;
}

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


//////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
    
    // Numerical tray poses for each step of the sequence
    value_tray_pose = {{
  /*0*/ {-pi/2,   0.0, -pi/2-tray_angle1, 1.20, -0.07, 1.75}, //MTQ1, MTQ2 
  /*1*/ {-pi/2, -pi/2, -pi/2-tray_angle1, 1.20, -0.07, 1.75}, //MTQ3, MAG
  /*2*/ {-pi/2,  pi/2, pi/2-tray_angle1, 1.20+displ_aoc*cos(tray_angle1), -0.07-displ_aoc*sin(tray_angle1), 1.75},//CMG2 UP
  /*3*/ {-pi/2,  pi/2, -pi/2-tray_angle1, 1.20, -0.07, 1.75},//CMG2 DOWN
  /*4*/ {-pi/2, -pi/2, pi/2-tray_angle1, 1.20+displ_aoc*cos(tray_angle1), -0.07-displ_aoc*sin(tray_angle1), 1.75},//CMG1 UP
  /*5*/ {-pi/2, -pi/2, -pi/2-tray_angle1, 1.20, -0.07, 1.75},//CMG1 DOWN
  /*6*/ {-pi/2,    pi, pi/2-tray_angle2, 1.20+displ_aoc*cos(tray_angle2), -0.07-displ_aoc*sin(tray_angle2), 1.75},//CMG34 UP
  /*7*/ {-pi/2,    pi, -pi/2-tray_angle2, 1.20, -0.07, 1.75},//CMG34 DOWN
  /*8*/ {-1,-1,-1,-1,-1,-1}, // DHC TRAY
  /*9*/ {-pi/2,  pi/2, pi/2-tray_angle1, 1.20+displ_eps*cos(tray_angle1), -0.07-displ_eps*sin(tray_angle1), 1.75},//BAT1 UP
 /*10*/ {-pi/2,  pi/2, -pi/2-tray_angle1, 1.20-(orig_bott_dist_eps-orig_bott_dist_aoc)*cos(tray_angle1), -0.07+(orig_bott_dist_eps-orig_bott_dist_aoc)*sin(tray_angle1), 1.75},//BAT1 DOWN
 /*11*/ {-pi/2, -pi/2, pi/2-tray_angle1, 1.20+displ_eps*cos(tray_angle1), -0.07-displ_eps*sin(tray_angle1), 1.75},//BAT2 UP
 /*12*/ {-pi/2, -pi/2, -pi/2-tray_angle1, 1.20-(orig_bott_dist_eps-orig_bott_dist_aoc)*cos(tray_angle1), -0.07+(orig_bott_dist_eps-orig_bott_dist_aoc)*sin(tray_angle1), 1.75},//BAT2 DOWN
 /*13*/ {-pi/2,    pi, pi/2-tray_angle2, 1.20+displ_eps*cos(tray_angle2), -0.07-displ_eps*sin(tray_angle2), 1.75},//PCDU UP
 /*14*/ {-pi/2,    pi, -pi/2-tray_angle2, 1.20-(orig_bott_dist_eps-orig_bott_dist_aoc)*cos(tray_angle2), -0.07+(orig_bott_dist_eps-orig_bott_dist_aoc)*sin(tray_angle2), 1.75},//PCDU DOWN
 /*15*/ {-1,-1,-1,-1,-1,-1} // PAYLOAD TRAY
    }};
    ROS_INFO("📦 Hardcoded value_tray_pose:");
    for (size_t i = 0; i < 16; ++i) {
        ROS_INFO("tray_pose[%ld]: [%f, %f, %f, %f, %f, %f]", i,
            value_tray_pose[i][0],
            value_tray_pose[i][1],
            value_tray_pose[i][2],
            value_tray_pose[i][3],
            value_tray_pose[i][4],
            value_tray_pose[i][5]);
    }

    new_tray_indeces = {start_index,8,9,15,16}; // First index of a tray
    operator_indeces = {0,2,4,6,9,11,13}; // When the tray need to be placed to add other components
    horiz_rotation_indeces = {6,7,13,14}; // Steps in which tray rotation is around horizontal axis
    change_pose_indeces = {0,1,2,4,6,9,11,13}; // When the tray need to be moved to another pose 
    tray_down_indeces = {0,1,3,5,7,10,12,14}; // Steps in which tray is oriented with the bottom towards the screwing robot

    tip_position = {-0.299605,0,0.260354}; // Position of the tool tip wrt the flange RF

    // Table pose
    table_pose.orientation = rpy_to_quat(0,0,-pi/2); 
    table_pose.position.x = 1.252;
    table_pose.position.y = -0.12;
    table_pose.position.z = 0.004;

    // Tray initial pose
    tray_init_pose.orientation= rpy_to_quat(0,0,pi/2); 
    tray_init_pose.position.x = table_pose.position.x-0.24184;
    tray_init_pose.position.y = table_pose.position.y+0.23975;

    // Bottom panel initial pose
    bottom_init_pose=tray_init_pose;
    bottom_init_pose.position.z = table_pose.position.z+0.82025+0.285;

    // Recharge pose
    recharge_pose.orientation = rpy_to_quat(0,0,0);
    recharge_pose.position.x = table_pose.position.x-0.55160+tip_position[0];               
    recharge_pose.position.y = table_pose.position.y-0.17248+tip_position[1];
    recharge_pose.position.z = table_pose.position.z+0.82025+0.051+tip_position[2];

    // Pre-recharge pose
    compute_screw_pose(pre_recharge,recharge_pose,-1);

    recharge_pose.position.z = recharge_pose.position.z +0.02;

    // Rest pose
    rest_pose.orientation = rpy_to_quat(0,0,-pi/4); 
    rest_pose.position.x = 0.8;             
    rest_pose.position.y = -0.8;
    rest_pose.position.z = 0.81+tip_position[2]+0.3;

    // Pick pose
    pick_pose.orientation = rpy_to_quat(pi/2,pi/2, pi/2);
    pick_pose.position.x = tray_init_pose.position.x;
    pick_pose.position.y = tray_init_pose.position.y;
    std::cout << "PickPose_orientation" << std::endl;

    std::cout << pick_pose.orientation << std::endl;

    // Pick pose aoc+bottom panel
    bottom_pick_pose = pick_pose;
    bottom_pick_pose.position.z = table_pose.position.z+0.82025+height_tray+orig_bott_dist_aoc+0.4795-0.0015+0.285+0.001;

    //Pre-pick pose
    pre_pick_pose.orientation = rpy_to_quat(pi/2,pi/2, pi/2);//-0.025
    pre_pick_pose.position.x = pick_pose.position.x;
    pre_pick_pose.position.y = pick_pose.position.y;
    pre_pick_pose.position.z = table_pose.position.z+0.82025+height_tray+0.4795+0.4;

    // Post-pick pose
    post_pick_pose.orientation = pre_pick_pose.orientation;
    post_pick_pose.position.x = pick_pose.position.x;
    post_pick_pose.position.y = pick_pose.position.y;
    post_pick_pose.position.z = table_pose.position.z+0.82025+height_tray+0.4795+0.8; 

    // Tray pose for operator
    tray_pose_operator={0,0,pi,tray_init_pose.position.x,1,1.55};

    // Tray final pose
    tray_final_pose={0,0,pi/2,0,-1.43,0.579+0.117+0.055};//0.700675-orig_bott_dist_aoc};

    // Pose for external screw
    ext_screw_tray_pose=tray_final_pose;
    
    // End pose
    end_pose.orientation = post_pick_pose.orientation;
    end_pose.position.x = 0;
    end_pose.position.y = -1.430;

    // Pre-end pose
    pre_end_pose.orientation = post_pick_pose.orientation;
    pre_end_pose.position.x = end_pose.position.x;
    pre_end_pose.position.y = end_pose.position.y;
    pre_end_pose.position.z = tray_final_pose[5]+2*height_tray+height_dhc+height_payload+0.4795+0.4;

    // Put this after your initialization of all parameters
    ROS_INFO("========== DEBUG: All initialized parameters ==========");

    // Tray Poses
    for (size_t i = 0; i < value_tray_pose.size(); ++i) {
        ROS_INFO("value_tray_pose[%ld]: [%f, %f, %f, %f, %f, %f]",
                i,
                value_tray_pose[i][0], value_tray_pose[i][1], value_tray_pose[i][2],
                value_tray_pose[i][3], value_tray_pose[i][4], value_tray_pose[i][5]);
    }

    // Indices
    ROS_INFO("new_tray_indeces: %s", vector_to_string(new_tray_indeces).c_str());
    ROS_INFO("operator_indeces: %s", vector_to_string(operator_indeces).c_str());
    ROS_INFO("horiz_rotation_indeces: %s", vector_to_string(horiz_rotation_indeces).c_str());
    ROS_INFO("tray_down_indeces: %s", vector_to_string(tray_down_indeces).c_str());

    // Tip position
    ROS_INFO("tip_position: [%f, %f, %f]", tip_position[0], tip_position[1], tip_position[2]);

    // Poses
    print_pose("table_pose", table_pose);
    print_pose("tray_init_pose", tray_init_pose);
    print_pose("bottom_init_pose", bottom_init_pose);
    print_pose("recharge_pose", recharge_pose);
    print_pose("rest_pose", rest_pose);
    print_pose("pick_pose", pick_pose);
    print_pose("bottom_pick_pose", bottom_pick_pose);
    print_pose("pre_pick_pose", pre_pick_pose);
    print_pose("post_pick_pose", post_pick_pose);
    ROS_INFO("tray_pose_operator: [%f, %f, %f, %f, %f, %f]",
        tray_pose_operator[0], tray_pose_operator[1], tray_pose_operator[2],
        tray_pose_operator[3], tray_pose_operator[4], tray_pose_operator[5]);
    ROS_INFO("tray_final_pose: [%f, %f, %f, %f, %f, %f]",
        tray_final_pose[0], tray_final_pose[1], tray_final_pose[2],
        tray_final_pose[3], tray_final_pose[4], tray_final_pose[5]);
    print_pose("end_pose", end_pose);
    print_pose("pre_end_pose", pre_end_pose);

    ROS_INFO("=======================================================");


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

    //  lambda function to pass objects to the service
    ros::ServiceServer service_add_tray = nh.advertiseService<custom_interfaces::AddTray::Request, custom_interfaces::AddTray::Response>(
        "add_tray", [&](auto &req, auto &res) { return handleAddTray(req, res, planning_scene); });
    
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












