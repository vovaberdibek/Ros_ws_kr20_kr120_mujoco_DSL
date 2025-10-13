#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <std_msgs/Int32.h>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <linux/limits.h>

ros::ServiceClient delete_client;
ros::ServiceClient spawn_client;
int32_t shouldSpawn = 100;

void actionCallback(const std_msgs::Int32::ConstPtr& msg){
    shouldSpawn = msg->data; // Aggiorna la variabile in base al messaggio ricevuto
}

void spawnModel(const std::string &modelName, const std::string &modelPath,int32_t index){
    
    gazebo_msgs::SpawnModel spawn_srv;
    spawn_srv.request.model_name = modelName;
    spawn_srv.request.model_xml = modelPath; // Carica il modello da un file XML
    spawn_client.call(spawn_srv);
    ROS_INFO_STREAM(spawn_srv.response.status_message);
    
}

int main(int argc, char **argv){

    ros::init(argc, argv, "model_spawner");
    ros::NodeHandle nh;

    spawn_client = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    delete_client = nh.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");

    // Sottoscrivi al topic
    ros::Subscriber sub = nh.subscribe("action_completed", 1, actionCallback);

    // char cwd[PATH_MAX];
    //     if (getcwd(cwd, sizeof(cwd)) != NULL) {
    //         printf("Current working dir: %s\n", cwd);
    //     } else {
    //         perror("getcwd() error");
    //     }

    while (ros::ok()){
        ros::spinOnce(); // Controlla i messaggi in arrivo
        
        if (shouldSpawn<100){
            std::string tray_file;
            std::string tray_name;
            if(shouldSpawn<=7){
                tray_file="/home/user/cella_ws/src/kr20_kr120/urdf/tray_aoc.sdf";
                tray_name="tray_aoc";
            }else if(shouldSpawn==8){
                tray_file="/home/user/cella_ws/src/kr20_kr120/urdf/tray_dhc.sdf";
                tray_name="tray_dhc";
            }else if(shouldSpawn<=14){
                tray_file="/home/user/cella_ws/src/kr20_kr120/urdf/tray_eps.sdf";
                tray_name="tray_eps";
            }else if(shouldSpawn==15){
                tray_file="/home/user/cella_ws/src/kr20_kr120/urdf/tray_payload.sdf";
                tray_name="tray_payload";
            }else if(shouldSpawn==20){
                tray_file="/home/user/cella_ws/src/kr20_kr120/urdf/MTQ3_MAG.sdf";
                tray_name="MTQ3_MAG";
            }else if(shouldSpawn==21){
                tray_file="/home/user/cella_ws/src/kr20_kr120/urdf/MTQ12.sdf";
                tray_name="MTQ12";
            }else if(shouldSpawn==22){
                tray_file="/home/user/cella_ws/src/kr20_kr120/urdf/CMG2.sdf";
                tray_name="CMG2";
            }else if(shouldSpawn==24){
                tray_file="/home/user/cella_ws/src/kr20_kr120/urdf/CMG1.sdf";
                tray_name="CMG1";
            }else if(shouldSpawn==26){
                tray_file="/home/user/cella_ws/src/kr20_kr120/urdf/CMG34.sdf";
                tray_name="CMG34";
            }else if(shouldSpawn==29){
                tray_file="/home/user/cella_ws/src/kr20_kr120/urdf/BAT1.sdf";
                tray_name="BAT1";
            }else if(shouldSpawn==31){
                tray_file="/home/user/cella_ws/src/kr20_kr120/urdf/BAT2.sdf";
                tray_name="BAT2";
            }else if(shouldSpawn==33){
                tray_file="/home/user/cella_ws/src/kr20_kr120/urdf/PCDU.sdf";
                tray_name="PCDU";
            }else if(shouldSpawn==50){
                tray_file="/home/user/cella_ws/src/kr20_kr120/urdf/bottom_panel.sdf";
                tray_name="bottom_panel";
            }
            
            std::string line;
            std::stringstream xml;
            std::ifstream file(tray_file);
            if (!file.is_open()) {
                std::cerr << "Impossibile aprire il file!" << std::endl;
            }
            while (std::getline(file, line)) {
                xml<<line;
            }
            spawnModel(tray_name, xml.str(),shouldSpawn);
            shouldSpawn = 100; // Resetta la condizione dopo lo spawning
        }
    }

    return 0;

}