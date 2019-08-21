/*
Sergio Gonzalez Muriel
Degree thesis:  Reinforcement learning for object manipulation by a robotic arm
Test code for getting the distance error.
*/

#include "utils.cpp"

double sim_x;
double real_distance;
stringstream complete_distances_error_log_name;

/*------------------------------------
 Actualize log for distance error:
-----------------------------------*/
void actualizeDistanceErrorLog(){
    string str(complete_distances_error_log_name.str());
    log_file.open(str.c_str(), ios::app | ios::out);
    real_distance = ((3.55 - sim_x) + 0.087);
    log_file << real_distance << "," << abs(robot_state.distance_c - real_distance) << "\n";
    log_file.close();
}

 /*------------------------------------
 Get simulation pose x:
 -----------------------------------*/
 void getSimulationPoseCallbackError(const Vector3ConstPtr& sim_pose_msg){
     if(update_pose){
        sim_x = sim_pose_msg->x;
     }
 }

int main(int argc, char** argv){
    ros::init(argc, argv, "get_error");

    // We check if it is a gazebo simulation
    
    Handlers handlers;
    // Initialize all publishers and subscribers
    ros::master::getTopics(topic_info);
    isSimulation();
    color_image_sub = handlers.getIT().subscribe("/camera/rgb/image_color", 1, &callbackImage);
    camera_info_sub = handlers.getNH().subscribe("/camera/rgb/camera_info", 1, &callbackCameraInfo);
    joint_states_sub = handlers.getNH().subscribe("/joint_states", 1, &getGripperEffortCallback);
    sim_pose_sub = handlers.getNH().subscribe("/simulation/pose", 1 , &getSimulationPoseCallbackError);

    base = handlers.getNH().advertise<Twist>("/mobile_base/commands/velocity", 1);

    complete_distances_error_log_name << ros::package::getPath("learning") 
                                      << "/distances_error_logs/distance_error_log_4m.txt";

    update_pose = true;
    initializeTSB();
    initializeI2P();
    processMessages();
    updateState();
    actualizeDistanceErrorLog();
    ROS_INFO("\n============STATE===========\n\tDistance: %d\n\tAngle:%d\n\tHeight:%d\n\tPicked:%d\n\tFolded:%d\n\n",
    robot_state.distance_d, robot_state.angle_d, robot_state.height_d, robot_state.object_picked, robot_state.folded);
    ros::shutdown();
    return 0;
}