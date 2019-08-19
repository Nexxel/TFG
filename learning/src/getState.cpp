/*
Sergio Gonzalez Muriel
Degree thesis:  Reinforcement learning for object manipulation by a robotic arm
Test code for getting the state of the robot
*/

#include "utils.cpp"

/*------------------------------------
 Methods
 -----------------------------------*/
int main(int argc, char** argv){
    ros::init(argc, argv, "get_state");

    // We check if it is a gazebo simulation
    
    Handlers handlers;
    // Initialize all publishers and subscribers
    ros::master::getTopics(topic_info);
    isSimulation();
    color_image_sub = handlers.getIT().subscribe("/camera/rgb/image_color", 1, &callbackImage);
    camera_info_sub = handlers.getNH().subscribe("/camera/rgb/camera_info", 1, &callbackCameraInfo);
    joint_states_sub = handlers.getNH().subscribe("/joint_states", 1, &getGripperEffortCallback);

    initializeTSB();
    initializeI2P();
    processMessages();
    updateState();
    ROS_INFO("\n============STATE===========\n\tDistance: %d\n\tAngle:%d\n\tHeight:%d\n\tPicked:%d\n\tFolded:%d\n\n",
    robot_state.distance_d, robot_state.angle_d, robot_state.height_d, robot_state.object_picked, robot_state.folded);
    ros::shutdown();
    return 0;
}
