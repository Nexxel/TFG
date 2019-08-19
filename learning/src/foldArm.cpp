/*
Sergio Gonzalez Muriel
Degree thesis:  Reinforcement learning for object manipulation by a robotic arm
Test code for folding the WidowX arm
*/

#include "utils.cpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "fold_arm");

    // We check if it is a gazebo simulation
    
    Handlers handlers;

    // Initialize all publishers and subscribers
    ros::master::getTopics(topic_info);
    isSimulation();
    color_image_sub = handlers.getIT().subscribe("/camera/rgb/image_color", 1, &callbackImage);
    camera_info_sub = handlers.getNH().subscribe("/camera/rgb/camera_info", 1, &callbackCameraInfo);
    joint_states_sub = handlers.getNH().subscribe("/joint_states", 1, &getGripperEffortCallback);

    joints[0] = handlers.getNH().advertise<Float64>("/arm_1_joint/command", 1);
    joints[1] = handlers.getNH().advertise<Float64>("/arm_2_joint/command", 1);
    joints[2] = handlers.getNH().advertise<Float64>("/arm_3_joint/command", 1);
    joints[3] = handlers.getNH().advertise<Float64>("/arm_4_joint/command", 1);
    joints[4] = handlers.getNH().advertise<Float64>("/arm_5_joint/command", 1);

    gripper = handlers.getNH().advertise<Float64>("/gripper_1_joint/command", 1);

    initializeVecMat();
    processMessages();
    updateState();

    foldArm();
    processMessages();
    updateState();
    ROS_INFO("Effort: %.10f", gripper_effort);
    ros::shutdown();
    return 0;
}