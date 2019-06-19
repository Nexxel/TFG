#include "utils.cpp"

/*------------------------------------
 Methods
 -----------------------------------*/
int main(int argc, char** argv){
    ros::init(argc, argv, "move_arm_to_object");
    vec3 n;
    n << 0 << 0 << 1;

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

    initializeTSB();
    initializeI2P();
    processMessages();
    updateState();
    vec4 hom_obj_pos;
    hom_obj_pos << robot_state.angle_c << robot_state.height_c << robot_state.distance_c << 1;
    ROS_INFO("Hom_obj_pos: %.10f %.10f %.10f %.10f", hom_obj_pos(0), hom_obj_pos(1), hom_obj_pos(2), hom_obj_pos(3));
    vec4 next_position = (TSB * (hom_obj_pos));
    ROS_INFO("Next: %.10f %.10f %.10f %.10f", next_position(0), next_position(1), next_position(2), next_position(3));
    mci(next_position.rows(0,2),n);

    openGripper();
    ros::Duration(3).sleep();
    closeGripper();
    ros::Duration(9).sleep();
    //foldArm();
    processMessages();
    updateState();
    ROS_INFO("Effort: %.10f", gripper_effort);
    ros::shutdown();
    return 0;
}