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

    initializeVecMat();
    processMessages();
    updateState();
    foldArm();
    processMessages();
    updateState();
    vec4 hom_obj_pos;
    hom_obj_pos << robot_state.angle_c << robot_state.height_c << robot_state.distance_c << 1;
    ROS_INFO("Hom_obj_pos: %.10f %.10f %.10f %.10f", hom_obj_pos(0), hom_obj_pos(1), hom_obj_pos(2), hom_obj_pos(3));
    vec3 intermediate_position = home_pos;
    vec4 next_position;
    next_position = (TSB * (hom_obj_pos));
    intermediate_position(0) = next_position(0) - 0.04;
    intermediate_position(1) = next_position(1);    

    ROS_INFO("Next: %.10f %.10f %.10f %.10f", next_position(0), next_position(1), next_position(2), next_position(3));
    cout << "intermediate position 1: " << intermediate_position;
    mci(intermediate_position, n);
    openGripper();
    ros::Duration(4).sleep();
    intermediate_position.row(2) = next_position.row(2) + 0.06;
    cout << "Intermediate position 2: " << intermediate_position;
    mci(intermediate_position, n);
    ros::Duration(4).sleep();
    next_position(2) += 0.03;
    cout << "Next position: \t" << next_position; 
    mci(next_position.rows(0,2),n);
    /*
    ros::Duration(2).sleep();
    next_position(0) += 0.05;
    mci(next_position.rows(0,2),n);
    */

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