#include "utils.cpp"

cv_bridge::CvImagePtr cv_ptr_depth;

void getDepthCallback(const ImageConstPtr& image_msg){
        try
    {
    cv_ptr_depth = cv_bridge::toCvCopy(image_msg, image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }
}

int main(int argc, char** argv){
ros::init(argc, argv, "get_distance");
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
    image_transport::Subscriber depth_image_sub = handlers.getIT().subscribe("/camera/depth/image_raw", 1, &getDepthCallback);

    joints[0] = handlers.getNH().advertise<Float64>("/arm_1_joint/command", 1);
    joints[1] = handlers.getNH().advertise<Float64>("/arm_2_joint/command", 1);
    joints[2] = handlers.getNH().advertise<Float64>("/arm_3_joint/command", 1);
    joints[3] = handlers.getNH().advertise<Float64>("/arm_4_joint/command", 1);
    joints[4] = handlers.getNH().advertise<Float64>("/arm_5_joint/command", 1);

    gripper = handlers.getNH().advertise<Float64>("/gripper_1_joint/command", 1);

    initializeVecMat();
    processMessages();
    updateState();
    ROS_INFO("\n\n\n\n\nDistancia cogida por la cámara de profundidad: %.10f", cv_ptr_depth->image.at<double>(object_center(0) + object_center(1) * CAMERA_WIDTH)/255 * 8000);

    ros::shutdown();
    return 0;
}