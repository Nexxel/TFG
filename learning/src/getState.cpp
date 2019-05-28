#include "learning3.h"

/*------------------------------------
 Callbacks
 -----------------------------------*/
 /*------------------------------------
 Get the rgb image and process it:
 -----------------------------------*/
void callbackImage(const ImageConstPtr& image_msg){
        try
    {
    cv_ptr = cv_bridge::toCvCopy(image_msg, image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }

    // Convert image from color to B&W being white the red object
        
    cv_ptr->image = ~(cv_ptr->image);
        
    Mat3b imageHSV;
    // If it is not a simulation, apply filter for avoiding reflections
    if(!is_simulation){
        GaussianBlur(cv_ptr->image, cv_ptr->image, Size(15,15), 7, 7); // Size(9,9), 4, 4
    }
    // Get the red object
    cvtColor(cv_ptr->image, imageHSV, COLOR_BGR2HSV);
    inRange(imageHSV, Scalar(90 - 10, 100, 100), Scalar(90 + 10, 255, 255), cv_ptr->image);
    // Apply filter for avoiding false positives
    GaussianBlur(cv_ptr->image, cv_ptr->image, Size(7,7), 5, 5);
}

 /*------------------------------------
 Get the rgb camera info and get the projection matrix:
 -----------------------------------*/
 void callbackCameraInfo(const CameraInfoConstPtr& camera_info_msg){
    int counter = 0;
    for (int i  = 0; i < 3; i++){
        for (int j = 0; j < 4; j++){
            P[i][j] = camera_info_msg->P.at(counter);
            counter++;
        }
    }
    
    Mat auxP = Mat(3,4, DataType<double>::type, P);
    Mat auxP_inv = Mat(4,3, DataType<double>::type, P_inv);
    invert(auxP,auxP_inv, DECOMP_SVD);

    // Copy the aux Mat
    for(int i  = 0; i<4; i++){
        for(int j = 0; j<3; j++){
            P_inv[i][j] = auxP_inv.at<double>(i,j);
        }
    }
 }

 /*------------------------------------
 Get gripper effort:
 -----------------------------------*/
 void getGripperEffortCallback(const JointStateConstPtr& joint_states_msg){
    if (joint_states_msg->header.frame_id.empty()){
        gripper_effort = joint_states_msg->effort[5];
    }
 }

/*------------------------------------
 Process messages:
 -----------------------------------*/
 void processMessages(){
     // The counter if for ashuring that at least it is executed 1 time
     int counter = 0;
     gripper_effort = 0;
     while(ros::ok() 
            and (cv_ptr == NULL or P_inv == NULL or gripper_effort == 0 or counter == 0))
    {
        ros::Rate rate(ROS_RATE);
        ros::spinOnce();
        rate.sleep();
        counter++;
    }
 }

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

    processMessages();
    updateState();
    ROS_INFO("\n============STATE===========\n\tDistance: %d\n\tAngle:%d\n\tHeight:%d\n\tPicked:%d\n\tFolded:%d\n\n",
    robot_state.distance_d, robot_state.angle_d, robot_state.height_d, robot_state.object_picked, robot_state.folded);
    ros::shutdown();
    return 0;
}

/*------------------------------------
 Update state:
-----------------------------------*/
void updateState(){
    getLocation();
    calculateRealPos();
    discretizeValues();
    isObjectPicked();
}

/*------------------------------------
 Get the location in pixels of the object:
-----------------------------------*/
void getLocation(){
    if (gui){
        // Show the image
        imshow("Red objects image", cv_ptr->image);
        waitKey(3);
    }
    
    // Get the coordinates of the object pixels
    findNonZero(cv_ptr->image, pixel_locations);

    // Calculate pixel mean in x and y
    sum_x = 0; sum_y = 0; x_values.clear(); y_values.clear();
    for(int i = 0; i<pixel_locations.total(); i++){
        Point pixel = pixel_locations.at<Point>(i);
        sum_x += pixel.x;
        sum_y += pixel.y;
        x_values.insert(pixel.x);
        y_values.insert(pixel.y);
    }
    if (pixel_locations.total() != 0){
        object_center[0] = round(sum_x/pixel_locations.total());
        object_center[1] = round(sum_y/pixel_locations.total());
    }else{
        object_center[0] = -INFINITY;
        object_center[1] = -INFINITY;
    }   
}

/*------------------------------------
 Calculate the real position of the object with respect to the robot:
-----------------------------------*/
void calculateRealPos(){
    int max_u; int max_v;
    int min_u; int min_v;
    int min_x = INFINITY; int min_y = INFINITY;
    int max_x = -INFINITY; int max_y = -INFINITY;

    for(int i = 0; i < pixel_locations.total(); i++){
        Point pixel = pixel_locations.at<Point>(i);
        if(pixel.x == object_center[0] and pixel.y < min_y){
            min_y = pixel.y;
        }if(pixel.x == object_center[0] and pixel.y > max_y){
            max_y = pixel.y;
        }if(pixel.y == object_center[1] and pixel.x < min_x){
            min_x = pixel.x;
        }if(pixel.y == object_center[1] and pixel.x > max_x){
            max_x = pixel.x;
        }
    }

    ROS_INFO("Width object(px): %d", min_x - max_x);

    max_u = max_x;
    min_u = min_x;
    max_v = max_y;
    min_v = min_y;
    
    getObjectPosition(max_u,max_v,min_u, min_v);
}

/*------------------------------------
 Discretize values:
-----------------------------------*/
void discretizeValues(){
    double angle_step = double(cv_ptr->image.cols)/double(discr_level);
    double height_step = double(cv_ptr->image.rows)/double(discr_level);
    double depth_step = double(MAX_DISTANCE)/double(discr_level);
    
    // Discretize values in distance
    discretizeValuesAux(2, depth_step);
    // Discretize values in angle
    discretizeValuesAux(0,angle_step);
    // Discretize values in height
    discretizeValuesAux(1, height_step);
}

/*------------------------------------
 Discretize values auxiliar:
    Inputs:
        - selector:
            0: angle
            1: height
            2: distance
        - step
-----------------------------------*/
void discretizeValuesAux(int selector, double step){
    // Continuos and discretized value
    double *state_c; int *state_d;
    if (selector == 0){
        state_c = &object_center[0];
        state_d = &robot_state.angle_d;
    }else if(selector == 1){
        state_c = &object_center[1];
        state_d = &robot_state.height_d;
    }else{
        state_c = &robot_state.distance_c;
        state_d = &robot_state.distance_d;
    }
    int quadrant = 0;
    bool inside_quadrant = false;
    while (quadrant < discr_level and !inside_quadrant){
        double ranges[2] = {step*double(quadrant),
                         step*double(quadrant+1)};

        if(*state_c >= ranges[0]
            and *state_c < ranges[1])
        {
            inside_quadrant = true;
            *state_d = quadrant + 1;
        }
        quadrant++;
    }
    if (selector != 0 && selector != 1){
        if (*state_c >= step*double(quadrant)){
            *state_d = quadrant + 1;
        }
    }
}

/*------------------------------------
 Get object real position with respect to the robot:
    [X Y Z 1] = P^(-1) * [u v 1]
    Inputs:
        max_u: Max X coordinate of the center of the object (Pixels)
        max_v: Max Y coordinate of the center of the object (Pixels)
        min_u: Min X coordinate of the center of the object (Pixels)
        min_v: Min Y coordinate of the center of the object (Pixels)
-----------------------------------*/
void getObjectPosition(int max_u, int max_v, int min_u, int min_v){
    if ((max_u >= (cv_ptr->image.cols - 42)) || (min_u <= 42)){
        robot_state.angle_c = -INFINITY;
        robot_state.distance_c = -INFINITY;
        robot_state.height_c = -INFINITY;
    }else{
        // Get the distance of the object
        double f = P[0][0];
        double cx = P[0][2];
        double cy = P[1][2]; 
        double real_pos_max[2][1];
        real_pos_max[0][0] = ((max_u - cx));// * WIDTH_PX_2_M;
        real_pos_max[1][0] = ((max_v - cy));// * HEIGHT_PX_2_M;

        double real_pos_min[2][1];
        real_pos_min[0][0] = ((min_u - cx));// * WIDTH_PX_2_M;
        real_pos_min[1][0] = ((min_v - cy));// * HEIGHT_PX_2_M;

        double width = real_pos_max[0][0] - real_pos_min[0][0];
        
        robot_state.distance_c = (f * OBJECT_WIDTH) / width;

        // Get the pixel position in x,y
        double pixel_pos[3][1]; // 3 x 1
        double result[4][1];    // 4 x 1
        pixel_pos[0][0] = object_center[0]; 
        pixel_pos[1][0] = object_center[1];
        pixel_pos[2][0] = 1;
        multiplyP_Inv(result, P_inv, pixel_pos);
        robot_state.angle_c = (result[0][0]/result[3][0]) * WIDTH_PX_2_M * robot_state.distance_c + 0.005; // X = k*Z 
        //It should be -0.12, but as we don't see the entire object we have to modify it
        robot_state.height_c = (result[1][0]/result[3][0]) * HEIGHT_PX_2_M * robot_state.distance_c - 0.05;   // Y = k*Z 
        robot_state.distance_c -= 0.08;
        ROS_INFO("\n\nDistance, Angle, height: \n\t(%.10f, %.10f, %.10f)\n", robot_state.distance_c, robot_state.angle_c, robot_state.height_c);
    }
}

/*------------------------------------
 Multiply P_inv and pixel_pos
-----------------------------------*/
void multiplyP_Inv(double result[4][1], double P_inv[4][3], double pixel_pos[3][1]){
    for (int i = 0; i < 3; i++){
        result[i][0] = 0;
    }
    result[3][0] = 1;
    
    for (int i  = 0; i < 4; i++){
        for (int k  = 0; k < 3; k++){
            result[i][0] += P_inv[i][k] * pixel_pos[k][0];
        }
    }
}

/*------------------------------------
 Check if we are executing a gazebo simulation:
-----------------------------------*/
void isSimulation(){
    int i = 0;
    is_simulation = false;
    while(i < topic_info.size() and !is_simulation){
        if(topic_info[i].name == "/simulation/pose"){
            is_simulation = true;
        }
        i++;
    }
}

/*------------------------------------
 Detect picked object:
-----------------------------------*/
void isObjectPicked(){
    robot_state.object_picked = (!gripper_opened) 
                and (abs(gripper_effort) > MAX_EFFORT);
    ROS_INFO("gripper_effort: %.4f", gripper_effort);
}