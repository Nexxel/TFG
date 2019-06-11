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

    initializeTSB();
    initializeI2P();
    processMessages();
    updateState();
    ROS_INFO("\n============STATE===========\n\tDistance: %d\n\tAngle:%d\n\tHeight:%d\n\tPicked:%d\n\tFolded:%d\n\n",
    robot_state.distance_d, robot_state.angle_d, robot_state.height_d, robot_state.object_picked, robot_state.folded);
    ros::shutdown();
    return 0;
}

/*------------------------------------
 Initialize transformation matrix from Sensor frame to Widow-X arm frame:
 -----------------------------------*/
 void initializeTSB(){
     for(int i = 0; i < 4; i++){
         for(int j = 0; j < 4; j++){
             TSB[i][j] = 0;
         }
     }
    TSB[0][0] = -1; TSB[0][3] = 0.03;
    TSB[1][2] = 1; TSB[1][3] = 0.14;
    TSB[2][1] = 1; TSB[2][3] = 0.11;
    TSB[3][3] = 1;
 }

  /*------------------------------------
 Initialize transformation matrix from image frame to P2:
 -----------------------------------*/
 void initializeI2P(){
    for(int i = 0; i < 4; i++){
         for(int j = 0; j < 4; j++){
             I2P[i][j] = 0;
         }
    }
    I2P[0][0] = 1; I2P[1][1] = -1;
    I2P[0][2] = -320.5; I2P[1][2] = 240.5;
    I2P[2][2] = 1; 
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
    int max_u = INFINITY; int max_v = INFINITY;
    int min_u = -INFINITY; int min_v = -INFINITY;
    if(!x_values.empty() && !y_values.empty()){
        /*
        int x_avg = round(sum_x/y_values.size()); int y_avg = round(sum_y/x_values.size());
        x_avg = round(x_avg/x_values.size()); y_avg = round(y_avg/y_values.size());
        int x_diff = x_avg - *(x_values.begin()); int y_diff = y_avg - *(y_values.begin());
        ROS_INFO("y_values.size: %lu", y_values.size());
        ROS_INFO("x_avg: %d", x_avg);
        ROS_INFO("y_avg: %d", y_avg);

        min_u = object_center[0] - x_diff;  max_u = object_center[0] + x_diff;
        min_v = object_center[1] - y_diff;  max_v = object_center[1] + y_diff;
        */
       min_u = *(x_values.begin()); max_u = *(--x_values.end());
       min_v = *(y_values.begin()); max_v = *(--y_values.end());
    }
    ROS_INFO("min_u: %d, max_u: %d", min_u, max_u);
    ROS_INFO("min_v: %d, max_v: %d", min_v, max_v);
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
        object_center[0] = -INFINITY; 
        object_center[1] = -INFINITY; 
        ROS_INFO("angle, distance, height: %.10f, %.10f, %.10f",
         robot_state.angle_c, robot_state.distance_c, robot_state.height_c);
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
        ROS_INFO("Width: %.10f", width);
        ROS_INFO("f: %.10f", f);
        ROS_INFO("(f * OBJECT_WIDTH): %.10f", (f * OBJECT_WIDTH));
        robot_state.distance_c = (f * OBJECT_WIDTH) / width;

        // Get the pixel position in x,y
        double pixel_pos[3][1]; // 3 x 1
        double result[4][1];    // 4 x 1
        pixel_pos[0][0] = object_center[0]; 
        pixel_pos[1][0] = object_center[1];
        pixel_pos[2][0] = 1;
        image2sensor(result);
        robot_state.angle_c = result[0][0] * robot_state.distance_c; 
        //It should be -0.12, but as we don't see the entire object we have to modify it
        robot_state.height_c = result[1][0] * robot_state.distance_c;
        ROS_INFO("\n\nDistance, Angle, height: \n\t(%.10f, %.10f, %.10f)\n", robot_state.distance_c, robot_state.angle_c, robot_state.height_c);
    }
}

/*------------------------------------
 Multiply P_inv and pixel_pos
-----------------------------------*/
void multiplyP_Inv(double result[4][1], double pixel_pos[3][1]){
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
 Multiply 2 transformation matrices
-----------------------------------*/
void multiplyTransformations(double result[4][4], double first[4][4], double second[4][4]){
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            result[i][j] = 0;
        }
    }
    
    for (int i  = 0; i < 4; i++){
		for (int j = 0;j < 4; j++){
            for (int k = 0; k < 3; k++){
                result[i][j] += first[i][k] * second[k][j];
            }
		}
	}
}

/*------------------------------------
 Get the position of the object in the gripper frame
-----------------------------------*/
void sensor2Gripper(double result[4][1], double obj_pos[4][1]){
    double T[4][4];
    multiplyTransformations(T, T05, TSB);
    for (int i = 0; i < 3; i++){
        result[i][0] = 0;
    }
    result[3][0] = 1;
    
    for (int i  = 0; i < 4; i++){
        for (int k  = 0; k < 4; k++){
            result[i][0] += T[i][k] * obj_pos[k][0];
        }
    }
}

/*------------------------------------
 Get the position of the object in the sensor frame
-----------------------------------*/
void image2sensor(double result[4][1]){
    double homogeneus_object_pos[3][1] = {object_center[0], object_center[1], 1};
    double P2[3][1] = {0,0,1};
    for (int i  = 0; i < 4; i++){
        for (int k  = 0; k < 4; k++){
            P2[i][0] += I2P[i][k] * homogeneus_object_pos[k][0];
        }
    }
    multiplyP_Inv(result, P2);
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