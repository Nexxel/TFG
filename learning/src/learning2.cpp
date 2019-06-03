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
 Methods
 -----------------------------------*/
int main(int argc, char** argv){
    ros::init(argc, argv, "learning_node");

    // We check if it is a gazebo simulation
    
    Handlers handlers;

    learning(handlers);
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
 Make all the process of learning:
    While node is running:
        1. Get state
        2. Detect if it is reachable
        3. Move arm if correct, else move base
        4. Fold arm
        5. Check reward
 -----------------------------------*/
void learning(Handlers handlers){
    double n[3] = {1,0,0};
    initializeTSB();

    while(ros::ok()){
        // Set random seed by the time of the cpu
        srand( (unsigned)time(NULL) );
        startRandomSimulation();
        sleep(3);
        if (gui){
            namedWindow("Red objects image",CV_WINDOW_AUTOSIZE);
        }
        int counter = 0;

        robot_state.object_picked = false;
        robot_state.folded = false;

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
        base = handlers.getNH().advertise<Twist>("/mobile_base/commands/velocity", 1);
        int sa; int sp;
        while(ros::ok() && (!robot_state.object_picked || !robot_state.folded)){
            // Set random seed by the time of the cpu
            srand( (unsigned)time(NULL) );
            robot_state.angle_d = 0;
            robot_state.height_d = 0;
            robot_state.distance_d = 0;
            robot_state.angle_c = 0;
            robot_state.height_c = 0;
            robot_state.distance_c = 0;
            
            inside_learning = false;

            processMessages();

            // While the learning process, we just want to re-read the joints
            inside_learning = true;

            mcd();
            getGripperPosition();

            // If it's the first time, set the arm to the initial position
            if (counter == 0){
                openGripper();
                foldArm();
                // 1. Get state
                updateState();
                sa = getIndexFromState();
                counter++;
            }
            
            // 2. Select action
            selectAction(sa);

            // 3.1 Move arm if reachable
            if(action == 4){
                ROS_INFO("Moving arm...");
                double step = double(cv_ptr->image.rows)/double(discr_level)/2.0 * HEIGHT_PX_2_M;
                double next_position[3];
                // Hallarle el módulo, normalizarlo y multiplicarlo por la distancia
                double homogeneous_state_sensor[4][1] = {robot_state.angle_c, robot_state.height_c, robot_state.distance_c, 1};
                double homogeneous_obj_pos[4][1];
                sensor2Gripper(homogeneous_obj_pos, homogeneous_state_sensor);
                setNextPosition(next_position,
                            homogeneous_obj_pos[2][0]/homogeneous_obj_pos[3][0],
                            homogeneous_obj_pos[0][0]/homogeneous_obj_pos[3][0], 
                            robot_state.height_d * step);
                mci(next_position,n);
                openGripper();
                ros::Duration(3).sleep();
                closeGripper();
                ros::Duration(9).sleep();
            }
            // 3.2 Move base if not reachable
            else{
                if(robot_state.folded == 0){
                    foldArm();
                    ros::Duration(3).sleep();
                }
                Twist base_movement; 
                // Move front
                if (action == 2){
                    ROS_INFO("Moving front...");
                    base_movement.linear.x = 0.1;
                }
                // Move back
                else if(action == 3){
                    ROS_INFO("Moving back...");
                    base_movement.linear.x = -0.1;
                }
                // Turn left
                else if(action == 0){
                    ROS_INFO("Turning left...");
                    base_movement.angular.z = 0.1;
                }
                // Turn right
                else if(action == 1){
                    ROS_INFO("Turning right...");
                    base_movement.angular.z = -0.1;
                }
                base.publish(base_movement);
            }
            ros::Duration(6).sleep();

            // Update state
            processMessages();
            updateState();
            sp = getIndexFromState();


            // 5. Check reward
            double reward = calculateReward(sa, sp);
            
            // Update Q-matrix
            q_matrix[sa][action] = (1 - ALPHA) * q_matrix[sa][action] + ALPHA * (reward + GAMMA * V[sp]);

            // Update visit matrix
            visit_matrix[sa][action]++;

            // Update V and policy matrices
            updateVPolicy(sa);
            steps++;
            actualizeLog(sa, sp, reward);
            actualizeSimplifiedLog(sa, sp, reward);
            sa = sp;
        }
        if (gui){
            destroyWindow("Red objects image");
        }
        killSimulation();
        steps = 0;
    }
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
    if(x_values.size() != 0 && y_values.size() != 0){
        int x_avg = round(sum_x/x_values.size()); int y_avg = round(sum_y/y_values.size());
        min_u = object_center[0] - x_avg/2;  max_u = object_center[0] + x_avg/2;
        min_v = object_center[1] - y_avg/2;  max_v = object_center[1] + y_avg/2;
    }
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
        state_c = &robot_state.angle_c;
        state_d = &robot_state.angle_d;
    }else if(selector == 1){
        state_c = &robot_state.height_c;
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
        if(selector == 0){
            ranges[0] *= WIDTH_PX_2_M;
            ranges[1] *= WIDTH_PX_2_M;
        }else if(selector == 1){
            ranges[0] *= HEIGHT_PX_2_M;
            ranges[1] *= HEIGHT_PX_2_M;
        }
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
 Update the matrix T05 of the direct kinematic model copying aux into T05
-----------------------------------*/
void updateT05(double T05[4][4], double aux[4][4]){
	for(int i  = 0; i<4; i++){
		for(int j  = 0; j<4; j++){
			T05[i][j] = aux[i][j];
		}
	}
}

/*------------------------------------
 Get the direct kinematic model of the widowX-arm
-----------------------------------*/
void mcd(){
    double alpha[] = {0, -M_PI/2, M_PI, 0, M_PI/2};
	double a[] = {0,0,d2,L3,0};
	double d[] = {0,0,0,0,0};
	double q[] = {joint_angles[0],
                    joint_angles[1]-beta,
                    joint_angles[2]-beta,
                    joint_angles[3]+(M_PI/2),
                    joint_angles[4]};

	int i  = 0;
	double T01[][4] = {
		{cos(q[i]), -sin(q[i]), 0, a[i]},
		{sin(q[i])*cos(alpha[i]), cos(q[i])*cos(alpha[i]), -0, -0*d[i]},
		{sin(q[i])*0, cos(q[i])*0, cos(alpha[i]), cos(alpha[i])*d[i]},
		{0, 0, 0, 1}
	};

	i  = 1;
	double T12[][4] = {
		{cos(q[i]), -sin(q[i]), 0, a[i]},
		{sin(q[i])*0, cos(q[i])*0, -sin(alpha[i]), -sin(alpha[i])*d[i]},
		{sin(q[i])*sin(alpha[i]), cos(q[i])*sin(alpha[i]), 0, 0*d[i]},
		{0, 0, 0, 1}
	};

	i  = 2;
	double T23[][4] = {
		{cos(q[i]), -sin(q[i]), 0, a[i]},
		{sin(q[i])*cos(alpha[i]), cos(q[i])*cos(alpha[i]), -0, -0*d[i]},
		{sin(q[i])*0, cos(q[i])*0, cos(alpha[i]), cos(alpha[i])*d[i]},
		{0, 0, 0, 1}
	};

	i  = 3;
	double T34[][4] = {
		{cos(q[i]), -sin(q[i]), 0, a[i]},
		{sin(q[i])*cos(alpha[i]), cos(q[i])*cos(alpha[i]), -0, -0*d[i]},
		{sin(q[i])*0, cos(q[i])*0, cos(alpha[i]), cos(alpha[i])*d[i]},
		{0, 0, 0, 1}
	};

	i  = 4;
	double T45[][4] = {
		{cos(q[i]), -sin(q[i]), 0, a[i]},
		{sin(q[i])*0, cos(q[i])*0, -sin(alpha[i]), -sin(alpha[i])*d[i]},
		{sin(q[i])*sin(alpha[i]), cos(q[i])*sin(alpha[i]), 0, 0*d[i]},
		{0, 0, 0, 1}
	};

	double aux[4][4];

	multiplyTransformations(aux, T01, T12);
	
	updateT05(T05, aux);

	multiplyTransformations(aux, T05, T23);

	updateT05(T05, aux);

	multiplyTransformations(aux, T05, T34);

	updateT05(T05, aux);

	multiplyTransformations(aux, T05, T45);

	updateT05(T05, aux);
}

/*------------------------------------
 Get the position of the gripper by means of the Direct kinematic model:
 -----------------------------------*/
void getGripperPosition(){
    double ax = T05[0][2];
	double ay = T05[1][2];
	double az = T05[2][2];
	double px = T05[0][3];
	double py = T05[1][3];
	double pz = T05[2][3];

	gripper_position[0] = px + L45*ax;
	gripper_position[1] = py + L45*ay;
	gripper_position[2] = pz + L45*az;

    a[0] = ax;
    a[1] = ay;
    a[2] = az;
}

/*------------------------------------
 Get the angle of each joint in order to reach the desired position
 by means of the inverse kinematic model:
    Inputs:
        - next_position: Desired position
        - a: Desired angle orientation of the wrisp
        - n: Desired orientation of the wrisp 
 -----------------------------------*/
void mci(double next_position[3], double n[3]){
	double px = next_position[0] - L45*a[0];
	double py = next_position[1] - L45*a[1];
	double pz = next_position[2] - L45*a[2];

	double q1 = atan2(py, px);
            
	double k = pow(pz, 2) + pow(d2, 2) + pow(((px * cos(q1)) + (py * sin(q1))), 2) - pow(L3, 2);
	double k1 = 2 * d2 * px * cos(q1) + 2 * py * d2 * sin(q1);
	double k2 = 2 * pz * d2;
	double theta2b = atan2(k1, k2) - atan2(k, -sqrt(pow(k1,2)+pow(k2,2)-pow(k,2)));
	double q2 = theta2b + beta;

	double theta23 = asin((-pz - d2*sin(theta2b))/L3);
	double q3 = q2 - theta23;

	double L = a[2]*cos(q2-q3) + a[0]*sin(q2-q3)*cos(q1) + a[1]*sin(q2-q3)*sin(q1);
	double q4 = acos(-L) - (M_PI/2);

	double q5 = asin(n[0]*sin(q1) - n[1]*cos(q1));
    ROS_INFO("\n\nq1: %.2f\nq2: %.2f\nq3: %.2f\nq4: %.2f\nq5: %.2f\n", q1,q2,q3,q4,q5);

	Float64 angle;
    angle.data = q5;
    joints[4].publish(angle);
    angle.data = q4;
    joints[3].publish(angle);
    angle.data = q3;
    joints[2].publish(angle);
    angle.data = q2;
    joints[1].publish(angle);
    angle.data = q1;
    joints[0].publish(angle);

    robot_state.folded = (next_position[0] == 0
                        and next_position[1] == 0
                        and next_position[2] == 0);
    if(!isnan(q1))
    joint_angles[0] = q1;

    if(!isnan(q2))
    joint_angles[1] = q2;

    if(!isnan(q3))
    joint_angles[2] = q3;

    if(!isnan(q4))
    joint_angles[3] = q4;

    if(!isnan(q5))
    joint_angles[4] = q5;

    processMessages();
    mcd();
    getGripperPosition();
}

/*------------------------------------
 Open gripper:
 -----------------------------------*/
void openGripper(){
    Float64 gripper_value; gripper_value.data = 2.5;
    gripper.publish(gripper_value);
    gripper_opened = true;
    processMessages();
}

/*------------------------------------
 Close gripper:
 -----------------------------------*/
 void closeGripper(){
    Float64 gripper_value; gripper_value.data = 0;
    gripper.publish(gripper_value);
    gripper_opened = false;
    processMessages();
 }

/*------------------------------------
 Detect picked object:
-----------------------------------*/
void isObjectPicked(){
    robot_state.object_picked = (!gripper_opened) 
                and (abs(gripper_effort) > MAX_EFFORT);
    ROS_INFO("gripper_effort: %.4f", gripper_effort);
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
        multiplyP_Inv(result, pixel_pos);
        robot_state.angle_c = (result[0][0]/result[3][0]); 
        robot_state.height_c = (result[1][0]/result[3][0]); 
        ROS_INFO("\n\nDistance, Angle, height: \n\t(%.10f, %.10f, %.10f)\n", robot_state.distance_c, robot_state.angle_c, robot_state.height_c);
    }
}

/*------------------------------------
 Detect if object is reachable:
    Object is centered in x, near and up
-----------------------------------*/
void isObjectReachable(){
    object_reachable = robot_state.angle_d == (discr_level/2)+1
                        and robot_state.distance_d == 1
                        and robot_state.height_d <= discr_level/3;
}

/*------------------------------------
 Set next position:
   Inputs:
      next_position: The next position to initialize
      x, y, z: The coordinates of the next position
-----------------------------------*/
void setNextPosition(double next_position[3], double x, double y, double z){
    next_position[0] = x;
    next_position[1] = y;
    next_position[2] = z;
}
/*------------------------------------
 Fold arm:
-----------------------------------*/
void foldArm(){
    double n[3] = {0,0,1}; 
    double next_position[3];
    // Turn the arm to the position (0.3125,0,0.1450)
    setNextPosition(next_position,
                     0.3125,
                     0, 
                     0.1450);
    mci(next_position,n);

    robot_state.folded = true;
}

/*------------------------------------
 Start a new random simulation:
-----------------------------------*/
void startRandomSimulation(){
    int status;
        // Open an empty world in gazebo
        if (gui){
            status = system("xterm -hold -e \"roslaunch gazebo_ros empty_world.launch paused:=true\" &");
        }else{
            status = system("xterm -hold -e \"roslaunch gazebo_ros empty_world.launch paused:=true gui:=false\" &");
        }
        if (status == 0){
            sleep(6);
            double x = MIN_X + ((double)rand()/double(RAND_MAX))* (MAX_X-MIN_X);
            double y = MIN_Y + ((double)rand()/double(RAND_MAX))* (MAX_Y-MIN_Y);
            int box = MIN_BOX + ((double)rand()/double(RAND_MAX))* (MAX_BOX-MIN_BOX);
            double z = 0.05*(double)box;
            stringstream xterm_box; stringstream xterm_object;
            xterm_box << "xterm +hold -e \"rosrun gazebo_ros spawn_model -file $(rospack find learning)/urdf/box_" << box << ".urdf -urdf -x " << x
                    << " -z " << z << " -y " << y << " -model box\" &";
            xterm_object << "xterm +hold -e \"rosrun gazebo_ros spawn_model -file $(rospack find learning)/urdf/cylinder.urdf -urdf -x " 
                        << (x - 0.45) << " -z " << (z*2+0.05) << " -y " << y << " -model red_object\" &";
            string str(xterm_box.str());
            const char* xterm_box_str = str.c_str();
            system(xterm_box_str);
            sleep(3);
            str = xterm_object.str();
            const char* xterm_object_str = str.c_str();
            system(xterm_object_str);
            sleep(3);
            stringstream xterm_wall;
            xterm_wall << "xterm +hold -e \"rosrun gazebo_ros spawn_model -file $(rospack find learning)/urdf/wall.urdf -urdf -x " 
                        << (x - 0.5) << " -y " << (y+3) << " -model wall\" &";
            str = xterm_wall.str();
            const char* xterm_wall_str = str.c_str(); 
            system(xterm_wall_str);
            sleep(3);
            stringstream xterm_wall1;
            xterm_wall1 << "xterm +hold -e \"rosrun gazebo_ros spawn_model -file $(rospack find learning)/urdf/wall.urdf -urdf -x " 
                        << (x - 0.5) << " -y " << (y-3) << " -model wall1\" &";
            str = xterm_wall1.str();
            xterm_wall_str = str.c_str(); 
            system(xterm_wall_str);
            sleep(3);
            stringstream xterm_wall2;
            xterm_wall2 << "xterm +hold -e \"rosrun gazebo_ros spawn_model -file $(rospack find learning)/urdf/wall2.urdf -urdf -x " 
                        << (- 0.5) << " -y " << y << " -model wall2\" &";
            str = xterm_wall2.str();
            xterm_wall_str = str.c_str(); 
            system(xterm_wall_str);
            sleep(3);
            stringstream xterm_wall3;
            xterm_wall3 << "xterm +hold -e \"rosrun gazebo_ros spawn_model -file $(rospack find learning)/urdf/wall2.urdf -urdf -x " 
                        << (x/2- 0.5) << " -y " << y+5.55 << " -Y " << M_PI/2 << " -model wall3\" &";
            str = xterm_wall3.str();
            xterm_wall_str = str.c_str(); 
            system(xterm_wall_str);
            sleep(3);
            stringstream xterm_wall4;
            xterm_wall4 << "xterm +hold -e \"rosrun gazebo_ros spawn_model -file $(rospack find learning)/urdf/wall2.urdf -urdf -x " 
                        << (x/2- 0.5) << " -y " << y-5.55 << " -Y " << M_PI/2 << " -model wall4\" &";
            str = xterm_wall4.str();
            xterm_wall_str = str.c_str(); 
            system(xterm_wall_str);
            sleep(3);
            // Instantiate a turtlebot in that empty world
            system("xterm -hold -e \"roslaunch crumb_gazebo test.launch\" &");
            sleep(10);
            // Unpause simulation
            system("rosservice call /gazebo/unpause_physics");
            sleep(5);
            simulations++;   
        }else{
            system("killall -9 xterm gzserver");
            ros::shutdown();
        }
}
/*------------------------------------
 Kill the current simulation:
-----------------------------------*/
void killSimulation(){
    ROS_INFO("Killing actual simulation...");
    sleep(2);
    // Pause simulation
    system("rosservice call /gazebo/pause_physics");
    sleep(1);
    // Kill process
    system("killall -9 xterm gzserver");
    sleep(3); 
}
/*------------------------------------
 Get the matrix column index from the robot state:
-----------------------------------*/
int getIndexFromState(){
    int num_elems = discr_level + 1;
    return  (robot_state.distance_d) * pow(num_elems,2) * pow(2,2) +
            (robot_state.angle_d) * num_elems * pow(2,2) +
            (robot_state.height_d) * pow(2,2) +
            robot_state.object_picked * 2 +
            robot_state.folded;
}
/*------------------------------------
 Modify the state of the robot from the column index:
-----------------------------------*/
void getStateFromIndex(int index){
    int num_elems = discr_level + 1;
    robot_state.distance_d = ((int)index / (int)(pow(num_elems,2) * pow(2,2)));
    robot_state.angle_d = ((index % (int)(pow(num_elems,2) * pow(2,2))) / (int)(num_elems * pow(2,2)));
    robot_state.height_d = ((index % (int)(pow(num_elems,2) * pow(2,2)) % (int)(num_elems * pow(2,2))) / (int)pow(2,2));
    robot_state.object_picked = (index % (int)(pow(num_elems,2) * pow(2,2)) % (int)(num_elems * pow(2,2)) % (int)pow(2,2)) / 2;
    robot_state.folded = (index % (int)(pow(num_elems,2) * pow(2,2)) % (int)(num_elems * pow(2,2)) % (int)pow(2,2) % 2);
}
/*------------------------------------
 Select action:
-----------------------------------*/
void selectAction(int sa){
    float not_visited = 0;
    double prev_expl_rate = exploration_rate;
    for (int i = 0; i<N_ACTIONS; i++){
        not_visited += (visit_matrix[sa][i] == 0);
    }
    if (((float)not_visited/(float)N_ACTIONS) >= 0.25) { 
        exploration_rate = 100;
    }
    if (floor(unifRnd(0, 100)) <= exploration_rate){
        action = floor(unifRnd(0,N_ACTIONS-1));
    }else{
        action = policy_matrix[sa];
    }
    counter++;
    exploration_rate = prev_expl_rate;
    exploration_rate = MIN_EXPLORATION + (MAX_EXPLORATION - MIN_EXPLORATION) * exp(-DECAY * counter);
    ROS_INFO("\n\n\nStep %d exploration_rate: %.2f\n\n\n", counter, exploration_rate);
}
/*------------------------------------
 Update V and policy matrix:
-----------------------------------*/
void updateVPolicy(int s){
    V[s] = q_matrix[s][0];
    policy_matrix[s] = 0;
    for(int i = 1; i < N_ACTIONS; i++){
        if(q_matrix[s][i] > V[s]){
            V[s] = q_matrix[s][i];
            policy_matrix[s] = i;
        }
    }
}
/*------------------------------------
 Calculate reward:
-----------------------------------*/
double calculateReward(int sa, int sp){
    int prev_dist; int prev_ang; int prev_height;
    int act_dist; int act_ang; int act_height;
    getStateFromIndex(sa);
    prev_dist = robot_state.distance_d;
    prev_ang = robot_state.angle_d;
    prev_height = robot_state.height_d;
    getStateFromIndex(sp);
    act_dist = robot_state.distance_d;
    act_ang = robot_state.angle_d;
    act_height = robot_state.height_d;
    int reward = 0;
    if((prev_dist == 0 || prev_ang == 0 || prev_height == 0) &&
        (act_dist > 0 && act_ang > 0 && act_height >0)){
            reward += 2;
    }
    else if(act_dist > 0 && act_dist < prev_dist){
        reward += 3;
    }
    reward += 100 * robot_state.object_picked * robot_state.folded;
    return reward;
}

/*------------------------------------
 Actualize log:
-----------------------------------*/
void actualizeLog(int sa, int sp, double reward){
    if (steps == 1 && simulations == 1){
        log_file.open("$(rospack find learning)/logs/log_test_turning.txt");
    }else{
        log_file.open("$(rospack find learning)/logs/log_test_turning.txt", ios::app | ios::out);
    }
    log_file << "=======================================\n";
    log_file << "Simulation: " << simulations << "\n";
    log_file << "Iteration: " << steps << "\n";
    log_file << "----------\n";
    log_file << "State: " << sa << "\n";
    getStateFromIndex(sa);
    log_file << "\tDistance: " << robot_state.distance_d << "\n";
    log_file << "\tAngle: " << robot_state.angle_d << "\n";
    log_file << "\tHeight: " << robot_state.height_d << "\n";
    log_file << "\tObject picked: " << robot_state.object_picked << "\n";
    log_file << "\tArm folded: " << robot_state.folded << "\n";
    getStateFromIndex(sp);
    log_file << "State': " << sp << "\n";
    log_file << "\tDistance: " << robot_state.distance_d << "\n";
    log_file << "\tAngle: " << robot_state.angle_d << "\n";
    log_file << "\tHeight: " << robot_state.height_d << "\n";
    log_file << "\tObject picked: " << robot_state.object_picked << "\n";
    log_file << "\tArm folded: " << robot_state.folded << "\n";
    log_file << "Action: " << action << "\n";
    log_file << "\t" << ((action == 2) ? "Move front" :
                                ((action == 3) ? "Move back" :
                                (action == 0) ? "Turn left" :
                                (action == 1) ? "Turn right" : "Move arm")) << "\n";
    log_file << "Reward: " << reward << "\n";
    log_file << "New value of Visit matrix: " << visit_matrix[sa][action] << "\n";
    log_file << "New value of Q matrix: " << q_matrix[sa][action] << "\n";
    log_file << "New value of Value function: " << V[sa] << "\n";
    log_file << "New value of Policy matrix: " << policy_matrix[sa] << "\n\n";
    log_file.close();
}

/*------------------------------------
 Actualize simplified log:
-----------------------------------*/
void actualizeSimplifiedLog(int sa, int sp, double reward){
    if (steps == 1 && simulations == 1){
        log_file.open("$(rospack find learning)/simplified_logs/simplified_log_test_turning.txt");
    }else{
        log_file.open("$(rospack find learning)/simplified_logs/simplified_log_test_turning.txt", ios::app | ios::out);
    }
    log_file << simulations << ",";
    log_file << steps << ",";
    log_file << sa << ",";
    getStateFromIndex(sa);
    log_file << robot_state.distance_d << ",";
    log_file << robot_state.angle_d << ",";
    log_file << robot_state.height_d << ",";
    log_file << robot_state.object_picked << ",";
    log_file << robot_state.folded << ",";
    getStateFromIndex(sp);
    log_file << sp << ",";
    log_file << robot_state.distance_d << ",";
    log_file << robot_state.angle_d << ",";
    log_file << robot_state.height_d << ",";
    log_file << robot_state.object_picked << ",";
    log_file << robot_state.folded << ",";
    log_file << action << ",";
    log_file << ((action == 2) ? "\"Move front\"" :
                                ((action == 3) ? "\"Move back\"" :
                                (action == 0) ? "\"Turn left\"" :
                                (action == 1) ? "\"Turn right\"" : "\"Move arm\"")) << ",";
    log_file << reward << ",";
    log_file << visit_matrix[sa][action] << ",";
    log_file << q_matrix[sa][action] << ",";
    log_file << V[sa] << ",";
    log_file << policy_matrix[sa] << "\n";
    log_file.close();
}

/*------------------------------------
 Print debug:
-----------------------------------*/
void printDebug(string function, int line){
    ROS_INFO("\033[1;31m\nMethod %s:\n\tLine %u\n", function.c_str(), line);
}
/*------------------------------------
 Get uniform random:
    Inputs:
        - min
        - max
-----------------------------------*/
double unifRnd(double min, double max){
    return min + ((double)rand()/(double)RAND_MAX) * ((max+1) - min);
}