#include "learning.h"

/*------------------------------------
 Callbacks
 -----------------------------------*/
 /*------------------------------------
 Get the rgb image and process it:
 -----------------------------------*/
 void callbackImage(const ImageConstPtr& image_msg){
    if (!inside_learning){
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
 }

 /*------------------------------------
 Get the rgb camera info and get the projection matrix:
 -----------------------------------*/
 void callbackCameraInfo(const CameraInfoConstPtr& camera_info_msg){
    if(!inside_learning){
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
 }

/*------------------------------------
 Change discretization level:
 -----------------------------------*/
 void setDiscretizationLevel(const Int16ConstPtr& new_discr_level){
    if (!inside_learning){
        discr_level = new_discr_level->data;
    }
 }
/*------------------------------------
 Methods
 -----------------------------------*/
int main(int argc, char** argv){
    ros::init(argc, argv, "learning_node");

    // We check if it is a gazebo simulation
    ros::master::getTopics(topic_info);
    isSimulation();

    Handlers handlers;

    color_image_sub = handlers.getIT().subscribe("/camera/rgb/image_color", 1, &callbackImage);
    camera_info_sub = handlers.getNH().subscribe("/camera/rgb/camera_info", 1, &callbackCameraInfo);
    discr_level_sub = handlers.getNH().subscribe("/learning/set_discr_level", 1, setDiscretizationLevel);

    joints[0] = handlers.getNH().advertise<Float64>("/arm_1_joint/command", 1);
    joints[1] = handlers.getNH().advertise<Float64>("/arm_2_joint/command", 1);
    joints[2] = handlers.getNH().advertise<Float64>("/arm_3_joint/command", 1);
    joints[3] = handlers.getNH().advertise<Float64>("/arm_4_joint/command", 1);
    joints[4] = handlers.getNH().advertise<Float64>("/arm_5_joint/command", 1);

    gripper = handlers.getNH().advertise<Float64>("/gripper_1_joint/command", 1);
    base = handlers.getNH().advertise<Twist>("/mobile_base/commands/velocity", 1);

    robot_state.object_picked = false;
    robot_state.folded = false;
    robot_state.angle_d = 1;
    robot_state.height_d = 1;
    robot_state.distance_d = 1;

    learning(handlers);
    return 0;
}

/*------------------------------------
 Process messages:
 -----------------------------------*/
 void processMessages(){
     // The counter if for ashuring that at least it is executed 1 time
     int counter = 0;
     while(ros::ok() 
            and (cv_ptr == NULL or P_inv == NULL or counter == 0))
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
    namedWindow("Red objects image",CV_WINDOW_AUTOSIZE);
    double n[3] = {1,0,0};
    int counter = 0;
    while(ros::ok()){
        inside_learning = false;

        processMessages();

        // While the learning process, we just want to re-read the joints
       inside_learning = true;

        // If it's the first time, set the arm to the initial position
        if (counter == 0){
            openGripper();
            foldArm();
            counter++;
        }
        
        mcd();
        getGripperPosition();

        // 1. Get state
        updateState();
        //printDebug("learning 1", 149);
        // 2. Detect if object is reachable
        isObjectReachable();
        //printDebug("learning 2", 152);
        // 3.1 Move arm if reachable
        if(object_reachable and !robot_state.object_picked){
            double next_position[3];
            setNextPosition(next_position,
                        gripper_position[0],
                        gripper_position[1], 
                        robot_state.height_c);
            mci(next_position,n);
            setNextPosition(next_position,
                        gripper_position[0],
                        robot_state.angle_c, 
                        robot_state.height_c);
            mci(next_position,n);
            setNextPosition(next_position,
                        robot_state.distance_c,
                        robot_state.angle_c, 
                        robot_state.height_c);
            mci(next_position,n);
            closeGripper();
        }
        // 3.2 Move base if not reachable
        else{
            Twist base_movement; 
            int middle_quadrant = ceil(discr_level/2.0);
           if (robot_state.angle_d == middle_quadrant){
               base_movement.linear.x = 0.2;
           }else if(robot_state.angle_d < middle_quadrant){
                base_movement.angular.z = 0.2;
            }else{
                base_movement.angular.z = -0.2;
            }
            base.publish(base_movement);
            processMessages();
        }
        //printDebug("learning 3", 184);

        // 4. Fold arm
        updateState();
        if(robot_state.object_picked){
            foldArm();          
        }

        //printDebug("learning 4", 188);
        // 5. Check reward
        if(giveReward()){
            // Give reward
        }
        //printDebug("learning 5", 193);
    }
    destroyWindow("Red objects image");
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
    // Show the image
    imshow("Red objects image", cv_ptr->image);
    waitKey(3);
    // Get the coordinates of the object pixels
    findNonZero(cv_ptr->image, pixel_locations);

    // Calculate pixel mean in x and y
    int sum_x = 0; int sum_y = 0;
    for(int i = 0; i<pixel_locations.total(); i++){
        Point pixel = pixel_locations.at<Point>(i);
        sum_x += pixel.x;
        sum_y += pixel.y;
    }
    if (pixel_locations.total() != 0){
        object_center[0] = round(sum_x/pixel_locations.total());
        object_center[1] = round(sum_y/pixel_locations.total());
    }else{
        object_center[0] = 0;
        object_center[1] = 0;
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
    double angle_step = cv_ptr->image.cols/discr_level;
    double height_step = cv_ptr->image.rows/discr_level;
    double depth_step = 5.5/discr_level;    // Depth of field = [0.5,6]
    
    // Discretize values in angle
    discretizeValuesAux(0,angle_step);
    // Discretize values in height
    discretizeValuesAux(1, height_step);
    // Discretize values in distance
    discretizeValuesAux(2, depth_step);
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
        int ranges[2] = {round(step*quadrant),
                         round(step*(quadrant+1))};
        // Field of view is from 0.5m
        if(selector > 1){
            ranges[0] += 0.5; ranges[1] += 0.5;
        }
        if(*state_c >= ranges[0]
            and *state_c < ranges[1])
        {
            inside_quadrant = true;
            *state_d = quadrant + 1;
        }
        quadrant++;
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
void multiplyP_Inv(double result[4][1], double P_inv[4][3], double pixel_pos[3][1]){
    for (int i = 0; i < 4; i++){
        result[i][0] = 0;
    }
    
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
            
	double k = pow(pz, 2) + pow(d2, 2) + pow((px * cos(q1) + py * sin(q1)), 2) - pow(L3, 2);
	double k1 = 2 * d2 * px * cos(q1) + 2 * py * d2 * sin(q1);
	double k2 = 2 * pz * d2;
	double theta2b = atan2(k1, k2) - atan2(k, -sqrt(pow(k1,2)+pow(k2,2)-pow(k,2)));
	double q2 = theta2b + beta;

	double theta23 = asin((-pz - d2*sin(theta2b))/L3);
    ROS_INFO("\n\n(-pz - d2*sin(beta))/L3: %.10f\n", (-pz - d2*sin(beta))/L3);
	double q3 = q2 - theta23;

	double L = a[2]*cos(q2-q3) + a[0]*sin(q2-q3)*cos(q1) + a[1]*sin(q2-q3)*sin(q1);
	double q4 = acos(-L) - (M_PI/2);

	double q5 = asin(n[0]*sin(q1) - n[1]*cos(q1));

    ROS_INFO("\n\nq1: %.2f\nq2: %.2f\nq3: %.2f\nq4: %.2f\nq5: %.2f\n", q1,q2,q3,q4,q5);

	Float64 angle;
    angle.data = q1;
    joints[0].publish(angle);
    angle.data = q2;
    joints[1].publish(angle);
    angle.data = q3;
    joints[2].publish(angle);
    angle.data = q4;
    joints[3].publish(angle);
    angle.data = q5;
    joints[4].publish(angle);

    robot_state.folded = (next_position[0] == 0
                        and next_position[1] == 0
                        and next_position[2] == 0);

    joint_angles[0] = q1;
    joint_angles[1] = q2;
    joint_angles[2] = q3;
    joint_angles[3] = q4;
    joint_angles[4] = q5;

    processMessages();
    mcd();
    getGripperPosition();
}

/*------------------------------------
 Open gripper:
 -----------------------------------*/
void openGripper(){
    Float64 gripper_value; gripper_value.data = 0.0;
    gripper.publish(gripper_value);
    gripper_opened = true;
    processMessages();
}

/*------------------------------------
 Close gripper:
 -----------------------------------*/
 void closeGripper(){
    Float64 gripper_value; gripper_value.data = 2.0;
    gripper.publish(gripper_value);
    gripper_opened = false;
    processMessages();
 }

/*------------------------------------
 Detect picked object:
-----------------------------------*/
void isObjectPicked(){
    robot_state.object_picked = (!gripper_opened) 
                and (gripper_effort < MIN_EFFORT);
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
    // Get the distance of the object
    double f = P[0][0];
    double cx = P[0][2];
    double cy = P[1][2]; 
    double real_pos_max[2][1];
    real_pos_max[0][0] = ((max_u - cx) / f) * HEIGHT_PX_2_M;
    real_pos_max[1][0] = ((max_v - cy) / f) * WIDTH_PX_2_M;

    double real_pos_min[2][1];
    real_pos_min[0][0] = ((min_u - cx) / f) * HEIGHT_PX_2_M;
    real_pos_min[1][0] = ((min_v - cy) / f) * WIDTH_PX_2_M;

    double width = real_pos_max[0][0] - real_pos_min[0][0];
    
    ROS_INFO("\n\nwidth: %.10f\n", width);

    robot_state.distance_c = (f * WIDTH_PX_2_M * OBJECT_WIDTH) / width;

    // Get the pixel position in x,y
    double pixel_pos[3][1]; // 3 x 1
    double result[4][1];    // 4 x 1
    pixel_pos[0][0] = object_center[0]; 
    pixel_pos[1][0] = object_center[1];
    pixel_pos[2][0] = 1;
    multiplyP_Inv(result, P_inv, pixel_pos);
    robot_state.angle_c = (result[0][0]/result[3][0]) * WIDTH_PX_2_M * robot_state.distance_c; // X = k*Z
    robot_state.height_c = (result[1][0]/result[3][0]) * HEIGHT_PX_2_M * robot_state.distance_c;   // Y = k*Z
    ROS_INFO("\n\nDistancia, ángulo, altura: \n\t(%.2f, %.2f, %.2f)\n", robot_state.distance_c, robot_state.angle_c, robot_state.height_c);
}

/*------------------------------------
 Detect if object is reachble:
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
 Give reward:
-----------------------------------*/
bool giveReward(){
    return robot_state.folded and robot_state.object_picked;
}

/*------------------------------------
 Print debug:
-----------------------------------*/
void printDebug(string function, int line){
    ROS_INFO("\033[1;31m\nMethod %s:\n\tLine %u\n", function.c_str(), line);
}