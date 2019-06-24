#include "learning4.h"

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
            P(i,j) = camera_info_msg->P.at(counter);
            counter++;
        }
    }
    //P(2,3) = 1;
 }

 /*------------------------------------
 Methods
 -----------------------------------*/

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
            and (cv_ptr == NULL or gripper_effort == 0 or counter == 0))
    {
        ros::Rate rate(ROS_RATE);
        ros::spinOnce();
        rate.sleep();
        counter++;
    }
 }

 /*------------------------------------
 Initialize transformation matrix from Sensor frame to Widow-X arm base frame:
 -----------------------------------*/
 void initializeTSB(){
    TSB(0,2) = 1; TSB(0,3) = -0.0734;
    TSB(1,0) = -1; TSB(1,3) = -0.0125;
    TSB(2,1) = 1; TSB(2,3) = -0.2448;
    TSB(3,3) = 1;
 }

/*------------------------------------
 Initialize transformation matrix from image frame to P2:
-----------------------------------*/
 void initializeI2P(){
    I2P(0,0) = 1; 
    I2P(1,1) = -1;I2P(1,2) = 481;
    I2P(2,2) = 1; 
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
        object_center(0) = round(sum_x/pixel_locations.total());
        object_center(1) = round(sum_y/pixel_locations.total());
    }else{
        object_center(0) = -INFINITY;
        object_center(1) = -INFINITY;
    }   
}

/*------------------------------------
 Calculate the real position of the object with respect to the robot:
-----------------------------------*/
void calculateRealPos(){
    int max_u = INFINITY; int max_v = INFINITY;
    int min_u = -INFINITY; int min_v = -INFINITY;
    if(!x_values.empty() && !y_values.empty()){
       min_u = *(x_values.begin()); max_u = *(--x_values.end());
       min_v = *(y_values.begin()); max_v = *(--y_values.end());
    }
    getObjectPosition(max_u,max_v,min_u, min_v);
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
        object_center(0) = -INFINITY; 
        object_center(1) = -INFINITY; 
        ROS_INFO("angle, distance, height: %.10f, %.10f, %.10f",
         robot_state.angle_c, robot_state.distance_c, robot_state.height_c);
    }else{
        // Get the distance of the object
        double f = P(0,0);
        double cx = P(0,2);
        double cy = P(1,2); 
        vec3 bitmap_leftmost_pos;
        bitmap_leftmost_pos << min_u << object_center(1) << 1;
        vec4 sensor_leftmost_pos = f * image2sensor(bitmap_leftmost_pos);

        vec3 bitmap_rightmost_pos;
        bitmap_rightmost_pos << max_u << object_center(1) << 1;
        vec4 sensor_rightmost_pos = f * image2sensor(bitmap_rightmost_pos);

        double width = sensor_rightmost_pos(0) - sensor_leftmost_pos(0);
        d = ((f * OBJECT_WIDTH) / width) / 1000;
        ROS_INFO("width: %.10f, d: %.10f", width, d);

        // Get the pixel position in x,y
        vec3 pixel_pos; // 3 x 1
        vec4 result;    // 4 x 1
        pixel_pos(0) = object_center(0); 
        pixel_pos(1) = object_center(1);
        pixel_pos(2) = 1;
        result = image2sensor(pixel_pos);
        result /= norm(result);
        robot_state.angle_c = result(0) * d; 
        //It should be -0.12, but as we don't see the entire object we have to modify it
        robot_state.height_c = result(1) * d;
        robot_state.distance_c = result(2) * d;
        ROS_INFO("\n\nDistance, Angle, height: \n\t(%.10f, %.10f, %.10f)\n", robot_state.distance_c, robot_state.angle_c, robot_state.height_c);
    }
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
        state_c = &object_center(0);
        state_d = &robot_state.angle_d;
    }else if(selector == 1){
        state_c = &object_center(1);
        state_d = &robot_state.height_d;
    }else{
        state_c = &robot_state.distance_c;
        state_d = &robot_state.distance_d;
    }
    int quadrant = 0;
    bool inside_quadrant = false;
    while (quadrant < discr_level and !inside_quadrant){
        vec2 ranges;
        ranges << step*double(quadrant) << step*double(quadrant+1);
        if(*state_c >= ranges(0)
            and *state_c < ranges(1))
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
Transform a point in the image to the sensor frame
-----------------------------------*/
vec4 image2sensor(vec3 pixel_pos){
    vec4 result;
    vec e = I2P * pixel_pos;
    result(3) = 0;
    result(2) = e(2) - P(2,3);
    result(1) = (e(1) - P(1,2)*e(2) + P(2,3)*P(1,2))/P(1,1);
    result(0) = (e(0) - P(0,2)*e(2) + P(2,3)*P(0,2))/P(0,0);
    return result;
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
 Get the direct kinematic model of the widowX-arm
-----------------------------------*/
void mcd(){
    vec5 alpha;
    alpha << 0 << -M_PI/2 << M_PI << 0 << M_PI/2;
	vec5 a;
    a << 0 << 0 << d2 << L3 << 0;
	vec5 d = zeros<vec>(5);
	vec5 q;
    q << joint_angles(0)
      << joint_angles(1)-beta
      << joint_angles(2)-beta
      << joint_angles(3)+(M_PI/2)
      << joint_angles(4);
	int i  = 0;
	mat44 T01;
    T01 << cos(q(i)) << -sin(q(i)) << 0 << a(i) << endr
		<< sin(q(i))*cos(alpha(i)) << cos(q(i))*cos(alpha(i)) << -0 << -0*d(i) << endr
		<< sin(q(i))*0 << cos(q(i))*0 << cos(alpha(i)) << cos(alpha(i))*d(i) << endr
		<< 0 << 0 << 0 << 1 << endr;
    
	i  = 1;
	mat44 T12;
	T12	<< cos(q(i)) << -sin(q(i)) << 0 << a(i) << endr
		<< sin(q(i))*0 << cos(q(i))*0 << -sin(alpha(i)) << -sin(alpha(i))*d(i) << endr
		<< sin(q(i))*sin(alpha(i)) << cos(q(i))*sin(alpha(i)) << 0 << 0*d(i) << endr
		<< 0 << 0 << 0 << 1 << endr;

	i  = 2;
	mat44 T23;
	T23 << cos(q(i)) << -sin(q(i)) << 0 << a(i) << endr
		<< sin(q(i))*cos(alpha(i)) << cos(q(i))*cos(alpha(i)) << -0 << -0*d(i) << endr
		<< sin(q(i))*0 << cos(q(i))*0 << cos(alpha(i)) << cos(alpha(i))*d(i) << endr
		<< 0 << 0 << 0 << 1 << endr;

	i  = 3;
	mat44 T34;
	T34	<< cos(q(i)) << -sin(q(i)) << 0 << a(i) << endr
		<< sin(q(i))*cos(alpha(i)) << cos(q(i))*cos(alpha(i)) << -0 << -0*d(i) << endr
		<< sin(q(i))*0 << cos(q(i))*0 << cos(alpha(i)) << cos(alpha(i))*d(i) << endr
		<< 0 << 0 << 0 << 1 << endr;

	i  = 4;
	mat44 T45;
	T45	<< cos(q(i)) << -sin(q(i)) << 0 << a(i) << endr
		<< sin(q(i))*0 << cos(q(i))*0 << -sin(alpha(i)) << -sin(alpha(i))*d(i) << endr
		<< sin(q(i))*sin(alpha(i)) << cos(q(i))*sin(alpha(i)) << 0 << 0*d(i) << endr
		<< 0 << 0 << 0 << 1 << endr;
    
	T05 = T01 * T12 * T23 * T34 * T45;
}

/*------------------------------------
 Get the position of the gripper by means of the Direct kinematic model:
 -----------------------------------*/
void getGripperPosition(){
    double ax = T05(0,2);
	double ay = T05(1,2);
	double az = T05(2,2);
	double px = T05(0,3);
	double py = T05(1,3);
	double pz = T05(2,3);

	gripper_position(0) = px + L45*ax;
	gripper_position(1) = py + L45*ay;
	gripper_position(2) = pz + L45*az;

    ang_or(0) = ax;
    ang_or(1) = ay;
    ang_or(2) = az;
}

/*------------------------------------
 Get the angle of each joint in order to reach the desired position
 by means of the inverse kinematic model:
    Inputs:
        - next_position: Desired position
        - n: Desired orientation of the wrisp 
 -----------------------------------*/
void mci(vec3 next_position, vec3 n){
	double px = next_position(0) - L45*ang_or(0);
	double py = next_position(1) - L45*ang_or(1);
	double pz = next_position(2) - L45*ang_or(2);

	double q1 = atan2(py, px);
            
	double k = pow(pz, 2) + pow(d2, 2) + pow(((px * cos(q1)) + (py * sin(q1))), 2) - pow(L3, 2);
	double k1 = 2 * d2 * px * cos(q1) + 2 * py * d2 * sin(q1);
	double k2 = 2 * pz * d2;
    
	double theta2b = atan2(k1, k2) - atan2(k, -sqrt(pow(k1,2)+pow(k2,2)-pow(k,2)));
	double q2 = theta2b + beta;

	double theta23 = asin((-pz - d2*sin(theta2b))/L3);
	double q3 = q2 - theta23;

	double L = ang_or(2)*cos(q2-q3) + ang_or(0)*sin(q2-q3)*cos(q1) + ang_or(1)*sin(q2-q3)*sin(q1);
	double q4 = acos(-L) - (M_PI/2);

	double q5 = asin(n(0)*sin(q1) - n(1)*cos(q1));

    ROS_INFO("\n\nk: %.2f, k1: %.2f, k2: %.2f, theta2b: %.2f, theta23: %.2f, L: %.2f\n", k, k1, k2, theta2b, theta23, L);
    ROS_INFO("\n\nq1: %.2f\nq2: %.2f\nq3: %.2f\nq4: %.2f\nq5: %.2f\n", q1,q2,q3,q4,q5);

    if(!(isnan(q1) || isnan(q2) || isnan(q3) || isnan(q4) || isnan(q5))){
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
        joint_angles(0) = q1;
        joint_angles(1) = q2;
        joint_angles(2) = q3;
        joint_angles(3) = q4;
        joint_angles(4) = q5;
        robot_state.folded = (next_position(0) == 0
                            and next_position(1) == 0
                            and next_position(2) == 0);
    }else{
        ROS_INFO("Object not reachable.");
    }

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
 Set next position:
   Inputs:
      x, y, z: The coordinates of the next position
-----------------------------------*/
vec3 setNextPosition(double x, double y, double z){
    vec3 next_position;
    next_position(0) = x;
    next_position(1) = y;
    next_position(2) = z;
    return next_position;
}

/*------------------------------------
 Fold arm:
-----------------------------------*/
void foldArm(){
    vec3 n;
    n << 0 << 0 << 1; 
    vec3 next_position;
    // Turn the arm to the position (0.3125,0,0.1450)
    next_position = setNextPosition(
                     0.3125,
                     0, 
                     0.1450);
    mci(next_position,n);

    robot_state.folded = true;
}