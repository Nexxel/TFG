/*
Sergio Gonzalez Muriel
Degree thesis:  Reinforcement learning for object manipulation by a robotic arm
Implementation of all funtionalities for the Q-Learning implementation
*/

#include "learning.h"

/*------------------------------------
 Callbacks
 -----------------------------------*/

 /*------------------------------------
 Gets the rgb image and processes it:
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

    // Converts image from color to B&W being white the red object
        
    cv_ptr->image = ~(cv_ptr->image);
        
    Mat3b imageHSV;
    // If it is not a simulation, applies filter for avoiding reflections
    if(!is_simulation){
        GaussianBlur(cv_ptr->image, cv_ptr->image, Size(15,15), 7, 7); // Size(9,9), 4, 4
    }
    // Gets the red object
    cvtColor(cv_ptr->image, imageHSV, COLOR_BGR2HSV);
    inRange(imageHSV, Scalar(90 - 10, 100, 100), Scalar(90 + 10, 255, 255), cv_ptr->image);
    // Applies filter for avoiding false positives
    GaussianBlur(cv_ptr->image, cv_ptr->image, Size(3,3), 3, 3);
}

 /*------------------------------------
 Gets the rgb camera info and gets the projection matrix:
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
 Gets gripper effort:
 -----------------------------------*/
 void getGripperEffortCallback(const JointStateConstPtr& joint_states_msg){
    if (joint_states_msg->header.frame_id.empty()){
        gripper_effort = joint_states_msg->effort[5];
    }
 }

  /*------------------------------------
 Gets simulation pose and actualizes it:
 -----------------------------------*/
 void getSimulationPoseCallback(const Vector3ConstPtr& sim_pose_msg){
     if(update_pose){
        actualizePositionLog(sim_pose_msg->x, sim_pose_msg->y, sim_pose_msg->z);
        update_pose = false;
     }
 }

 /*------------------------------------
 Methods
 -----------------------------------*/

/*------------------------------------
 Initializes vectors and matrices
 -----------------------------------*/
 void initializeVecMat(){
    home_pos << 0.3125 << 0 << 0.1450;
    n << 1 << 0 << 0;
    initializeI2P();
    initializeTSB();
 }

/*------------------------------------
 Initializes learning elements reading from a log file
 -----------------------------------*/
 void readLog(){
    ifstream input_log;
    log_name.clear();
    cout << "Specify the log to read/write without extension: ";
    getline(cin, log_name);
    complete_log_name << ros::package::getPath("learning") << "/logs/log_" << log_name << ".txt";
    complete_simplified_log_name << ros::package::getPath("learning") << "/simplified_logs/simplified_log_" << log_name << ".txt";
    complete_exploitation_log_name << ros::package::getPath("learning") << "/exploitation_logs/exploitation_log_" << log_name << ".txt";
    complete_distance_log_name << ros::package::getPath("learning") << "/distance_logs/distance_log_" << log_name << ".txt";
    complete_iteration_distance_log_name << ros::package::getPath("learning") << "/iteration_distance_logs/iteration_distance_log_" << log_name << ".txt";
    complete_position_log_name << ros::package::getPath("learning") << "/position_logs/position_log_" << log_name << ".txt";
    complete_object_position_log_name << ros::package::getPath("learning") << "/object_position_logs/object_position_log_" << log_name << ".txt";

    cout << "Want to overwrite it?[y/N] ";
    string response;
    getline(cin, response);
    if(response != "y"){
        string str(complete_simplified_log_name.str());
        input_log.open(str.c_str(), ios::in);
        string line, word, temp;
        line.resize(100);
        while(getline(input_log, line)){
            vec row = arma::zeros<vec>(20);
            stringstream s(line);
            cout << "Line: " << line << '\n'; 
            int counter = 0;
            while(getline(s, word, ',')){
                if (counter != 15){
                    if(counter < 15){
                        row(counter) = atof(word.c_str());
                    }else{
                        row(counter-1) = atof(word.c_str());
                    }
                }
                counter++;
            }

            simulations = row(0);
            //steps = row(1);
            sa = row(2);
            sp = row(8);
            getStateFromIndex(sp);
            action = row(14);
            reward = row(15);
            visit_matrix(sa, action) = row(16);
            q_matrix(sa, action) = row(17);
            V(sa) = row(18);
            policy_matrix(sa) = row(19);
            //number_steps++;
        }
        input_log.close();
    } 
 }

/*------------------------------------
 Processes messages:
 -----------------------------------*/
 void processMessages(){
     // The counter if for ashuring that at least it is executed 1 time
     int counter = 0;
     gripper_effort = 0;
     cv_ptr.reset();
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
 Initializes transformation matrix from Sensor frame to Widow-X arm base frame:
 -----------------------------------*/
 void initializeTSB(){
    TSB(0,2) = 1; TSB(0,3) = -0.0734;
    TSB(1,0) = -1; TSB(1,3) = -0.0125;
    TSB(2,1) = 1; TSB(2,3) = -0.2448;
    TSB(3,3) = 1;
 }

/*------------------------------------
 Initializes transformation matrix from image frame to P2:
-----------------------------------*/
 void initializeI2P(){
    I2P(0,0) = 1; 
    I2P(1,1) = -1;I2P(1,2) = 481;
    I2P(2,2) = 1; 
 }

 /*------------------------------------
 Updates state:
-----------------------------------*/
void updateState(){
    getLocation();
    calculateRealPos();
    discretizeValues();
    mcd();
    //isObjectPicked();
    isObjectReachable();
}

/*------------------------------------
 Gets the location in pixels of the object:
-----------------------------------*/
void getLocation(){
    if (gui){
        // Shows the image
        imshow("Red objects image", cv_ptr->image);
        waitKey(3);
    }
    
    // Gets the coordinates of the object pixels
    findNonZero(cv_ptr->image, pixel_locations);

    // Calculates pixel mean in x and y
    sum_x = 0; sum_y = 0; x_values.clear(); y_values.clear();
    int prev_y = -1; int prev_x = -1;
    max_u = 0; max_v = 0;
    min_u = 0; min_v = 0;
    for(int i = 0; i<pixel_locations.total(); i++){
        Point pixel = pixel_locations.at<Point>(i);
        sum_x += pixel.x;
        sum_y += pixel.y;
        //cout << "\npixel.x: " << pixel.x << "\tpixel.y: " << pixel.y;
        x_values.insert(pixel.x);
        y_values.insert(pixel.y);
        if(prev_y != pixel.y){
            min_u += pixel.x;
            if(prev_y != -1){
                max_u += prev_x;
            }
        }
        if(i == (pixel_locations.total() -1)){
            max_u += pixel.x;
        }
        prev_y = pixel.y;
        prev_x = pixel.x;
    }
    if (pixel_locations.total() != 0){
        cout << "\n\n\nmin_u: " << min_u << "\tmax_u: " << max_u << "\n\n";
        min_u /= y_values.size();
        max_u /= y_values.size();
        object_center(0) = round(sum_x/pixel_locations.total());
        object_center(1) = round(sum_y/pixel_locations.total());
    }else{
        object_center(0) = -INFINITY;
        object_center(1) = -INFINITY;
    }   
}

/*------------------------------------
 Calculates the real position of the object with respect to the robot:
-----------------------------------*/
void calculateRealPos(){
    if(!x_values.empty() && !y_values.empty()){
       min_v = *(y_values.begin()); max_v = *(--y_values.end());
       ROS_INFO("width in pixels = %.2f", abs(max_u-min_u));
    }else{
        max_u = INFINITY; max_v = INFINITY;
        min_u = -INFINITY; min_v = -INFINITY;
    }
    getObjectPosition();
}

/*------------------------------------
Gets object real position with respect to the sensor frame where:
    max_u: Max X coordinate of the center of the object (Pixels)
    max_v: Max Y coordinate of the center of the object (Pixels)
    min_u: Min X coordinate of the center of the object (Pixels)
    min_v: Min Y coordinate of the center of the object (Pixels)
-----------------------------------*/
void getObjectPosition(){
    if ((max_u >= (cv_ptr->image.cols - 42)) || (min_u <= 42)){
        robot_state.angle_c = -INFINITY;
        robot_state.distance_c = -INFINITY;
        robot_state.height_c = -INFINITY;
        object_center(0) = -INFINITY; 
        object_center(1) = -INFINITY; 
        ROS_INFO("angle, distance, height: %.10f, %.10f, %.10f",
         robot_state.angle_c, robot_state.distance_c, robot_state.height_c);
    }else{
        // Gets the distance of the object
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
        dist = ((f * OBJECT_WIDTH) / width) / 1000;
        ROS_INFO("width: %.10f, real distance: %.10f", width, dist);

        // Gets the pixel position in x,y
        vec3 pixel_pos; // 3 x 1
        vec4 result;    // 4 x 1
        pixel_pos(0) = object_center(0); 
        pixel_pos(1) = object_center(1);
        pixel_pos(2) = 1;
        result = image2sensor(pixel_pos);
        result /= norm(result);
        robot_state.angle_c = result(0) * dist; 
        //It should be -0.12, but as we don't see the entire object we have to modify it
        robot_state.height_c = result(1) * dist;
        robot_state.distance_c = result(2) * dist;
        ROS_INFO("\n\nDistance, Angle, height: \n\t(%.10f, %.10f, %.10f)\n", robot_state.distance_c, robot_state.angle_c, robot_state.height_c);
    }
}

/*------------------------------------
 Discretizes values:
-----------------------------------*/
void discretizeValues(){
    double angle_step = double(cv_ptr->image.cols)/double(discr_level);
    double height_step = double(cv_ptr->image.rows)/double(discr_level);
    double depth_step = double(MAX_DISTANCE)/double(discr_level);
    
    // Discretizes values in distance
    discretizeValuesAux(2, depth_step);
    // Discretizes values in angle
    discretizeValuesAux(0,angle_step);
    // Discretizes values in height
    discretizeValuesAux(1, height_step);
}

/*------------------------------------
 Discretizes values auxiliar:
    Inputs:
        selector:
            0: angle
            1: height
            2: depth
        step: difference between each level of discretization
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
    *state_d = 0;
    while (quadrant < discr_level and !inside_quadrant and *state_c >= 0){
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
Transforms a point in the image to the sensor frame
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
 Checks if we are executing a gazebo simulation:
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
 Gets the direct kinematic model of the Widow-X arm
-----------------------------------*/
void mcd(){
    vec5 alpha;
    alpha << 0 << -M_PI/2 << M_PI << 0 << M_PI/2;
	vec5 a;
    a << 0 << 0 << d2 << L3 << 0;
	vec5 d = arma::zeros<vec>(5);
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
    getGripperPosition();
}

/*------------------------------------
 Gets the position of the gripper by means of the Direct kinematic model:
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
Gets the angle of each joint in order to reach the desired position
by means of the inverse kinematic model, sending a message to the servos with the angle to reach
and updating the mcd and sensor data:
Inputs:
    next_position: Desired position
 -----------------------------------*/
void mci(vec3 next_position){
	double px = next_position(0) - L45*ang_or(0);
	double py = next_position(1) - L45*ang_or(1);
	double pz = next_position(2) - L45*ang_or(2);

    ROS_INFO("px: %.10f, py: %.10f, pz: %.10f", px, py, pz);
    ROS_INFO("ang_or: [%.10f %.10f %.10f]", ang_or(0), ang_or(1), ang_or(2));
    cout << T05;

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
}

/*------------------------------------
 Moves Widow-X arm to object:
 -----------------------------------*/
void moveArmToObject(){
    vec4 hom_obj_pos;
    hom_obj_pos << robot_state.angle_c << robot_state.height_c << robot_state.distance_c << 1;
    vec3 intermediate_position = home_pos;
    vec4 next_position;
    next_position = (TSB * (hom_obj_pos));
    intermediate_position(0) = next_position(0) - 0.05;
    mci(intermediate_position);
    openGripper();
    ros::Duration(2).sleep();

    intermediate_position(0) = next_position(0) - 0.025; 
    intermediate_position(1) = next_position(1);
    intermediate_position(2) = next_position(2) + 0.06;
    mci(intermediate_position);
    ros::Duration(2).sleep();

    next_position(2) += 0.06;
    cout << "Next position: \t" << next_position; 
    mci(next_position.rows(0,2));
    ros::Duration(2).sleep();

    closeGripper();
    ros::Duration(7).sleep();

    processMessages();
    updateState();
}

/*------------------------------------
 Opens gripper:
 -----------------------------------*/
void openGripper(){
    Float64 gripper_value; gripper_value.data = 2.5;
    gripper.publish(gripper_value);
    gripper_opened = true;
    processMessages();
}

/*------------------------------------
 Closes gripper:
 -----------------------------------*/
 void closeGripper(){
    Float64 gripper_value; gripper_value.data = 0;
    gripper.publish(gripper_value);
    gripper_opened = false;
    processMessages();
}

/*------------------------------------
 Detects picked object:
-----------------------------------*/
void isObjectPicked(){
    robot_state.object_picked = ((!gripper_opened) 
                and (abs(gripper_effort) > MAX_EFFORT)) 
                || 
                (is_simulation * (object_reachable && action == 4));
    ROS_INFO("gripper_effort: %.4f", gripper_effort);
}

/*------------------------------------
 Detects if object is reachable:
    Object is centered in x and near
-----------------------------------*/
void isObjectReachable(){
    object_reachable = robot_state.angle_d >= round((1 + discr_level)/3)
                        and robot_state.angle_d <= round(2 * (1 + discr_level)/3)
                        and robot_state.distance_d <= 3; //(1 + discr_level)/4;
}

/*------------------------------------
 Sets next position:
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
 Folds arm:
-----------------------------------*/
void foldArm(){ 
    // Turn the arm to the position (0.3125,0,0.1450)
    mci(home_pos);

    robot_state.folded = true;
}

/*------------------------------------
 Starts a new random simulation:
-----------------------------------*/
void startRandomSimulation(){
    int status;
        // Opens an empty world in gazebo
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
            // Instantiates a turtlebot in that empty world
            system("xterm -hold -e \"roslaunch crumb_gazebo test.launch\" &");
            sleep(10);
            // Unpauses simulation
            system("rosservice call /gazebo/unpause_physics");
            sleep(5);
            simulations++;
            actualizeObjectPositionLog((x - 0.45), y, (z*2+0.05));
        }else{
            system("killall -9 xterm gzserver");
            ros::shutdown();
        }
}
/*------------------------------------
 Kills the current simulation:
-----------------------------------*/
void killSimulation(){
    ROS_INFO("Killing actual simulation...");
    sleep(2);
    // Pauses simulation
    system("rosservice call /gazebo/pause_physics");
    sleep(1);
    // Kills processes
    system("killall -9 xterm gzserver");
    sleep(3); 
}
/*------------------------------------
 Gets the matrix column index from the robot state:
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
 Modifies the state of the robot from the column index:
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
 Selects action:
-----------------------------------*/
void selectAction(){
    if (exploit == "y" || floor(unifRnd(0, 100)) > EXPLORATION_RATE){
        ROS_INFO("Exploting...");
        double maximum = arma::max(q_matrix.row(sa));
        cout << "\n\nMaximum: " << maximum << "\n\n"; 
        cout << "\n\nq_matrix_max:\n" << arma::find((q_matrix.row(sa) == maximum))
             << "\nvisit_matrix > 0:\n" << arma::find((visit_matrix.row(sa) > 0))
             << "\nBoth:\n" << (arma::find((q_matrix.row(sa) == maximum) and (visit_matrix.row(sa) > 0)));
        uvec maximum_values_pos_visited = (arma::find((q_matrix.row(sa) == maximum) and (visit_matrix.row(sa) > 0)));
        uvec maximum_values_pos = (arma::find((q_matrix.row(sa) == maximum)));
        if(maximum_values_pos_visited.n_rows == 0){
            cout << "\n\nMaximum non visited: " << maximum_values_pos << "\n\n";  
            action = maximum_values_pos.at(floor(unifRnd(0,maximum_values_pos.n_rows-1)));
        }else{
            cout << "\n\nMaximum visited: " << maximum_values_pos_visited << "\n\n";
            action = maximum_values_pos_visited.at(floor(unifRnd(0,maximum_values_pos_visited.n_rows-1)));
        }
    }else{
        ROS_INFO("Exploring...");
        action = floor(unifRnd(0,N_ACTIONS-1));
    }
}
/*------------------------------------
 Updates V and policy matrix:
-----------------------------------*/
void updateVPolicy(){
    V(sa) = q_matrix(sa,0);
    policy_matrix(sa) = 0;
    for(int i = 1; i < N_ACTIONS; i++){
        if(q_matrix(sa,i) > V(sa)){
            V(sa) = q_matrix(sa,i);
            policy_matrix(sa) = i;
        }
    }
}
/*------------------------------------
 Calculates reward:
-----------------------------------*/
void calculateReward(){
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
    reward = 0;
    if((act_dist == 0 || act_ang == 0 || act_height == 0) &&
        (prev_dist > 0 && prev_ang > 0 && prev_height > 0)){
            reward -= 3;
    }else if((prev_dist == 0 || prev_ang == 0 || prev_height == 0) &&
        (act_dist > 0 && act_ang > 0 && act_height > 0)){
            reward += 3;
    }
    
    if(act_dist > 0 && prev_dist != 0 && act_dist < prev_dist){
        reward += 5;
    }else if(prev_dist > 0 && act_dist != 0 && act_dist > prev_dist){
        reward -= 5;
    }
    if((prev_dist > 0 && prev_ang > 0 && prev_height > 0)
        && (act_ang != prev_ang && act_ang > 0)){
        if(abs(act_ang - ceil(discr_level/2)) > abs(prev_ang - ceil(discr_level/2))){
            reward -= abs(act_ang - ceil((double)discr_level/2.0));
        }else{
            reward += abs(prev_ang - ceil((double)discr_level/2.0));
        }
    }
    
    reward += 100 * robot_state.object_picked;
}

/*------------------------------------
 Actualizes log:
-----------------------------------*/
void actualizeLog(){
    if (steps == 1 && simulations == 1){
        string str(complete_log_name.str());
        log_file.open(str.c_str());
    }else{
        string str(complete_log_name.str());
        log_file.open(str.c_str(), ios::app | ios::out);
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
    log_file << "New value of Visit matrix: " << visit_matrix(sa,action) << "\n";
    log_file << "New value of Q matrix: " << q_matrix(sa,action) << "\n";
    log_file << "New value of Value function: " << V(sa) << "\n";
    log_file << "New value of Policy matrix: " << policy_matrix(sa) << "\n\n";
    log_file.close();
}

/*------------------------------------
 Actualizes simplified log:
-----------------------------------*/
void actualizeSimplifiedLog(){
    if (steps == 1 && simulations == 1){
        string str(complete_simplified_log_name.str());
        log_file.open(str.c_str());
    }else{
        string str(complete_simplified_log_name.str());
        log_file.open(str.c_str(), ios::app | ios::out);
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
    log_file << visit_matrix(sa,action) << ",";
    log_file << q_matrix(sa,action) << ",";
    log_file << V(sa) << ",";
    log_file << policy_matrix(sa) << "\n";
    log_file.close();
}

/*------------------------------------
 Actualizes log for exploitation:
-----------------------------------*/
void actualizeExploitationLog(){
    if (steps == 1 && simulations == 1){
        string str(complete_exploitation_log_name.str());
        log_file.open(str.c_str());
    }else{
        string str(complete_exploitation_log_name.str());
        log_file.open(str.c_str(), ios::app | ios::out);
    }
    log_file << sa << "," << action << "\n";
    log_file.close();
}

/*------------------------------------
 Actualizes log for distances:
-----------------------------------*/
void actualizedistanceLog(){
    if (simulations == 1){
        string str(complete_distance_log_name.str());
        log_file.open(str.c_str());
    }else{
        string str(complete_distance_log_name.str());
        log_file.open(str.c_str(), ios::app | ios::out);
    }
    log_file << d << "," << e << "\n";
    log_file.close();
}

/*------------------------------------
 Actualizes log for distances each iteration:
-----------------------------------*/
void actualizeIterationDistanceLog(){
    if (steps == 1 && simulations == 1){
        string str(complete_iteration_distance_log_name.str());
        log_file.open(str.c_str());
    }else{
        string str(complete_iteration_distance_log_name.str());
        log_file.open(str.c_str(), ios::app | ios::out);
    }
    log_file << simulations << "," << steps << "," << d << "," << e << "\n";
    log_file.close();
}

/*------------------------------------
 Actualizes log for position of robot each iteration:
-----------------------------------*/
void actualizePositionLog(double x, double y, double z){
    if (steps == 1 && simulations == 1){
        string str(complete_position_log_name.str());
        log_file.open(str.c_str());
    }else{
        string str(complete_position_log_name.str());
        log_file.open(str.c_str(), ios::app | ios::out);
    }
    log_file << simulations << "," << steps << "," << x << "," << y << "," << z << "\n";
    log_file.close();
}

/*------------------------------------
 Actualizes log for position of object on each simulation:
-----------------------------------*/
void actualizeObjectPositionLog(double x, double y, double z){
    if (steps == 1 && simulations == 1){
        string str(complete_object_position_log_name.str());
        log_file.open(str.c_str());
    }else{
        string str(complete_object_position_log_name.str());
        log_file.open(str.c_str(), ios::app | ios::out);
    }
    log_file << simulations << "," << x << "," << y << "," << z << "\n";
    log_file.close();
}

/*------------------------------------
 Prints debug:
    Inputs:
        function: Function where you are
        line: Line in which you are
-----------------------------------*/
void printDebug(string function, int line){
    ROS_INFO("\033[1;31m\nMethod %s:\n\tLine %u\n", function.c_str(), line);
}

/*------------------------------------
 Gets random number following an uniform distribution:
    Inputs:
        min, max: Min and max values
-----------------------------------*/
double unifRnd(double min, double max){
    return min + ((double)rand()/(double)RAND_MAX) * ((max+1) - min);
}