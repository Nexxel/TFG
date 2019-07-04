#include "utils.cpp"
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
 Make all the process of learning:
    While node is running:
        1. Get state
        2. Detect if it is reachable
        3. Move arm if correct, else move base
        4. Fold arm
        5. Check reward
 -----------------------------------*/
void learning(Handlers handlers){
    initializeVecMat();
    readLog();
    cout << "Do you want just to exploit?[y|N] ";
    getline(cin, exploit);

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
        bool end_simulation = false;
        while(ros::ok() && !end_simulation){
            // Set random seed by the time of the cpu
            srand( (unsigned)time(NULL) );
            robot_state.angle_d = 0;
            robot_state.height_d = 0;
            robot_state.distance_d = 0;
            robot_state.angle_c = 0;
            robot_state.height_c = 0;
            robot_state.distance_c = 0;
            
            processMessages();

            updateState();

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
            selectAction();

            // 3.1 Move arm if reachable
            if(action == 4){
                ROS_INFO("Moving arm...");
                moveArmToObject();
                isObjectReachable();
                if(object_reachable){
                    end_simulation = true;
                }
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

            if (exploit != "y"){
                // 5. Check reward
                double reward = calculateReward();
                
                // Update Q-matrix
                q_matrix(sa,action) = (1 - ALPHA) * q_matrix(sa,action) + ALPHA * (reward + GAMMA * V(sp));

                // Update visit matrix
                visit_matrix(action)++;

                // Update V and policy matrices
                updateVPolicy();
                steps++;
                actualizeLog();
                actualizeSimplifiedLog();
            }
            sa = sp;
        }
        if (gui){
            destroyWindow("Red objects image");
        }
        killSimulation();
        steps = 0;
    }
}