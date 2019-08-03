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
    string gui_response;
    cout << "Do you want to watch the simulation?[y|N] ";
    getline(cin, gui_response);
    if(gui_response == "y"){
        gui = true;
    }

    bool end_simulation = false;

    while(ros::ok() && !end_simulation){
        // Set random seed by the time of the cpu
        srand( (unsigned)time(NULL) );
        startRandomSimulation();
        sleep(3);
        if (gui){
            namedWindow("Red objects image",CV_WINDOW_AUTOSIZE);
        }
        int counter = 0;

        // Create timers
        double time0 = ros::Time::now().toSec();
        while(time0 == 0){
            time0 = ros::Time::now().toSec();
        }

        robot_state.object_picked = false;
        robot_state.folded = false;

        // Initialize all publishers and subscribers
        ros::master::getTopics(topic_info);
        isSimulation();
        color_image_sub = handlers.getIT().subscribe("/camera/rgb/image_color", 1, &callbackImage);
        camera_info_sub = handlers.getNH().subscribe("/camera/rgb/camera_info", 1, &callbackCameraInfo);
        joint_states_sub = handlers.getNH().subscribe("/joint_states", 1, &getGripperEffortCallback);
        sim_pose_sub = handlers.getNH().subscribe("/simulation/pose", 1 , &getSimulationPoseCallback);

        joints[0] = handlers.getNH().advertise<Float64>("/arm_1_joint/command", 1);
        joints[1] = handlers.getNH().advertise<Float64>("/arm_2_joint/command", 1);
        joints[2] = handlers.getNH().advertise<Float64>("/arm_3_joint/command", 1);
        joints[3] = handlers.getNH().advertise<Float64>("/arm_4_joint/command", 1);
        joints[4] = handlers.getNH().advertise<Float64>("/arm_5_joint/command", 1);

        gripper = handlers.getNH().advertise<Float64>("/gripper_1_joint/command", 1);
        base = handlers.getNH().advertise<Twist>("/mobile_base/commands/velocity", 1);

        steps = 0;
        prev_V = V;
        robot_state.angle_d = 0;
        robot_state.height_d = 0;
        robot_state.distance_d = 0;
        robot_state.angle_c = 0;
        robot_state.height_c = 0;
        robot_state.distance_c = 0;

        bool end_episode = false;
        while(ros::ok() && !end_episode){
            update_pose = true;
            // Update state
            processMessages();
            updateState();
            update_pose = false;

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
                if(object_reachable){
                    ROS_INFO("Moving arm...");
                    moveArmToObject();
                    if(exploit != "y"){
                        end_episode = true;
                    }
                    robot_state.object_picked = true;
                }else{
                    ROS_INFO("Trying to move arm but object is not reachable...");
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
                if(robot_state.distance_d != 0 && robot_state.angle_d != 0 && robot_state.height_d != 0){
                    int prev_dist = robot_state.distance_d;
                    int prev_ang = robot_state.angle_d;
                    bool changed_state = false;
                    int counter = 0;
                    while(!changed_state && counter < 7){
                        base.publish(base_movement);
                        ros::Duration(3).sleep();
                        processMessages();
                        updateState();
                        if(action == 0 || action == 1){
                            changed_state = (prev_ang != robot_state.angle_d);
                        }else{
                            changed_state = (prev_dist != robot_state.distance_d);
                        }
                        counter++;
                    }
                }else{
                    double necessary_time;
                    if(action == 0 || action == 1){
                        necessary_time = (double)angle_per_level/(double)abs(base_movement.angular.z);
                    }else{
                        necessary_time = (double)distance_per_level/(double)abs(base_movement.linear.x);
                    }
                    double diff_time = 0;
                    bool first_loop = true;
                    while((diff_time < necessary_time) && ros::ok()){
                        if(first_loop){
                            time0 = ros::Time::now().toSec();
                            first_loop = false;
                        }
                        base.publish(base_movement);
                        diff_time = ros::Time::now().toSec() - time0;
                    }
                }      
            }
            ros::Duration(3).sleep();
            // Update state
            processMessages();
            updateState();

            sp = getIndexFromState();

            if (exploit != "y"){
                
                // 5. Check reward
                calculateReward();

                // Update visit matrix
                visit_matrix(sa, action)++;
                
                // Update Q-matrix
                alpha = 1/(pow(visit_matrix(sa,action),0.5));
                q_matrix(sa,action) = (1 - alpha) * q_matrix(sa,action) + alpha * (reward + GAMMA * V(sp));


                // Update V and policy matrices
                vec prev_V_it = V;      // Prev V function on each iteration
                updateVPolicy();
                d = norm(V - prev_V_it);
                e = arma::min(arma::abs(V - prev_V_it));
                actualizeIterationDistanceLog(); 

                steps++;
                actualizeLog();
                actualizeSimplifiedLog();
                if(steps == 200){
                    end_episode = true;
                }
            }else{
                actualizeExploitationLog();
            }
            sa = sp;
        }
        if (gui){
            destroyWindow("Red objects image");
        }
        killSimulation();
        d = norm(V - prev_V);
        e = arma::min(arma::abs(V - prev_V));
        actualizedistanceLog(); 
        if(d < 0 || e < 0){
            end_simulation = true;
            ROS_INFO("The robot has already learn. End of simulation...");
        }
    }
}