/*
Sergio Gonzalez Muriel
Degree thesis:  Reinforcement learning for object manipulation by a robotic arm
Test code for turning the robot
*/

#include "utils.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turn_robot");

    while(ros::ok()){
        Handlers handlers;
        base = handlers.getNH().advertise<Twist>("/mobile_base/commands/velocity", 1);
        ROS_INFO("Turning left...");
        Twist base_movement; 
        double angle = ((double)48.6/(double)discr_level) * (M_PI/180);
        base_movement.angular.z = 0.1;

        // Creates timers
        double time0 = ros::Time::now().toSec();
        while(time0 == 0){
            time0 = ros::Time::now().toSec();
        }

        double necessary_time = (double)angle/(double)abs(base_movement.angular.z);
        double diff_time = 0;
        int counter = 0;
        while((diff_time < necessary_time) && ros::ok()){
            if(counter == 0){
                time0 = ros::Time::now().toSec();
                counter++;
            }
            base.publish(base_movement);
            diff_time = ros::Time::now().toSec() - time0;
            cout << "time1 - time0: " << diff_time << " necessary_time: " << necessary_time << "\n";
        }
        ros::shutdown();
        return 0;
    }  
}
