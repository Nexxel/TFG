#include "utils.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_robot");

    while(ros::ok()){
        Handlers handlers;
        base = handlers.getNH().advertise<Twist>("/mobile_base/commands/velocity", 1);
        ROS_INFO("Moving back...");
        Twist base_movement; 
        double distance = (double)MAX_DISTANCE/(double)discr_level;
        base_movement.linear.x = 0.1;

        // Create timers
        double time0 = ros::Time::now().toSec();
        while(time0 == 0){
            time0 = ros::Time::now().toSec();
        }

        double necessary_time = (double)distance/(double)abs(base_movement.linear.x);
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
