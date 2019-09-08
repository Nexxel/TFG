/*
Sergio Gonzalez Muriel
Degree thesis:  Reinforcement learning for object manipulation by a robotic arm
Test code for moving and turning the robot base
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

using namespace std;

ros::Publisher base;

void callbackMoveArm(const std_msgs::StringConstPtr& msg){
    // Mirar más detenidamente
        string order = msg->data;
        geometry_msgs::Twist command;
        if(order == "LEFT"){
            command.angular.z += 0.1;
            ROS_INFO("Turning base to the left");
        }else if(order == "RIGHT"){
            command.angular.z -= 0.1;
            ROS_INFO("Turning base to the right");
        }else if(order == "FRONT"){
            command.linear.x += 0.1;
            ROS_INFO("Moving base to the front");
        }else if(order == "BACK"){
            command.linear.x -= 0.1;
            ROS_INFO("Moving base to the back");
        }else{
            ROS_INFO("Not possible action");
        }

        base.publish(command);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_base");
    ROS_INFO("Waiting for commands...");
    ros::NodeHandle n;

    ros::Subscriber direction_arm = n.subscribe("/move_base/direction", 1000, callbackMoveArm);
    base = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

    ros::spin();
    return 0;
}