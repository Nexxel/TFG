/*
Sergio Gonzalez Muriel
Degree thesis:  Reinforcement learning for object manipulation by a robotic arm
Test code for moving the robot Widow-X arm
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include "Kinematic_models.cpp"

using namespace std;

ros::Publisher arm_1;
ros::Publisher arm_2;
ros::Publisher arm_3;
ros::Publisher arm_4;
ros::Publisher arm_5;

double T05[4][4];
double pr[3];
double q[5];

void callbackJointState(const sensor_msgs::JointStateConstPtr& msg){
    if (msg->header.frame_id.empty()){
        double q1 = msg->position[0]; double q2 = msg->position[1]; double q3 = msg->position[2];
        double q4 = msg->position[3]; double q5 = msg->position[4];

        
        mcd(T05, pr, q1, q2, q3, q4, q5);
    }  
}

void callbackMoveArm(const std_msgs::StringConstPtr& msg){
    if(pr != NULL){
        string order = msg->data;
        if(order == "UP"){
            pr[2] += 0.05;
            ROS_INFO("Moving arm up");
        }else if(order == "DOWN"){
            pr[2] -= 0.05;
            ROS_INFO("Moving arm down");
        }else if(order == "LEFT"){
            pr[1] += 0.05;
            ROS_INFO("Moving arm left");
        }else if(order == "RIGHT"){
            pr[1] -= 0.05;
            ROS_INFO("Moving arm right");
        }else if(order == "FRONT"){
            pr[0] += 0.05;
            ROS_INFO("Moving arm front");
        }else if(order == "BACK"){
            pr[0] -= 0.05;
            ROS_INFO("Moving arm back");
        }else{}

        mci(q, pr);

        std_msgs::Float64 angle;
        angle.data = q[0];
        arm_1.publish(angle);
        angle.data = q[1];
        arm_2.publish(angle);
        angle.data = q[2];
        arm_3.publish(angle);
        angle.data = q[3];
        arm_4.publish(angle);
        angle.data = q[4];
        arm_5.publish(angle);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_armx");
    ROS_INFO("Waiting for commands...");
    ros::NodeHandle n;

    ros::Subscriber joint_state = n.subscribe("/joint_states", 1000, callbackJointState);
    ros::Subscriber direction_arm = n.subscribe("/move_armx/direction", 1000, callbackMoveArm);
    arm_1 = n.advertise<std_msgs::Float64>("/arm_1_joint/command", 1000);
    arm_2 = n.advertise<std_msgs::Float64>("/arm_2_joint/command", 1000);
    arm_3 = n.advertise<std_msgs::Float64>("/arm_3_joint/command", 1000);
    arm_4 = n.advertise<std_msgs::Float64>("/arm_4_joint/command", 1000);
    arm_5 = n.advertise<std_msgs::Float64>("/arm_5_joint/command", 1000);

    ros::spin();
    return 0;
}
