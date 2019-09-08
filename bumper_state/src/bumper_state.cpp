/*
Sergio Gonzalez Muriel
Degree thesis:  Reinforcement learning for object manipulation by a robotic arm
Test code for getting the bumper state
*/
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <kobuki_msgs/BumperEvent.h>

using namespace std;

ros::Publisher bumper_state;
std_msgs::String state;

void callbackJointState(const kobuki_msgs::BumperEventConstPtr& msg){
    
    uint8_t bumper = msg->bumper; uint8_t pressed = msg->state;
    if(pressed == msg->RELEASED){
        state.data = "EL ROBOT HA DEJADO DE CHOCAR";
    }else{
        state.data = "EL ROBOT HA CHOCADO POR ";
        if(bumper == msg->LEFT){
            state.data += "LA IZQUIERDA";
        }else if(bumper == msg->CENTER){
            state.data += "EL CENTRO";
        }else{
            state.data += "LA DERECHA";
        }
    } 
    bumper_state.publish(state);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bumper_state");
    ROS_INFO("Getting bumper state...");
    ros::NodeHandle n;

    ros::Subscriber joint_state = n.subscribe("/mobile_base/events/bumper", 1000, callbackJointState);
    bumper_state = n.advertise<std_msgs::String>("/bumper_state", 1000);

    ros::spin();
    return 0;
}