#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

using namespace std;

ros::Publisher gripper_state;

void callbackJointState(const sensor_msgs::JointStateConstPtr& msg){
    if (msg->header.frame_id.empty()){
        std_msgs::String state;
        if(msg->position[5]>0.01 or msg->position[5]<2.49){
            state.data = "OBJETO ATRAPADO";
        }else{
            state.data = "PINZA LIBRE";
        }
        gripper_state.publish(state);
    }  
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_state");
    ROS_INFO("Getting gripper state...");
    ros::NodeHandle n;

    ros::Subscriber joint_state = n.subscribe("/joint_states", 1000, callbackJointState);
    gripper_state = n.advertise<std_msgs::String>("/gripper_state", 1000);

    ros::spin();
    return 0;
}