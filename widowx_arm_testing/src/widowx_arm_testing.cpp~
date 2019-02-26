#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

/**
 * Basic homing for the WidowX arm. Joints 1 to 5 are set to 0 degrees.
 * Ana Cruz-Martín, 2015
 *
 * DISCLAIMER: This is a work-in-progress test code, not finished yet.
 *
 * This code must be integrated into a ros package, and then called with rosrun,
 * while the widowx arm software by Robotnik is running.
 * https://github.com/RobotnikAutomation/widowx_arm
 *
 * For videos and further information
 * http://jafma.net/ana/theweekendarchaeologist/?post=564
 *
 * TO DO:
 * - Clean and organize code
 * - Use services for relaxing joints
 * - Open/close gripper
 * - Create a proper launch file
 * - rviz integration
 */


void jointsCallback(const sensor_msgs::JointState::ConstPtr& jointsmsg)
{
  ROS_INFO("JOINTS POSITIONS[1: %f] [2: %f] [3: %f] [4: %f] [5: %f] [6: %f]\n", jointsmsg->position[1], jointsmsg->position[2], jointsmsg->position[3], jointsmsg->position[4], jointsmsg->position[5], jointsmsg->position[6]);
  ROS_INFO("\nJOINTS VELOCITIES[1: %f] [2: %f] [3: %f] [4: %f] [5: %f] [6: %f]\n", jointsmsg->velocity[1], jointsmsg->velocity[2], jointsmsg->velocity[3], jointsmsg->velocity[4], jointsmsg->velocity[5], jointsmsg->velocity[6]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv , "widowx_arm_home");
  ros::NodeHandle n;

  // Topics of the joints.
  ros::Publisher joint1 = n.advertise<std_msgs::Float64>("/arm_1_joint/command", 1000);
  ros::Publisher joint2 = n.advertise<std_msgs::Float64>("/arm_2_joint/command", 1000);
  ros::Publisher joint3 = n.advertise<std_msgs::Float64>("/arm_3_joint/command", 1000);
  ros::Publisher joint4 = n.advertise<std_msgs::Float64>("/arm_4_joint/command", 1000);
  ros::Publisher joint5 = n.advertise<std_msgs::Float64>("/arm_5_joint/command", 1000);
  ros::Rate loop_rate(10);

  ros::Subscriber joints = n.subscribe("/joint_states", 1000, jointsCallback);

  // Relax services of the joints
  //ros::ServiceClient client_service_joint2 = n.serviceClient<arbotix_ros::arbotix_msgs::Relax>("Relax);
  //arbotix_ros::arbotix_msgs::Relax service_joint2;
  //service_joint2.request = ();

  /**
   * Main code.
   */

  std_msgs::Float64 degrees;
  degrees.data = 0;
  int count = 0;
  while (ros::ok() && count<=10)
  {
    if (count == 0)
    {
       ROS_INFO("3 SECONDS TO HOME\n");
       ros::Duration(3).sleep();
       joint2.publish(degrees);
       joint3.publish(degrees);
       joint4.publish(degrees);
       joint5.publish(degrees);
       joint1.publish(degrees);
    }
    ROS_INFO("Count: %d\n",count);
    count++;
 
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
