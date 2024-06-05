#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "arm_controller_pub");
    ros::NodeHandle nh;

    ros::Publisher joint0_pub = nh.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J0_controller/command", 1);
    ros::Publisher joint1_pub = nh.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J1_controller/command", 1);
    ros::Publisher joint2_pub = nh.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J2_controller/command", 1);
    ros::Publisher joint3_pub = nh.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J3_controller/command", 1);

    ros::Rate loop_rate(10);


  while (ros::ok())
  {

    std_msgs::Float64 joint0_command;
    joint0_command.data = 0;
    joint0_pub.publish(joint0_command);

    std_msgs::Float64 joint1_command;
    joint1_command.data = 0;
    joint1_pub.publish(joint1_command);

    std_msgs::Float64 joint2_command;
    joint2_command.data = 0;
    joint2_pub.publish(joint2_command);

    std_msgs::Float64 joint3_command;
    joint3_command.data = 0;
    joint3_pub.publish(joint3_command);

    ros::spinOnce();
    loop_rate.sleep();
  }

    return 0;
}