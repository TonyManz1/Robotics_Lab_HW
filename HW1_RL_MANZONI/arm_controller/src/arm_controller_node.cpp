#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    ROS_INFO("\nReceived joint positions:");
    for (size_t i = 0; i < msg->position.size(); i++) {
        ROS_INFO("Joint %zu: %f", i, msg->position[i]);
    }
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "arm_controller_node");
    ros::NodeHandle nh;

    // Subscribe to /joint_states with a buffer of size 10
    ros::Subscriber joint_state_sub = nh.subscribe("/arm/joint_states", 10, jointStateCallback);

    ros::spin(); 

    return 0;
}