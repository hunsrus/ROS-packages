#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "teleop_twist_keyboard");
    ros::NodeHandle nh;

    // Init cmd_vel publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Create Twist message
    geometry_msgs::Twist twist;

    while (ros::ok())
    {
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 2;

        // Publish it and resolve any remaining callbacks
        pub.publish(twist);
        ros::spinOnce();
    }
    return 0;
}
