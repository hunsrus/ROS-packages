#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/tf.h"
#include <list>
#include <cmath>

void odomCallBack(const nav_msgs::Odometry::ConstPtr msg)
{
    double yaw = tf::getYaw(msg->pose.pose.orientation);
    yaw = (yaw/M_PI)*180;
    //std::cout << "yaw:    " << yaw << std::endl;
    //std::cout << "x:      " << msg->pose.pose.position.x << std::endl;
    //std::cout << "y:      " << msg->pose.pose.position.y << std::endl;
    
}

int main(int argc, char **argv)
{
    int i;
    std::list<geometry_msgs::Point> camino;
    geometry_msgs::Twist twist;

    ros::init(argc, argv, "pathfinding_test");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber sub = nh.subscribe("odom", 10, &odomCallBack);
    ros::Rate loop_rate(50);

    double roll, pitch, yaw;
    
int c=0;
char *in;
    while (ros::ok())
    {
        /*twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;


        pub.publish(twist);*/
        //std::cin >> in;
        std::cout << &c << " input " << c << std::endl;
        c++;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

