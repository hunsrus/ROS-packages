#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <cstdlib>

#include "joystick.h"

int main(int argc, char **argv)
{
    joystick control;
    int js;
    struct js_event event;
    struct axis_state axes[3] = {0};
    size_t axis;

    if (argc > 1)
        control.setDevice(argv[1]);
    else
        control.setDevice("/dev/input/js0");

    js = open(control.getDevice(), O_RDONLY);

    if (js == -1)
        perror("No se pudo abrir el joystick");

    ros::init(argc, argv, "teleop_joystick");
    
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    geometry_msgs::Twist twist;

    while ((control.read_event(js, &event) == 0) && (ros::ok()))
    {
        system("clear");

        switch (event.type)
        {
            case JS_EVENT_BUTTON:
                if((event.number == 5)&&(event.value)) twist.linear.x = 0.22;
                else if((event.number == 4)&&(event.value)) twist.linear.x = -0.22;
                else twist.linear.x = 0;
                break;
            case JS_EVENT_AXIS:
                axis = control.get_axis_state(&event, axes);
                if (axis == 0) twist.angular.z = -axes[0].x/10000.0f;
                break;
            default:
                break;
        }
        
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;

        pub.publish(twist);
        ros::spinOnce();

        std::cout << "Lineal" << std::endl
            << "x: " << twist.linear.x << std::endl
            << "y: " << twist.linear.y << std::endl
            << "z: " << twist.linear.z << std::endl
            << "Angular" << std::endl 
            << "x: " << twist.angular.x << std::endl
            << "y: " << twist.angular.y << std::endl
            << "z: " << twist.angular.z << std::endl;
    }

    close(js);
    return 0;
}
