#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include <iostream>

#define ADEL 0
#define IZQ 1
#define DER 2

using namespace std;

class Scanner
{
    private:
        double check_dist = 0.6;
        double info_scan[3] = {0.0,0.0,0.0};
        geometry_msgs::Vector3 obst_dir;
        ros::NodeHandle nh;
        ros::Publisher info_obst_pub;
        ros::Subscriber laser_scan_sub;
    public:
        Scanner();
        void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
        void detectarObstaculos();
        void publicar();
};

Scanner::Scanner()
{
    info_obst_pub = nh.advertise<geometry_msgs::Vector3>("obst_pos", 1);
    laser_scan_sub = nh.subscribe("scan", 10, &Scanner::laserScanMsgCallBack, this);
}

void Scanner::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    int i, angulo[3] = {0,30,330};                      //0° adelante; 30° izquierda; 330° derecha
    for(i = 0; i < 3; i++)
        if (std::isinf(msg->ranges.at(angulo[i])))
            info_scan[i] = msg->range_max;              //Si no hay obstaculos (distancia infinita) toma el rango máximo
        else
            info_scan[i] = msg->ranges.at(angulo[i]);   //Toma la información escaneada a en la dirección correspondiente
}

void Scanner::detectarObstaculos()
{
    if(info_scan[ADEL] <= check_dist) obst_dir.x = 1;
    else obst_dir.x = 0;

    if(info_scan[IZQ] <= check_dist) obst_dir.y = 1;
    else obst_dir.y = 0;

    if(info_scan[DER] <= check_dist) obst_dir.z = 1;
    else obst_dir.z = 0;
}

void Scanner::publicar()
{
    info_obst_pub.publish(obst_dir);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_test");
    Scanner scanner;

    ros::Rate loop_rate(125);

    while (ros::ok())
    {
   
        scanner.detectarObstaculos();
        scanner.publicar();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}