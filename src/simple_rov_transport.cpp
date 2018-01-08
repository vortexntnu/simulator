#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Wrench.h"

void callback(const geometry_msgs::Wrench &msg, geometry_msgs::Wrench*
wrench_arg)
{
    float tx = msg.torque.y;
    ROS_INFO("Torque: [%f]", tx);
    pub.publish(*msg);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_client");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("rov_forces", 1, callback);
    ros::Publisher pub = n.advertise<geometry_msgs::Wrench>("rov_forces", 1);
    ros::Rate loop_rate(10);


    geometry_msgs::Wrench wrench;
    while(ros::ok())
    {
        ROS_INFO("Publishing Wrench msg.")
        pub.publish(msg)
    }
    return 0;
}

