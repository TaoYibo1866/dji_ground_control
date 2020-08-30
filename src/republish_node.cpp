#include <ros/forwards.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

ros::Publisher loc_pos_pub;

void locPosCb(const geometry_msgs::PointStampedConstPtr& msg, int a)
{
    loc_pos_pub.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "republish");
    ros::NodeHandle nh;
    loc_pos_pub = nh.advertise<geometry_msgs::PointStamped>("/gc/local_position", 1);
    ros::Subscriber loc_pos_sub = nh.subscribe<geometry_msgs::PointStamped>("/dji_sdk/local_position", 10, boost::bind(&locPosCb, _1, 100),
                                                                                                            ros::VoidConstPtr(),
                                                                                                            ros::TransportHints().tcpNoDelay());
    return 0;
}
