#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Joy.h>

template <class Type>
void transportCb(const Type& msg, ros::Publisher pub)
{
    pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "republish_node");
    ros::NodeHandle n;

    ros::Publisher loc_pos_pub = n.advertise<geometry_msgs::PointStamped>("/gc/local_position", 1);
    ros::Publisher att_pub = n.advertise<geometry_msgs::QuaternionStamped>("/gc/attitude", 1);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Vector3Stamped>("/gc/velocity", 1);
    ros::Publisher height_pub = n.advertise<std_msgs::Float32>("/gc/height_above_takeoff", 1);
    ros::Publisher acc_pub = n.advertise<geometry_msgs::Vector3Stamped>("/gc/acceleration_ground_fused", 1);
    ros::Publisher gps_pos_pub = n.advertise<sensor_msgs::NavSatFix>("/gc/gps_position", 1);
    ros::Publisher ang_vel_pub = n.advertise<geometry_msgs::Vector3Stamped>("/gc/angular_velocity_fused", 1);
    ros::Publisher gps_health_pub = n.advertise<std_msgs::UInt8>("/gc/gps_health", 1);
    ros::Publisher battery_state_pub = n.advertise<sensor_msgs::BatteryState>("/gc/battery_state", 1);
    ros::Publisher flight_status_pub = n.advertise<std_msgs::UInt8>("/gc/flight_status", 1);
    ros::Publisher rc_pub = n.advertise<sensor_msgs::Joy>("/gc/rc", 1);
    ros::Publisher fscg_pub = n.advertise<sensor_msgs::Joy>("/gc/flight_control_setpoint_generic", 1);
    
    ros::Subscriber loc_pos_sub = n.subscribe<geometry_msgs::PointStamped>("/dji_sdk/local_position", 1,
                                            boost::bind(transportCb<geometry_msgs::PointStamped::ConstPtr>, _1, loc_pos_pub), 
                                            ros::VoidConstPtr(),
                                            ros::TransportHints().unreliable().maxDatagramSize(1000));
    ros::Subscriber att_sub = n.subscribe<geometry_msgs::QuaternionStamped>("/dji_sdk/attitude", 1,
                                            boost::bind(transportCb<geometry_msgs::QuaternionStamped::ConstPtr>, _1, att_pub), 
                                            ros::VoidConstPtr(),
                                            ros::TransportHints().unreliable().maxDatagramSize(1000));
    ros::Subscriber vel_sub = n.subscribe<geometry_msgs::Vector3Stamped>("/dji_sdk/velocity", 1,
                                            boost::bind(transportCb<geometry_msgs::Vector3Stamped::ConstPtr>, _1, vel_pub), 
                                            ros::VoidConstPtr(),
                                            ros::TransportHints().unreliable().maxDatagramSize(1000)); 
    ros::Subscriber height_sub = n.subscribe<std_msgs::Float32>("/dji_sdk/height_above_takeoff", 1,
                                            boost::bind(transportCb<std_msgs::Float32::ConstPtr>, _1, height_pub), 
                                            ros::VoidConstPtr(),
                                            ros::TransportHints().unreliable().maxDatagramSize(1000));
    ros::Subscriber acc_sub = n.subscribe<geometry_msgs::Vector3Stamped>("/dji_sdk/acceleration_ground_fused", 1, 
                                            boost::bind(transportCb<geometry_msgs::Vector3Stamped::ConstPtr>, _1, acc_pub), 
                                            ros::VoidConstPtr(),
                                            ros::TransportHints().unreliable().maxDatagramSize(1000));
    ros::Subscriber gps_pos_sub = n.subscribe<sensor_msgs::NavSatFix>("/dji_sdk/gps_position", 1,
                                            boost::bind(transportCb<sensor_msgs::NavSatFix::ConstPtr>, _1, gps_pos_pub), 
                                            ros::VoidConstPtr(),
                                            ros::TransportHints().unreliable().maxDatagramSize(1000));
    ros::Subscriber ang_vel_sub = n.subscribe<geometry_msgs::Vector3Stamped>("/dji_sdk/angular_velocity_fused", 1,
                                            boost::bind(transportCb<geometry_msgs::Vector3Stamped::ConstPtr>, _1, ang_vel_pub), 
                                            ros::VoidConstPtr(),
                                            ros::TransportHints().unreliable().maxDatagramSize(1000));
    ros::Subscriber gps_health_sub = n.subscribe<std_msgs::UInt8>("/dji_sdk/gps_health", 1,
                                            boost::bind(transportCb<std_msgs::UInt8::ConstPtr>, _1, gps_health_pub), 
                                            ros::VoidConstPtr(),
                                            ros::TransportHints().unreliable().maxDatagramSize(1000));
    ros::Subscriber battery_state_sub = n.subscribe<sensor_msgs::BatteryState>("/dji_sdk/battery_state", 1,
                                            boost::bind(transportCb<sensor_msgs::BatteryState::ConstPtr>, _1, battery_state_pub), 
                                            ros::VoidConstPtr(),
                                            ros::TransportHints().unreliable().maxDatagramSize(1000));
    ros::Subscriber flight_status_sub = n.subscribe<std_msgs::UInt8>("/dji_sdk/flight_status", 1,
                                            boost::bind(transportCb<std_msgs::UInt8::ConstPtr>, _1, flight_status_pub), 
                                            ros::VoidConstPtr(),
                                            ros::TransportHints().unreliable().maxDatagramSize(1000));  
    ros::Subscriber rc_sub = n.subscribe<sensor_msgs::Joy>("/dji_sdk/rc", 1,
                                            boost::bind(transportCb<sensor_msgs::Joy::ConstPtr>, _1, rc_pub), 
                                            ros::VoidConstPtr(),
                                            ros::TransportHints().unreliable().maxDatagramSize(1000));
    ros::Subscriber fscg_sub = n.subscribe<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 1,
                                            boost::bind(transportCb<sensor_msgs::Joy::ConstPtr>, _1, fscg_pub), 
                                            ros::VoidConstPtr(),
                                            ros::TransportHints().unreliable().maxDatagramSize(1000));
    ros::spin();
    return 0;
}