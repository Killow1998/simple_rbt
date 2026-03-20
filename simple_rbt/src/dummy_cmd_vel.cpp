#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_cmd_vel_publisher");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  std::string topic_name = "cmd_vel";
  pnh.param<std::string>("topic", topic_name, topic_name);

  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>(topic_name, 10);

  ros::Rate loop_rate(10);
  geometry_msgs::Twist msg;
  while (ros::ok())
  {
    cmd_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
