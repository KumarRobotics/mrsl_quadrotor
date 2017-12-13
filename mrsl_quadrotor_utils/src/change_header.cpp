#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

std::string world_frame;
std::string robot_frame;

ros::Publisher odom_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  nav_msgs::Odometry odom_out = *msg;
  odom_out.header.frame_id = world_frame;
  odom_out.child_frame_id = robot_frame;
  odom_pub.publish(odom_out);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "change_header");
  ros::NodeHandle nh("~");
  nh.param("world_frame", world_frame, std::string("map"));
  nh.param("robot_frame", robot_frame, std::string("romeo"));
  ros::Subscriber odom_sub = nh.subscribe("odom_in", 10, &odomCallback,
                                          ros::TransportHints().tcpNoDelay());
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom_out", 10);
  ros::spin();
  return 0;
}
