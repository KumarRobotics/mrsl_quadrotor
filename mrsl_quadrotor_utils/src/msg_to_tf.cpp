#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  static tf2_ros::TransformBroadcaster odom_br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header = msg->header;
  transformStamped.child_frame_id = msg->child_frame_id;
  transformStamped.transform.translation.x = msg->pose.pose.position.x;
  transformStamped.transform.translation.y = msg->pose.pose.position.y;
  transformStamped.transform.translation.z = msg->pose.pose.position.z;
  transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
  transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
  transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
  transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

  odom_br.sendTransform(transformStamped);
  ROS_WARN_ONCE("publish tf from [%s] to [%s]",
                transformStamped.header.frame_id.c_str(),
                transformStamped.child_frame_id.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "msg_to_tf");
  ros::NodeHandle nh("~");
  ros::Subscriber sub = nh.subscribe("odom", 10, odomCallback,
                                     ros::TransportHints().tcpNoDelay());

  ros::spin();
  return 0;
};
