#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <topic_tools/shape_shifter.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

std::string child_frame_id;

void process(const geometry_msgs::Pose &pose, const std_msgs::Header& header, std::string child_frame_id) {
  static tf2_ros::TransformBroadcaster odom_br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header = header;
  transformStamped.child_frame_id = child_frame_id;
  transformStamped.transform.translation.x = pose.position.x;
  transformStamped.transform.translation.y = pose.position.y;
  transformStamped.transform.translation.z = pose.position.z;
  transformStamped.transform.rotation.x = pose.orientation.x;
  transformStamped.transform.rotation.y = pose.orientation.y;
  transformStamped.transform.rotation.z = pose.orientation.z;
  transformStamped.transform.rotation.w = pose.orientation.w;

  odom_br.sendTransform(transformStamped);
  ROS_WARN_ONCE("publish tf from [%s] to [%s]",
                transformStamped.header.frame_id.c_str(),
                transformStamped.child_frame_id.c_str());
}

void msgCallback(const topic_tools::ShapeShifter::ConstPtr &msg) {
  if(msg->getDataType() == "nav_msgs/Odometry") {
    auto odom = msg->instantiate<nav_msgs::Odometry>();
    process(odom->pose.pose, odom->header, odom->child_frame_id);
  }
  else if(msg->getDataType() == "geometry_msgs/PoseStamped") {
    auto pose = msg->instantiate<geometry_msgs::PoseStamped>();
    process(pose->pose, pose->header, child_frame_id);
  }
  else 
    ROS_WARN_ONCE("Unrecognized msg type! [%s]", msg->getDataType().c_str());
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "msg_to_tf");
  ros::NodeHandle nh("~");

  nh.param("child_frame_id", child_frame_id, std::string("null_frame"));

  ros::Subscriber sub = nh.subscribe("msg", 10, msgCallback,
                                     ros::TransportHints().tcpNoDelay());

  ros::spin();
  return 0;
};
