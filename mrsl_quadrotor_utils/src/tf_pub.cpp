#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class TfPub
{
public:
  TfPub();
private:
  void OdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg);

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber odom_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::string odom_frame_id_, base_frame_id_;
};

TfPub::TfPub()
{
  pnh_ = ros::NodeHandle("~");
  pnh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
  pnh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
  odom_sub_ = pnh_.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&TfPub::OdometryCallback, this, _1));
}

void TfPub::OdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg) {

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = odometry_msg->header.stamp;
  odom_trans.header.frame_id = odom_frame_id_;
  odom_trans.child_frame_id = base_frame_id_;

  odom_trans.transform.translation.x = odometry_msg->pose.pose.position.x;
  odom_trans.transform.translation.y = odometry_msg->pose.pose.position.y;
  odom_trans.transform.translation.z = odometry_msg->pose.pose.position.z;
  odom_trans.transform.rotation = odometry_msg->pose.pose.orientation;

  //send the transform
  tf_broadcaster_.sendTransform(odom_trans);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_pub");
  TfPub tf_pub;
  ros::spin();
}