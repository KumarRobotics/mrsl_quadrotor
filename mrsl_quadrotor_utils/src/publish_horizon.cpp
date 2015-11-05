#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <mrsl_quadrotor_utils/general_util.h>

std::string horizon_frame_;
double shift_yaw_;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  static tf2_ros::TransformBroadcaster tf_pub;                             
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = msg->header.stamp;
  transformStamped.header.frame_id = msg->header.frame_id;
  transformStamped.child_frame_id = horizon_frame_;
  transformStamped.transform.translation.x = msg->pose.pose.position.x;
  transformStamped.transform.translation.y = msg->pose.pose.position.y;
  transformStamped.transform.translation.z = msg->pose.pose.position.z;

  double yaw = tf::getYaw(msg->pose.pose.orientation) + shift_yaw_;

  transformStamped.transform.rotation.w = cos(yaw / 2);
  transformStamped.transform.rotation.x = 0.0;
  transformStamped.transform.rotation.y = 0.0;
  transformStamped.transform.rotation.z = sin(yaw / 2);

  tf_pub.sendTransform(transformStamped);   
  ROS_WARN_ONCE("publish tf from [%s] to [%s]",
                transformStamped.header.frame_id.c_str(),
                transformStamped.child_frame_id.c_str());

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_horizon");
  ros::NodeHandle nh("~");
  nh.param("horizon_frame", horizon_frame_, std::string(""));
  nh.param("shift_yaw", shift_yaw_, 0.0);
  ros::Subscriber sub = nh.subscribe("odom", 10, odomCallback);

  ros::spin();
  return 0;
};



