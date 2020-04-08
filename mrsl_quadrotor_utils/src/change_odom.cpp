#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

// tf::Quaternion init_yaw_q_;
bool first_msg_ = true;
bool first_msg_time_set_ = false;
double dt_thr_;
ros::Publisher odom_pub;
ros::Time first_msg_time_;
nav_msgs::Odometry first_odom_;

nav_msgs::Odometry substractOdom(nav_msgs::Odometry odom1,
                                 const nav_msgs::Odometry &odom2) {
  // odom_out = odom1 -odom2
  tf::Pose world_pose, initial_pose;
  tf::poseMsgToTF(odom1.pose.pose, world_pose);
  tf::poseMsgToTF(odom2.pose.pose, initial_pose);
  tf::Transform local_tf = initial_pose.inverse() * world_pose;
  tf::poseTFToMsg(local_tf, odom1.pose.pose);
  double yaw = std::atan2(odom2.pose.pose.orientation.z, odom2.pose.pose.orientation.w) * 2;
  double vx_w = odom1.twist.twist.linear.x;
  double vy_w = odom1.twist.twist.linear.y;
  odom1.twist.twist.linear.x = cos(yaw) * vx_w + sin(yaw) * vy_w;
  odom1.twist.twist.linear.y = -sin(yaw) * vx_w + cos(yaw) * vy_w;
  return odom1;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  if(dt_thr_ > 0 && !first_msg_time_set_) {
    first_msg_time_set_ = true;
    first_msg_time_ = ros::Time::now();
  }

  if(first_msg_time_set_) {
    if((ros::Time::now() - first_msg_time_).toSec() < dt_thr_) {
      ROS_INFO_THROTTLE(1, "Skip this odom message, time left: %f",
       dt_thr_ - (ros::Time::now() - first_msg_time_).toSec());
      return;
    }
  }

  if (first_msg_) {
    first_msg_ = false;
    first_odom_ = *msg;
  }
  nav_msgs::Odometry odom_out = *msg;
  odom_out = substractOdom(odom_out, first_odom_);
  odom_pub.publish(odom_out);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "change_odom");
  ros::NodeHandle nh("~");

  // time after which the odom start publishing
  nh.param("dt_thr", dt_thr_, 0.0);
  ros::Subscriber odom_sub = nh.subscribe("odom_in", 10, &odomCallback,
                                          ros::TransportHints().tcpNoDelay());
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom_out", 10);
  ros::spin();
  return 0;
}
