#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

tf::Quaternion init_yaw_q_;
static bool first_msg_ = true;
ros::Publisher odom_pub;
nav_msgs::Odometry first_odom_;

nav_msgs::Odometry substractOdom(nav_msgs::Odometry odom1, const nav_msgs::Odometry& odom2)
{
  static tf::Pose init_yaw_tf(init_yaw_q_);
  //odom_out = odom1 -odom2
  tf::Pose world_pose, initial_pose;
  tf::poseMsgToTF(odom1.pose.pose, world_pose);
  tf::poseMsgToTF(odom2.pose.pose, initial_pose);
  tf::Transform local_tf = init_yaw_tf * initial_pose.inverse() * world_pose;
  tf::poseTFToMsg(local_tf, odom1.pose.pose);
  return odom1;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(first_msg_)
  {
    first_msg_ = false;
    first_odom_ = *msg;
  }
  nav_msgs::Odometry odom_out = *msg;
  odom_out = substractOdom(odom_out, first_odom_);
  odom_pub.publish(odom_out);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "change_odom");
  ros::NodeHandle nh("~");
  double yaw;
  nh.param("init_yaw", yaw, 0.0);
  
  init_yaw_q_.setEuler(0.0, 0.0, yaw);

  ros::Subscriber odom_sub = nh.subscribe("odom_in", 10, &odomCallback);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom_out", 10);
  ros::spin();
  return 0;
}
