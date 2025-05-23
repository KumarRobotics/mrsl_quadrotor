#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

bool first_msg = true;
bool first_msg_time_set = false;
double dt_thr = 0.0;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
rclcpp::Time first_msg_time;
nav_msgs::msg::Odometry first_odom;
std::shared_ptr<rclcpp::Node> node;

nav_msgs::msg::Odometry subtractOdom(const nav_msgs::msg::Odometry &odom1,
                                     const nav_msgs::msg::Odometry &odom2) {
  tf2::Transform world_pose, initial_pose;
  tf2::fromMsg(odom1.pose.pose, world_pose);
  tf2::fromMsg(odom2.pose.pose, initial_pose);
  tf2::Transform local_tf = initial_pose.inverse() * world_pose;
  nav_msgs::msg::Odometry result = odom1;
  result.pose.pose.position.x = local_tf.getOrigin().x();
  result.pose.pose.position.y = local_tf.getOrigin().y();
  result.pose.pose.position.z = local_tf.getOrigin().z();
  result.pose.pose.orientation.x = local_tf.getRotation().x();
  result.pose.pose.orientation.y = local_tf.getRotation().y();
  result.pose.pose.orientation.z = local_tf.getRotation().z();
  result.pose.pose.orientation.w = local_tf.getRotation().w();

  // Compute yaw from initial pose
  tf2::Quaternion q;
  tf2::fromMsg(odom2.pose.pose.orientation, q);
  double yaw = tf2::getYaw(q);

  // Transform twist
  double vx_w = result.twist.twist.linear.x;
  double vy_w = result.twist.twist.linear.y;
  result.twist.twist.linear.x = std::cos(yaw) * vx_w + std::sin(yaw) * vy_w;
  result.twist.twist.linear.y = -std::sin(yaw) * vx_w + std::cos(yaw) * vy_w;

  return result;
}

void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  rclcpp::Time now = node->get_clock()->now();

  if (dt_thr > 0.0 && !first_msg_time_set) {
    first_msg_time_set = true;
    first_msg_time = now;
  }

  if (first_msg_time_set) {
    if ((now - first_msg_time).seconds() < dt_thr) {
      RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
        "Skipping odom (%.2fs left)", dt_thr - (now - first_msg_time).seconds());
      return;
    }
  }

  if (first_msg) {
    first_msg = false;
    first_odom = *msg;
  }

  auto odom_out = subtractOdom(*msg, first_odom);
  odom_pub->publish(odom_out);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("change_odom");

  node->declare_parameter("dt_thr", 0.0);
  node->get_parameter("dt_thr", dt_thr);

  odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom_out", 10);

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "odom_in", 10, odomCallback);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
