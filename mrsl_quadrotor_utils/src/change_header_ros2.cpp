#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

std::string world_frame;
std::string robot_frame;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto odom_out = *msg;
  odom_out.header.frame_id = world_frame;
  odom_out.child_frame_id = robot_frame;
  odom_pub->publish(odom_out);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("change_header");

  // Declare and get parameters
  node->declare_parameter<std::string>("world_frame", "map");
  node->declare_parameter<std::string>("robot_frame", "romeo");
  node->get_parameter("world_frame", world_frame);
  node->get_parameter("robot_frame", robot_frame);

  // Create publisher and subscriber
  odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom_out", 10);
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "odom_in", 10, odomCallback);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}