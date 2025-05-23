#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

/* -------------------------------------------------------------------- */
/*                       global state / singletons                      */
/* -------------------------------------------------------------------- */
std::string                                              child_frame_id;
std::string                                              frame_id;
std::shared_ptr<rclcpp::Node>                            node;
std::shared_ptr<tf2_ros::TransformBroadcaster>           tf_bcaster;

/* -------------------------------------------------------------------- */
/*                         helper that does the work                    */
/* -------------------------------------------------------------------- */
void publish_tf(const geometry_msgs::msg::Pose   &pose,
                const std_msgs::msg::Header      &header,
                const std::string                &child_frame)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header            = header;
  ts.header.frame_id   = frame_id;
  ts.child_frame_id    = child_frame;
  ts.transform.translation.x = pose.position.x;
  ts.transform.translation.y = pose.position.y;
  ts.transform.translation.z = pose.position.z;
  ts.transform.rotation.x = pose.orientation.x;
  ts.transform.rotation.y = pose.orientation.y;
  ts.transform.rotation.z = pose.orientation.z;
  ts.transform.rotation.w = pose.orientation.w;

  tf_bcaster->sendTransform(ts);

  RCLCPP_WARN_ONCE(node->get_logger(),
                   "publish tf from [%s] to [%s]",
                   ts.header.frame_id.c_str(),
                   ts.child_frame_id.c_str());
}

/* -------------------------------------------------------------------- */
/*                              callbacks                               */
/* -------------------------------------------------------------------- */
void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  publish_tf(msg->pose.pose, msg->header, child_frame_id);
}

// void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
// {
//   publish_tf(msg->pose, msg->header, child_frame_id);
// }

/* -------------------------------------------------------------------- */
/*                                  main                                */
/* -------------------------------------------------------------------- */
int main(int argc, char ** argv)
{
  /* initialise ROS 2 */
  rclcpp::init(argc, argv);

  /* create the node _after_ init() so the context is valid */
  node = rclcpp::Node::make_shared("msg_to_tf");

  /* declare / read parameter */
  child_frame_id =
      node->declare_parameter<std::string>("child_frame_id", "null_frame");
  RCLCPP_INFO(node->get_logger(), "child_frame_id = %s", child_frame_id.c_str());
  frame_id =
      node->declare_parameter<std::string>("frame_id", "null_frame");
  RCLCPP_INFO(node->get_logger(), "frame_id = %s", frame_id.c_str());

  /* TF broadcaster */
  tf_bcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  /* two subscriptions to the same topic, different concrete types */
  auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "msg", qos, &odom_cb);

  // auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
  //     "msg", qos, &pose_cb);

  /* spin */
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
