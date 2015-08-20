#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <boost/foreach.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "play_tf_static");
  ros::NodeHandle nh, pnh("~");
  tf2_ros::StaticTransformBroadcaster broad_static;
  if (argc != 2) {
    ROS_ERROR("Usage: play_tf_static bagfile");
    return 1;
  }
  std::string bagfile = argv[1];

  ROS_INFO("Starting to read %s", bagfile.c_str());
  rosbag::Bag bag;
  bag.open(bagfile, rosbag::bagmode::Read);
  
  rosbag::View tf_view(bag, rosbag::TopicQuery("/tf_static"));
  std::vector<geometry_msgs::TransformStamped> transforms;
  BOOST_FOREACH(rosbag::MessageInstance const m, tf_view) {
    tf2_msgs::TFMessage::Ptr msg = m.instantiate<tf2_msgs::TFMessage>();
    for (int i = 0; i < msg->transforms.size(); ++i) {
      transforms.push_back(msg->transforms.at(i));
    }
  }
  ROS_INFO("Sending %zu transforms", transforms.size());
  broad_static.sendTransform(transforms);

  ros::spin();
  return 0;
}
