#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>

#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

std::string robot_frame;
std::string world_frame;

ros::Publisher cloud_pub;

tf2_ros::Buffer tfBuffer;

sensor_msgs::PointCloud transform(const sensor_msgs::PointCloud& cloud, const Eigen::Affine3d& TF)
{
  sensor_msgs::PointCloud cloud_out = cloud;
  for(size_t i = 0; i < cloud.points.size(); i++)
  {
    Eigen::Vector3d raw(cloud.points[i].x,
                        cloud.points[i].y,
                        cloud.points[i].z);
    raw = TF * raw;
    cloud_out.points[i].x = raw(0);
    cloud_out.points[i].y = raw(1);
    cloud_out.points[i].z = raw(2);
  }
  return cloud_out;
}

bool getTF(const ros::Time &t, Eigen::Affine3d& TF)
{                     
  geometry_msgs::TransformStamped transformStamped;
  try{                
    transformStamped = tfBuffer.lookupTransform(
      world_frame, robot_frame,
      t, ros::Duration(0.1));
  }                   
  catch (tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(5, "%s",ex.what());
    return false;     
  }                   

  geometry_msgs::Pose pose;
  pose.position.x = transformStamped.transform.translation.x;
  pose.position.y = transformStamped.transform.translation.y;
  pose.position.z = transformStamped.transform.translation.z;
  pose.orientation.w = transformStamped.transform.rotation.w;
  pose.orientation.x = transformStamped.transform.rotation.x;
  pose.orientation.y = transformStamped.transform.rotation.y;
  pose.orientation.z = transformStamped.transform.rotation.z;

  tf::poseMsgToEigen (pose, TF);
  return true;        
}                     


void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  Eigen::Affine3d TF;
  robot_frame = msg->header.frame_id;
  if(getTF(msg->header.stamp, TF))
  {
    sensor_msgs::PointCloud cloud_out = transform(*msg, TF);
    cloud_out.header.frame_id = world_frame;
    cloud_pub.publish(cloud_out);
  }
  else
    ROS_WARN_THROTTLE(2, "Fail to get TF from %s to %s",
                      world_frame.c_str(), robot_frame.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "change_header");
  ros::NodeHandle nh("~");
  static tf2_ros::TransformListener tfListener(tfBuffer);
  nh.param("world_frame", world_frame, std::string("map"));
  ros::Subscriber sub = nh.subscribe("cloud_in", 10, &cloudCallback);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud_out", 10);

  ros::spin();
  return 0;
}


