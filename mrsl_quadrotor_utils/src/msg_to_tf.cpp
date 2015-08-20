#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <mrsl_quadrotor_utils/general_util.h>

std::string horizon_frame_;
double shift_z_;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  static tf2_ros::TransformBroadcaster odom_br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header = msg->header;
  transformStamped.child_frame_id = msg->child_frame_id;
  transformStamped.transform.translation.x = msg->pose.pose.position.x;
  transformStamped.transform.translation.y = msg->pose.pose.position.y;
  transformStamped.transform.translation.z = msg->pose.pose.position.z;
  transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
  transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
  transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
  transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

  odom_br.sendTransform(transformStamped);
  ROS_WARN_ONCE("publish tf from [%s] to [%s]",
                transformStamped.header.frame_id.c_str(),
                transformStamped.child_frame_id.c_str());

  if(!horizon_frame_.empty()){
    Eigen::Vector3d ypr = getYPR(msg->pose.pose.orientation);    
    Eigen::AngleAxisd yawAngle(-M_PI / 4, Eigen::Vector3d::UnitZ());          
    Eigen::AngleAxisd pitchAngle(ypr(1), Eigen::Vector3d::UnitY());     
    Eigen::AngleAxisd rollAngle(ypr(2), Eigen::Vector3d::UnitX());      

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    geometry_msgs::Quaternion qq;                      
    tf::quaternionEigenToMsg(q.inverse(), qq); 

    static tf2_ros::TransformBroadcaster tf_pub;                             
    transformStamped.header.stamp = msg->header.stamp;
    transformStamped.header.frame_id = msg->child_frame_id;
    transformStamped.child_frame_id = horizon_frame_;
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = shift_z_;
    transformStamped.transform.rotation.x = qq.x;
    transformStamped.transform.rotation.y = qq.y;
    transformStamped.transform.rotation.z = qq.z;
    transformStamped.transform.rotation.w = qq.w;

    tf_pub.sendTransform(transformStamped);   
    ROS_WARN_ONCE("publish tf from [%s] to [%s]",
                  transformStamped.header.frame_id.c_str(),
                  transformStamped.child_frame_id.c_str());
 
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "msg_to_tf");
  ros::NodeHandle nh("~");
  nh.param("shift_z", shift_z_, 0.0);
  ros::Subscriber sub = nh.subscribe("odom", 10, odomCallback);
  nh.param("horizon_frame", horizon_frame_, std::string(""));

  ros::spin();
  return 0;
};



