#include <ros/ros.h>                                                  
#include <gazebo_msgs/LinkState.h>
#include <eigen_conversions/eigen_msg.h>   
#include <tf_conversions/tf_eigen.h>   
#include <angles/angles.h>
#include <tf2_ros/transform_broadcaster.h>
#include <mrsl_quadrotor_utils/general_util.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_rotate");
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<gazebo_msgs::LinkState>("gazebo/set_link_state", 10);

  std::string link_frame_base, ref_frame_base, robot;
  double upper, lower, w_abs;
  nh.param("link_frame", link_frame_base, std::string("laser_frame"));
  nh.param("ref_frame", ref_frame_base, std::string("base_link"));
  nh.param("robot", robot, std::string("juliett"));
  nh.param("upper", upper, M_PI / 4);
  nh.param("lower", lower, -M_PI / 4);
  nh.param("w", w_abs, M_PI / 2);

  std::string link_name = robot + "::" + link_frame_base;
  std::string ref_name = robot + "::" + ref_frame_base;

  std::string link_frame = robot + "/" + link_frame_base;
  std::string ref_frame = robot + "/" + ref_frame_base;

  double angle = 0;
  double w = w_abs;

  int r = 100;

  static tf2_ros::TransformBroadcaster tf_pub;                             
  ros::Time t0 = ros::Time::now();
  ros::Rate loop_rate(r);
  while(ros::ok())
  {
    ros::Time t1 = ros::Time::now();
    double dt = (t1 - t0).toSec();
    t0 = t1;
    gazebo_msgs::LinkState msg;

    angle += w * dt;

    angle = angles::normalize_angle(angle);

    Eigen::Affine3d sensor_to_body = Eigen::Translation3d(0.1, 0.1, 0.0) 
      *Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ())
      *Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY())              
      *Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());        

    tf::Pose tf_pose;                                                   
    tf::poseEigenToTF(sensor_to_body, tf_pose);
    geometry_msgs::Pose pose;
    tf::poseTFToMsg(tf_pose, pose);

    if(angle >= upper)
      w = -w_abs;
    else if(angle <= lower)
      w = w_abs;


    msg.pose = pose;
    msg.twist.angular.y = w;
    msg.reference_frame = ref_name;
    msg.link_name = link_name;

    pub.publish(msg);


    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = ref_frame;
    transformStamped.child_frame_id = link_frame;
    transformStamped.transform.translation.x = msg.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.position.z;

    transformStamped.transform.rotation.w = msg.pose.orientation.w;
    transformStamped.transform.rotation.x = msg.pose.orientation.x;
    transformStamped.transform.rotation.y = msg.pose.orientation.y;
    transformStamped.transform.rotation.z = msg.pose.orientation.z;

    tf_pub.sendTransform(transformStamped);   
    ROS_WARN_ONCE("publish tf from [%s] to [%s]",
                  transformStamped.header.frame_id.c_str(),
                  transformStamped.child_frame_id.c_str());

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
};


