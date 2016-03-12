#include <ros/ros.h>                                                  
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/LinkStates.h>
#include <eigen_conversions/eigen_msg.h>   
#include <tf_conversions/tf_eigen.h>   
#include <angles/angles.h>
#include <tf2_ros/transform_broadcaster.h>
#include <mrsl_quadrotor_utils/general_util.h>

double upper, lower, w_abs, w;
std::string link_frame_base, ref_frame_base, robot;
std::string link_name, ref_name;
std::string link_frame, ref_frame;
bool need_publish = true;
ros::Publisher pub;
ros::Time ref_t;

void linkCallback(const gazebo_msgs::LinkStates::ConstPtr& state){
  static tf2_ros::TransformBroadcaster tf_pub;                             
  geometry_msgs::Pose ref_pose, link_pose;
  geometry_msgs::Twist ref_twist;
  bool has_ref = false;
  bool has_link = false;
  for(unsigned int i = 0; i < state->name.size(); i++){
    if(state->name[i] == ref_name){
      has_ref = true;
      ref_pose = state->pose[i];
      ref_twist = state->twist[i];
    }
    else if(state->name[i] == link_name){
      has_link = true;
      link_pose = state->pose[i];
    }
  }

  if(!has_ref || !has_link)
    return;

  
  tf::Pose ref_tf_pose, link_tf_pose;
  tf::poseMsgToTF(ref_pose, ref_tf_pose);
  tf::poseMsgToTF(link_pose, link_tf_pose);

  tf::Pose ref_to_link = ref_tf_pose.inverse() * link_tf_pose;
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(ref_to_link, pose);

  double roll, pitch, yaw;
  tf::Matrix3x3(ref_to_link.getRotation()).getRPY(roll, pitch, yaw);
  //ROS_INFO_THROTTLE(0.1, "RPY: [%f, %f, %f]", roll, pitch, yaw);

  
  double dt = (ros::Time::now() - ref_t).toSec();
  if(pitch >= upper && dt > 0.1)
  {
    need_publish = true;
    w = -w_abs;
  }
  else if(pitch <= lower && dt > 0.1)
  {
    need_publish = true;
    w = w_abs;
  }

  tf::Vector3 des_w(0, w, 0);
  des_w = tf::Matrix3x3(link_tf_pose.getRotation()) * des_w;

  gazebo_msgs::LinkState msg;

  msg.pose = pose;
  msg.twist.angular.x = des_w.x() + ref_twist.angular.x;
  msg.twist.angular.y = des_w.y() + ref_twist.angular.y;
  msg.twist.angular.z = des_w.z() + ref_twist.angular.z;
  msg.reference_frame = ref_name;
  msg.link_name = link_name;

  if(need_publish)
  {
    ref_t = ros::Time::now();
    need_publish = false;
    pub.publish(msg);
  }


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
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_rotate");
  ros::NodeHandle nh("~");
  ros::Subscriber sub = nh.subscribe("gazebo/link_states", 10, linkCallback);
  pub = nh.advertise<gazebo_msgs::LinkState>("gazebo/set_link_state", 10);

  nh.param("link_frame", link_frame_base, std::string("laser_frame"));
  nh.param("ref_frame", ref_frame_base, std::string("base_link"));
  nh.param("robot", robot, std::string("juliett"));
  nh.param("upper", upper, M_PI / 4);
  nh.param("lower", lower, -M_PI / 4);
  nh.param("w", w_abs, M_PI / 2);

  link_name = robot + "::" + link_frame_base;
  ref_name = robot + "::" + ref_frame_base;

  link_frame = robot + "/" + link_frame_base;
  ref_frame = robot + "/" + ref_frame_base;


  w = w_abs;
  ref_t = ros::Time::now();
  ros::spin();

  return 0;
};


