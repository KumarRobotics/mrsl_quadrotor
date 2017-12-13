#include <ros/ros.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

void publishTF() {
  Eigen::Affine3d sensor_to_body =
      Eigen::Translation3d(0, 0, 0) *
      Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());

  tf::Pose tf_pose;
  tf::poseEigenToTF(sensor_to_body, tf_pose);
  geometry_msgs::Pose pose_msg;
  tf::poseTFToMsg(tf_pose, pose_msg);

  printf("tf [%f, %f, %f], [%f, %f, %f, %f]\n\n", pose_msg.position.x,
         pose_msg.position.y, pose_msg.position.z, pose_msg.orientation.x,
         pose_msg.orientation.y, pose_msg.orientation.z,
         pose_msg.orientation.w);

  Eigen::Matrix3d m;
  m << 1 / sqrt(2), 0, 1 / sqrt(2), -1 / sqrt(2), 0, 1 / sqrt(2), 0, -1, 0;

  Eigen::Quaterniond q(m);
  printf("[%f, %f, %f, %f]", q.x(), q.y(), q.z(), q.w());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "calculate_tf");
  ros::NodeHandle nh("~");

  ros::Rate loop_rate(1);
  while (ros::ok()) {
    publishTF();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
};
