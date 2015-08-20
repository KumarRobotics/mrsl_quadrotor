#ifndef GENERAL_UTIL_H
#define GENERAL_UTIL_H

#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/PointCloud.h>

Eigen::Vector3d getYPR(const geometry_msgs::Quaternion& q)
{
  double q0 = q.w;
  double q1 = q.x;
  double q2 = q.y;
  double q3 = q.z;
  Eigen::Vector3d ypr;
  ypr(0) = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
  ypr(1) = asin(2*(q0*q2 - q3*q1));
  ypr(2) = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
  return ypr;
}

Eigen::Quaterniond getQuaternionFromYPR(const Eigen::Vector3d& ypr)
{
  double r2 = ypr(2)/2;
  double p2 = ypr(1)/2;
  double y2 = ypr(0)/2;
  Eigen::Quaterniond q(cos(r2)*cos(p2)*cos(y2) + sin(r2)*sin(p2)*sin(y2),
                       sin(r2)*cos(p2)*cos(y2) - cos(r2)*sin(p2)*sin(y2),
                       cos(r2)*sin(p2)*cos(y2) + sin(r2)*cos(p2)*sin(y2),
                       cos(r2)*cos(p2)*sin(y2) - sin(r2)*sin(p2)*cos(y2));
  return q;
}

double getYawFromEigen(const Eigen::Affine3d& dTF)
{
  Eigen::Matrix3d m = dTF.rotation();
  Eigen::Quaterniond q(m);
  double yaw = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y()+q.z()*q.z()));
  return yaw;
} 


void printTF(const Eigen::Affine3d& TF)
{     
  geometry_msgs::Transform transform;
  tf::transformEigenToMsg(TF, transform);
  double q0 = transform.rotation.w;
  double q1 = transform.rotation.x;
  double q2 = transform.rotation.y;
  double q3 = transform.rotation.z;
  Eigen::Vector3d ypr;
  ypr(0) = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
  ypr(1) = asin(2*(q0*q2 - q3*q1));
  ypr(2) = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));

  printf("x: %f, y: %f, z: %f\n", transform.translation.x, transform.translation.y, transform.translation.z);
  printf("yaw: %f, pitch: %f, roll: %f\n", ypr(0), ypr(1), ypr(2));
}     

geometry_msgs::Pose transferEigenToPose(const Eigen::Affine3d& TF)
{
  geometry_msgs::Transform transform;
  tf::transformEigenToMsg(TF, transform);
  geometry_msgs::Pose pose;
  pose.position.x = transform.translation.x; 
  pose.position.y = transform.translation.y; 
  pose.position.z = transform.translation.z; 
  pose.orientation.w = transform.rotation.w;
  pose.orientation.x = transform.rotation.x;
  pose.orientation.y = transform.rotation.y;
  pose.orientation.z = transform.rotation.z;
  return pose;
}
#endif
