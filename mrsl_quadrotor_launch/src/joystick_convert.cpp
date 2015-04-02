#include <ros/ros.h>

ros::Publisher cmd_pub;

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick_convert");
  ros::NodeHandle nh("~");
  ros::Subscriber sub = nh.subscribe("cmd_vel", 10, &cmdCallback);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud_out", 10);

  ros::spin();
  return 0;
}


