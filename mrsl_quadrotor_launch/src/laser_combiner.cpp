#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> Policy_Laser;
typedef message_filters::Synchronizer<Policy_Laser> SYNC_Laser;
ros::Publisher scan_pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan1,
                  const sensor_msgs::LaserScan::ConstPtr& scan2)
{
  sensor_msgs::LaserScan scan_out;
  scan_out = *scan1;

  for(int k = 972; k <= 982; k++)
    scan_out.ranges[k] = scan2->ranges[k - 972];

  scan_pub.publish(scan_out);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_combiner");
  ros::NodeHandle nh("~");
  scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_out", 10);
  SYNC_Laser* sync_laser = new SYNC_Laser(Policy_Laser(100));
  message_filters::Subscriber<sensor_msgs::LaserScan>  scan1_sub;
  message_filters::Subscriber<sensor_msgs::LaserScan>  scan2_sub;

  scan1_sub.subscribe(nh, "scan1_in", 5);
  scan2_sub.subscribe(nh, "scan2_in", 5);

  sync_laser->connectInput(scan1_sub, scan2_sub);
  sync_laser->registerCallback(scanCallback);

  ros::spin();
  return 0;
}

     
