#include <ros/ros.h>
#include "ole_laserscan/ole_laserscan.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ole_laserscan_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // create oleLaserScan class
  ole_laserscan::oleLaserScan n(nh, nh_priv);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
