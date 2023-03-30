#include <ros/ros.h>
#include "ole_driver/driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ole_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // start the driver
  ole_driver::oleDriver dvr(node, private_nh);

  // loop until shut down or end of file
  while (ros::ok())
  {
    // poll device until end of file
    bool polled_ = dvr.poll();
    if (!polled_) ROS_ERROR_THROTTLE(1.0, "ole - Failed to poll device.");

    ros::spinOnce();
  }

  return 0;
}
