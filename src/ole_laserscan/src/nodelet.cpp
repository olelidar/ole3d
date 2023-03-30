#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "ole_laserscan/ole_laserscan.h"

namespace ole_laserscan
{

class LaserScanNodelet: public nodelet::Nodelet
{
public:
  LaserScanNodelet() {}
  ~LaserScanNodelet() {}

private:
  virtual void onInit()
  {
    node_.reset(new oleLaserScan(getNodeHandle(), getPrivateNodeHandle()));
  }

  boost::shared_ptr<oleLaserScan> node_;
};

}  // namespace ole_laserscan

PLUGINLIB_EXPORT_CLASS(ole_laserscan::LaserScanNodelet, nodelet::Nodelet);
