#ifndef ole_LASERSCAN_ole_LASERSCAN_H
#define ole_LASERSCAN_ole_LASERSCAN_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

#include <dynamic_reconfigure/server.h>
#include <ole_laserscan/oleLaserScanConfig.h>

namespace ole_laserscan
{

class oleLaserScan
{
public:
  oleLaserScan(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);

private:
  boost::mutex connect_mutex_;
  void connectCb();
  void recvCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  oleLaserScanConfig cfg_;
  dynamic_reconfigure::Server<oleLaserScanConfig> srv_;
  void reconfig(oleLaserScanConfig& config, uint32_t level);

  unsigned int ring_count_;
};

}  // namespace ole_laserscan

#endif  // ole_LASERSCAN_ole_LASERSCAN_H
