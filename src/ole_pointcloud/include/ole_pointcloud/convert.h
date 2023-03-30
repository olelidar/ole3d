#ifndef ole_pointcloud_CONVERT_H
#define ole_pointcloud_CONVERT_H

#include <string>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <sensor_msgs/PointCloud2.h>
#include <ole_pointcloud/rawdata.h>

#include <dynamic_reconfigure/server.h>
#include <ole_pointcloud/CloudNodeConfig.h>

namespace ole_pointcloud
{
class Convert
{
  public:
    Convert(
        ros::NodeHandle node,
        ros::NodeHandle private_nh,
        std::string const & node_name = ros::this_node::getName());
    ~Convert() {}

  private:
    void callback(ole_pointcloud::CloudNodeConfig &config, uint32_t level);
    void processScan(const ole_msgs::oleScan::ConstPtr &scanMsg);

    boost::shared_ptr<dynamic_reconfigure::Server<ole_pointcloud::CloudNodeConfig> > srv_;

    boost::shared_ptr<ole_rawdata::RawData> data_;
    ros::Subscriber ole_scan_;
    ros::Publisher output_;

    boost::shared_ptr<ole_rawdata::DataContainerBase> container_ptr_;

    boost::mutex reconfigure_mtx_;

    /// configuration parameters
    typedef struct
    {
      std::string target_frame;      ///< target frame
      std::string fixed_frame;       ///< fixed frame
      bool organize_cloud;           ///< enable/disable organized cloud structure
      double max_range;              ///< maximum range to publish
      double min_range;              ///< minimum range to publish
      uint16_t num_lasers;           ///< number of lasers
      int npackets;                  ///< number of packets to combine
    }
    Config;
    Config config_;
    bool first_rcfg_call;


  // diagnostics updater
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
};
}  // namespace ole_pointcloud

#endif  // ole_pointcloud_CONVERT_H
