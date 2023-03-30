
#ifndef ole_DRIVER_DRIVER_H
#define ole_DRIVER_DRIVER_H

#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>

#include <ole_driver/input.h>
#include <ole_driver/oleNodeConfig.h>

namespace ole_driver
{

class oleDriver
{
public:
  oleDriver(ros::NodeHandle node, ros::NodeHandle private_nh, std::string const & node_name = ros::this_node::getName());
  ~oleDriver() {}

  bool poll(void);

private:
  // Callback for dynamic reconfigure
  void callback(ole_driver::oleNodeConfig &config, uint32_t level);
  // Callback for diagnostics update for lost communication with vlp
  void diagTimerCallback(const ros::TimerEvent&event);

  // Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<ole_driver::oleNodeConfig> > srv_;

  // configuration parameters
  struct
  {
    std::string frame_id;            // tf frame ID
    std::string model;               // device model name
    int    npackets;                 // number of packets to collect
    double rpm;                      // device rotation rate (RPMs)
    int cut_angle;                   // cutting angle in 1/100°
    double time_offset;              // time in seconds added to each ole time stamp
    bool enabled;                    // polling is enabled
    bool timestamp_first_packet;
  }
  config_;

  boost::shared_ptr<Input> input_;
  ros::Publisher output_;
  int last_azimuth_;

  // added by zyl for split frame
  ole_msgs::olePacket pre_split_packet_;
  int pre_length_;
  int post_length_;
  /* diagnostics updater */
  ros::Timer diag_timer_;
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
};

}  // namespace ole_driver

#endif  // ole_DRIVER_DRIVER_H
