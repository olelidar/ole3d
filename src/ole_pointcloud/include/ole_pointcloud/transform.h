// Copyright (C) 2009, 2010, 2011, 2012, 2019 Austin Robot Technology, Jack O'Quin, Jesse Vera, Joshua Whitley,
// Sebastian Pütz All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** @file

    This class transforms raw ole 3D LIDAR packets to PointCloud2
    in the /map frame of reference.

*/

#ifndef ole_pointcloud_TRANSFORM_H
#define ole_pointcloud_TRANSFORM_H

#include <string>
#include <ros/ros.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/PointCloud2.h>

#include <ole_pointcloud/rawdata.h>
#include <ole_pointcloud/pointcloudXYZIR.h>

#include <dynamic_reconfigure/server.h>
#include <ole_pointcloud/TransformNodeConfig.h>

namespace ole_pointcloud
{
using TransformNodeCfg = ole_pointcloud::TransformNodeConfig;

class Transform
{
public:
  Transform(
      ros::NodeHandle node,
      ros::NodeHandle private_nh,
      std::string const & node_name = ros::this_node::getName());
  ~Transform()
  {
  }

private:
  void processScan(const ole_msgs::oleScan::ConstPtr& scanMsg);

  // Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<ole_pointcloud::TransformNodeConfig>> srv_;
  void reconfigure_callback(ole_pointcloud::TransformNodeConfig& config, uint32_t level);

  const std::string tf_prefix_;
  boost::shared_ptr<ole_rawdata::RawData> data_;
  message_filters::Subscriber<ole_msgs::oleScan> ole_scan_;
  ros::Publisher output_;
  boost::shared_ptr<tf::MessageFilter<ole_msgs::oleScan>> tf_filter_ptr_;
  boost::shared_ptr<tf::TransformListener> tf_ptr_;

  /// configuration parameters
  typedef struct
  {
    std::string target_frame;  ///< target frame
    std::string fixed_frame;   ///< fixed frame
    bool organize_cloud;       ///< enable/disable organized cloud structure
    double max_range;          ///< maximum range to publish
    double min_range;          ///< minimum range to publish
    uint16_t num_lasers;       ///< number of lasers
  }
  Config;
  Config config_;

  bool first_rcfg_call;

  boost::shared_ptr<ole_rawdata::DataContainerBase> container_ptr;

  // diagnostics updater
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
  boost::mutex reconfigure_mtx_;
};
}  // namespace ole_pointcloud

#endif  // ole_pointcloud_TRANSFORM_H
