/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw ole 3D LIDAR packets to PointCloud2.

*/

#include "ole_pointcloud/convert.h"

#include <ole_pointcloud/pointcloudXYZIR.h>
#include <ole_pointcloud/organized_cloudXYZIR.h>
// for debug
#include <chrono>
#include <sensor_msgs/point_cloud2_iterator.h>


namespace ole_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh, std::string const & node_name):
    data_(new ole_rawdata::RawData()), first_rcfg_call(true),
    diagnostics_(node, private_nh, node_name)
  {
    //
    boost::optional<ole_pointcloud::Calibration> calibration = data_->setup(private_nh);
    if(calibration)
    {
        ROS_DEBUG_STREAM("Calibration file loaded.");
        config_.num_lasers = static_cast<uint16_t>(calibration.get().num_lasers);
    }
    else
    {
        ROS_ERROR_STREAM("Could not load calibration file!");
    }

    if(config_.organize_cloud)
    {
      container_ptr_ = boost::shared_ptr<OrganizedCloudXYZIR>(
          new OrganizedCloudXYZIR(config_.max_range, config_.min_range,
              config_.target_frame, config_.fixed_frame,
              config_.num_lasers, data_->scansPerPacket()));
    }
    else
    {
      container_ptr_ = boost::shared_ptr<PointcloudXYZIR>(
          new PointcloudXYZIR(config_.max_range, config_.min_range,
              config_.target_frame, config_.fixed_frame,
              data_->scansPerPacket()));
    }


    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("olei_points", 10);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<ole_pointcloud::
      CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<ole_pointcloud::CloudNodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to oleScan packets
    ole_scan_ =
      node.subscribe("ole_packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));

    // Diagnostics
    diagnostics_.setHardwareID("ole Convert");
    // Arbitrary frequencies since we don't know which RPM is used, and are only
    // concerned about monitoring the frequency.
    diag_min_freq_ = 2.0;
    diag_max_freq_ = 20.0;
    using namespace diagnostic_updater;
    diag_topic_.reset(new TopicDiagnostic("olei_points", diagnostics_,
                                       FrequencyStatusParam(&diag_min_freq_,
                                                            &diag_max_freq_,
                                                            0.1, 10),
                                       TimeStampStatusParam()));
  }

  void Convert::callback(ole_pointcloud::CloudNodeConfig &config,
                uint32_t level)
  {
    ROS_INFO("Reconfigure Request");
    data_->setParameters(config.min_range, config.max_range, config.view_direction,
                         config.view_width);
    config_.fixed_frame = config.fixed_frame;
    config_.target_frame = config.target_frame;
    config_.min_range = config.min_range;
    config_.max_range = config.max_range;

    if(first_rcfg_call || config.organize_cloud != config_.organize_cloud){
        first_rcfg_call = false;
        config_.organize_cloud = config.organize_cloud;
        if(config_.organize_cloud) // TODO only on change
        {
            ROS_INFO_STREAM("Using the organized cloud format...");
            container_ptr_ = boost::shared_ptr<OrganizedCloudXYZIR>(
                new OrganizedCloudXYZIR(config_.max_range, config_.min_range,
                    config_.target_frame, config_.fixed_frame,
                    config_.num_lasers, data_->scansPerPacket()));
        }
        else
        {
            container_ptr_ = boost::shared_ptr<PointcloudXYZIR>(
                new PointcloudXYZIR(config_.max_range, config_.min_range,
                    config_.target_frame, config_.fixed_frame,
                    data_->scansPerPacket()));
        }
    }

    container_ptr_->configure(config_.max_range, config_.min_range, config_.fixed_frame, config_.target_frame);

  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const ole_msgs::oleScan::ConstPtr &scanMsg)
  {
    //if (output_.getNumSubscribers() == 0)         // no one listening?
    //  return;                                     // avoid much work

    // monitor sub time and size
    auto t1=std::chrono::steady_clock::now();
    double t1_us=std::chrono::duration<double,std::micro>(t1.time_since_epoch()).count();
    size_t size_packets = scanMsg->packets.size();


    boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);
    // allocate a point cloud with same time and frame ID as raw data
    container_ptr_->setup(scanMsg);

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size()-1 ; ++i)
    {
      data_->unpack(scanMsg->packets[i], *container_ptr_,i);
    }
    // the last packet
    size_t i = scanMsg->packets.size()-1;
    data_->unpack(scanMsg->packets[i], *container_ptr_,1000);

    // publish the accumulated cloud message
    diag_topic_->tick(scanMsg->header.stamp);
    diagnostics_.update();

    sensor_msgs::PointCloud2 cloud= container_ptr_->finishCloud();
    output_.publish(cloud);
    //output_.publish(container_ptr_->finishCloud());

    auto t2=std::chrono::steady_clock::now();
    double dt_us=std::chrono::duration<double,std::micro>(t2-t1).count();


    //std::cout << "processScan Start: " << std::to_string(t1_us)
    //          << ":" << std::to_string(dt_us)
    //          << ":" <<  std::to_string(size_packets)
    //         << ":" <<  std::to_string(cloud.width)
    //         << std::endl;

  }

} // namespace ole_pointcloud
