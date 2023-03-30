#ifndef ole_pointcloud_POINTCLOUDXYZIR_H
#define ole_pointcloud_POINTCLOUDXYZIR_H

#include <ole_pointcloud/datacontainerbase.h>
#include <string>

namespace ole_pointcloud
{
class PointcloudXYZIR : public ole_rawdata::DataContainerBase
{
public:
  PointcloudXYZIR(const double max_range, const double min_range, const std::string& target_frame,
                  const std::string& fixed_frame, const unsigned int scans_per_block,
                  boost::shared_ptr<tf::TransformListener> tf_ptr = boost::shared_ptr<tf::TransformListener>());

  virtual void newLine();

  virtual void setup(const ole_msgs::oleScan::ConstPtr& scan_msg);

  virtual void addPoint(float x, float y, float z, uint16_t ring, uint16_t azimuth, float distance, float intensity);

  sensor_msgs::PointCloud2Iterator<float> iter_x, iter_y, iter_z, iter_intensity;
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring;
};
}  // namespace ole_pointcloud

#endif  // ole_pointcloud_POINTCLOUDXYZIR_H
