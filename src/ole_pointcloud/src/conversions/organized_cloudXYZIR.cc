
#include <ole_pointcloud/organized_cloudXYZIR.h>

namespace ole_pointcloud
{

  OrganizedCloudXYZIR::OrganizedCloudXYZIR(
      const double max_range, const double min_range,
      const std::string& target_frame, const std::string& fixed_frame,
      const unsigned int num_lasers, const unsigned int scans_per_block,
      boost::shared_ptr<tf::TransformListener> tf_ptr)
    : DataContainerBase(
        max_range, min_range, target_frame, fixed_frame,
        num_lasers, 0, false, scans_per_block, tf_ptr, 5,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32,
        "ring", 1, sensor_msgs::PointField::UINT16),
        iter_x(cloud, "x"), iter_y(cloud, "y"), iter_z(cloud, "z"),
        iter_intensity(cloud, "intensity"), iter_ring(cloud, "ring")
  {
  }

  void OrganizedCloudXYZIR::newLine()
  {
    iter_x = iter_x + config_.init_width;
    iter_y = iter_y + config_.init_width;
    iter_z = iter_z + config_.init_width;
    iter_ring = iter_ring + config_.init_width;
    iter_intensity = iter_intensity + config_.init_width;
    ++cloud.height;
  }

  void OrganizedCloudXYZIR::setup(const ole_msgs::oleScan::ConstPtr& scan_msg){
    DataContainerBase::setup(scan_msg);
    iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
    iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
    iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
    iter_intensity = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
    iter_ring = sensor_msgs::PointCloud2Iterator<uint16_t >(cloud, "ring");
  }


  void OrganizedCloudXYZIR::addPoint(float x, float y, float z,
      const uint16_t ring, const uint16_t /*azimuth*/, const float distance, const float intensity)
  {
    /** The laser values are not ordered, the organized structure
     * needs ordered neighbour points. The right order is defined
     * by the laser_ring value.
     * To keep the right ordering, the filtered values are set to
     * NaN.
     */
    if (pointInRange(distance))
    {
      if(config_.transform)
        transformPoint(x, y, z);

      *(iter_x+ring) = x;
      *(iter_y+ring) = y;
      *(iter_z+ring) = z;
      *(iter_intensity+ring) = intensity;
      *(iter_ring+ring) = ring;
    }
    else
    {
      *(iter_x+ring) = nanf("");
      *(iter_y+ring) = nanf("");
      *(iter_z+ring) = nanf("");
      *(iter_intensity+ring) = nanf("");
      *(iter_ring+ring) = ring;
    }
  }
}

