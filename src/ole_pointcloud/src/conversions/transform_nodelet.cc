#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "ole_pointcloud/transform.h"

namespace ole_pointcloud
{
  class TransformNodelet: public nodelet::Nodelet
  {
  public:

    TransformNodelet() {}
    ~TransformNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<Transform> tf_;
  };

  /** @brief Nodelet initialization. */
  void TransformNodelet::onInit()
  {
    tf_.reset(new Transform(getNodeHandle(), getPrivateNodeHandle(), getName()));
  }

} // namespace ole_pointcloud 
PLUGINLIB_EXPORT_CLASS(ole_pointcloud::TransformNodelet, nodelet::Nodelet)
