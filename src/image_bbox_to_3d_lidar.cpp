#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace sensor_msgs;
using namespace message_filters;

typedef pcl::PointXYZI PointT;

void callback(const ImageConstPtr& image1, const PointCloud2ConstPtr msg)
{
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*msg, *cloud);

  // Solve all of perception here...sensor_msgs::
  printf("image received \n");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<Image> image1_sub(nh, "/Multisense/left/image_rect_color", 1);
  message_filters::Subscriber<PointCloud2> pc2_sub(nh, "/velodyne_points", 1);

  typedef sync_policies::ApproximateTime<Image, PointCloud2> MySyncPolicy;

  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, pc2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
