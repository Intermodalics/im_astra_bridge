#include <Astra/Astra.h>
#include <AstraUL/AstraUL.h>
#include <chrono>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <string>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PclPointCloud;
typedef sensor_msgs::PointCloud2::Ptr RosPointCloud;

class PointFrameListener : public astra::FrameReadyListener {
  public:
    // A changing pointer to const 3d point clouds.
    const astra::Vector3f* pointData;

    // Width and height of the 3d point cloud.
    size_t width;
    size_t height;

    PointFrameListener() :
      pointData(), 
      width(0), 
      height(0)
      {}

    virtual void on_frame_ready(astra::StreamReader& reader,
      astra::Frame& frame) override {

        // Read a new 3d point frame.
        astra::PointFrame pointFrame = frame.get<astra::PointFrame>();

        if (pointFrame.is_valid()) {

          astra::Frame frame = reader.get_latest_frame();
          astra::PointFrame pointFrame = frame.get<astra::PointFrame>();

          // Resolution of the astra 3d point frame.
          this->width = pointFrame.resolutionX();
          this->height = pointFrame.resolutionY();

          //const astra::Vector3f* pointData = pointFrame.data();
          this->pointData = pointFrame.data();

        } else {
          ROS_ERROR_ONCE("Point frame is not valid.");
        }
      }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "astra_bridge");
  ros::NodeHandle n("~");

  // Init astra.
  astra::Astra::initialize();

  // Create astra stream reader.
  astra::StreamSet streamset;
  astra::StreamReader reader = streamset.create_reader();
  
  // Start the astra point stream reader.
  reader.stream<astra::DepthStream>().start();
  reader.stream<astra::PointStream>().start();
 
  // Add 3d point cloud listener to point stream reader.
  PointFrameListener listener;
  reader.addListener(listener);

  // Create the publishers.
  ros::Publisher cloud_pub = 
    n.advertise<sensor_msgs::PointCloud2>("depth/points", 1);

  // Set the loop rate.
  ros::Rate loop_rate(30);

  ros::Duration(1.0).sleep();
  ros::spinOnce();

  while (ros::ok()){
    // Update 3d point frame listener.
    astra_temp_update();

    // Create new point cloud.
    PclPointCloud cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->header.frame_id = "base_link";
    const size_t width = listener.width;
    const size_t height = listener.height;
    const astra::Vector3f* pointData = listener.pointData;
    cloud->width = width;
    cloud->height = height;
    //cloud->is_dense = false;
    cloud->points.resize(width * height);

    if (width == 0 || height == 0) {
      ROS_ERROR_ONCE("3d point cloud size is zero.\n");
    }

    // Fill point cloud with 3d points from orbbec camera.
    for (unsigned y = 0; y < height; ++y) {
      for (unsigned x = 0; x < width; ++x, ++pointData) {
        cloud->points[(y * width + x)].x = (*pointData).x / 1000.0;
        cloud->points[(y * width + x)].y = (*pointData).y / 1000.0;
        cloud->points[(y * width + x)].z = (*pointData).z / 1000.0;
      }
    }
    // Convert to sensormsg::Pointcloud2 type.
    RosPointCloud final_cloud (new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud, *final_cloud);
    final_cloud->header.frame_id = "base_link";
    final_cloud->header.stamp = ros::Time::now();
    cloud_pub.publish(final_cloud);
    
  }

  // Close astra point stream.
  astra::Astra::terminate();

  return 0;
}
